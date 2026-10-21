//
// 机械臂按键输入层实现（边沿事件 + 按键决策 + 菜单 + 末端点动）
//

#include "arm_button.h"

#include "arm_action.h"
#include "arm_control.h"
#include "arm_endpoint.h"
#include "arm_manual.h"     // ARM_MANUAL_INTERRUPT_DEADZONE（打断阈值）
#include "arm_shake.h"
#include "control_mode.h"
#include "gripper.h"

namespace {

/* --------------------------- 边沿检测内部状态 --------------------------- */
uint16_t s_previous_buttons = 0U;   // 上一次喂进来的按键状态（= 当前按住哪些）
uint16_t s_pending_events = 0U;     // 尚未被取走的"新按下"事件
bool s_connected = false;

/* ---------------------------- 摇杆（打断用） ---------------------------- */
float s_stick[4] = { 0.0f, 0.0f, 0.0f, 0.0f };   // 底座/肩/肘/夹爪
bool s_sticks_valid = false;

/* ------------------------------- UI 模式 ------------------------------- */
bool s_menu = false;                 // true = 动作选择菜单
bool s_endpoint_active = false;      // 本周期十字键正在做末端点动

/* --------------------------- 动作选择菜单的槽位 ---------------------------
 * 机械臂模式下按 R1 进入菜单，此时 8 个键（左十字键 4 个 + 右面键 4 个）各触发
 * 一个动作；无论按的是槽位键还是 R1，都退出菜单回到正常态（一次性菜单）。
 * 占位槽位（ActionId::kCount）按下也会退出菜单，只是不触发动作。
 *
 * 新增动作时改三处：① arm_action.h 加 ActionId；② arm_action.cpp 的
 * kSequences[] 登记路径（取放点关键帧加在 arm_keyframe）；③ 下面对一行。 */
struct MenuSlot {
    uint16_t button;     // 触发它的键
    ActionId action;     // 绑定的动作（ActionId::kCount = 占位）
};

const MenuSlot kMenuSlots[] = {
    { kArmBtnTriangle, ActionId::kEnergyCorePlatform },   // 右·上 △ 能量核心_高台
    { kArmBtnCircle,   ActionId::kEnergyUnitPlatRight },  // 右·右 ○ 能量单元_高台右
    { kArmBtnCross,    ActionId::kCount },                // 右·下 × 占位
    { kArmBtnSquare,   ActionId::kEnergyUnitPlatLeft },   // 右·左 □ 能量单元_高台左
    { kArmBtnPadUp,    ActionId::kEnergyCoreShelf },      // 左·上 能量核心_物资架
    { kArmBtnPadRight, ActionId::kCount },                // 左·右 占位
    { kArmBtnPadDown,  ActionId::kFrontLow },             // 左·下 前方低点
    { kArmBtnPadLeft,  ActionId::kEnergyUnitShelfLeft },  // 左·左 能量单元_物资架下左
};

/** 任一摇杆轴越过打断阈值 = 操作员想手动接管 */
bool sticks_beyond_interrupt(void)
{
    if (!s_sticks_valid) {
        return false;
    }
    for (uint32_t i = 0; i < 4U; i++) {
        if (s_stick[i] > ARM_MANUAL_INTERRUPT_DEADZONE ||
            s_stick[i] < -ARM_MANUAL_INTERRUPT_DEADZONE) {
            return true;
        }
    }
    return false;
}

/* --------------------- 十字键 → 末端点动方向（长按） ---------------------
 * 方向约定只在这里定义，实物反了就改这里的两行符号：
 *   ↑ = 末端上升（+z）   ↓ = 末端下降（−z）
 *   ← = 末端向前伸出（+r，远离底座）   → = 末端收回（−r，靠近底座） */
float pad_axis_dz(void)
{
    float dz = 0.0f;
    if ((s_previous_buttons & kArmBtnPadUp) != 0U) {
        dz += 1.0f;
    }
    if ((s_previous_buttons & kArmBtnPadDown) != 0U) {
        dz -= 1.0f;
    }
    return dz;
}

float pad_axis_dr(void)
{
    float dr = 0.0f;
    if ((s_previous_buttons & kArmBtnPadLeft) != 0U) {
        dr += 1.0f;
    }
    if ((s_previous_buttons & kArmBtnPadRight) != 0U) {
        dr -= 1.0f;
    }
    return dr;
}

/** 菜单态的 8 个槽位分派（一次性菜单：按任意槽位键都退出菜单） */
void dispatch_menu(uint16_t events, uint32_t now_ms)
{
    for (uint32_t i = 0; i < (uint32_t)(sizeof(kMenuSlots) / sizeof(kMenuSlots[0])); i++) {
        const MenuSlot &slot = kMenuSlots[i];
        if ((events & slot.button) == 0U) {
            continue;
        }

        /* 按了任意槽位键：先退出菜单，回到正常态（2026/10/21 需求） */
        s_menu = false;

        if (slot.action == ActionId::kCount) {
            return;       // 占位槽位：只退出菜单，不触发动作
        }
        /* 与正常模式的动作键同规矩（E5）：运行中再按 = 打断并停在当前位置 */
        if (arm_action_get_state() == ActionState::kRunning) {
            arm_action_cancel();
            return;
        }
        if (arm_shake_is_running()) {
            arm_shake_cancel();   // 抖动中选动作 = 先停抖动（肘交给动作）
            return;
        }
        (void)arm_action_start(slot.action, now_ms);
        return;
    }
}

/* ---------------- L2：卸力 ⇄ 恢复 + 逐路给力的"脱离"（E4） ----------------
 * 卸力：四路一起软下来，可徒手摆位。
 * 恢复：**不再**先把整条臂弹回卸力前姿态，而是交给"脱离"动作按顺序
 *       逐路给力——先只给大臂（肩）通电把它撑直到 90°（底座/肘/夹爪仍是
 *       软的，臂能自己让位），再让大臂保持、底座单独回中位，收尾时才把
 *       肘（小臂）与夹爪一起给力。详见 arm_keyframe.cpp 的脱离两帧。
 * 动作运行中按 L2 = 停止并立即卸力（一次按键完成"停 + 卸"）；
 * 只想停住而不卸力，推杆或按 SELECT。
 * **两种 UI 模式（正常/菜单）下都有效**：卸力是安全相关功能，不能因为
 * 进了动作菜单就按不出来。 */
void handle_relax_toggle(uint32_t now_ms)
{
    if (arm_is_relaxed()) {
        /* 恢复：先退出卸力模式（此时仍不给力），再启动脱离动作 */
        arm_set_relaxed(false);
        if (arm_action_get_state() == ActionState::kRunning) {
            arm_action_cancel();
        }
        if (!arm_action_start(ActionId::kEscape, now_ms)) {
            /* 兜底：脱离动作没起来（读回失败等），就直接整臂给力，
             * 否则会停在"模式已退出、却没有任何一路有力"的死状态。 */
            arm_set_torque_enabled(true);
        }
    } else {
        /* 卸力：先进入卸力模式再取消动作——顺序反了的话，取消动作的
         * 收尾会把刚软下来的关节重新给力。 */
        arm_set_relaxed(true);
        if (arm_action_get_state() == ActionState::kRunning) {
            arm_action_cancel();
        }
        arm_shake_cancel();   // 卸力也停掉末端抖动（恢复夹爪的写入被拒绝，无害）
    }
}

/** 正常模式的按键决策（L1 / ○ / △ / ×；L2 与 SELECT 在 apply 里统一处理） */
void handle_normal_mode(uint16_t events, uint32_t now_ms)
{
    /* 卸力状态下其余按键一律忽略：舵机写入会被拒绝，提前拦截，
     * 避免夹爪/动作状态被悄悄改成与实际不符的值。 */
    if (arm_is_relaxed()) {
        return;
    }

    /* ---------------- L1：点动两档调速（不参与打断） ----------------
     * 只影响手动点动速度，对正在执行的动作没有影响，所以放在打断判定之前。 */
    if ((events & kArmBtnL1) != 0U) {
        arm_manual_cycle_speed_level();
        return;
    }

    /* ---------------- ○：夹爪两档切换（E1） ----------------
     * 动作/抖动运行中先取消（否则关键帧与抖动会覆盖夹爪指令），
     * 臂停在当前位置。两档切换在任意位置都成立，所以这里不判断
     * 当前是否在档位上。 */
    if ((events & kArmBtnCircle) != 0U) {
        if (arm_action_get_state() == ActionState::kRunning) {
            arm_action_cancel();
        }
        if (arm_shake_is_running()) {
            arm_shake_cancel();
        }
        (void)gripper_toggle();
        return;
    }

    /* ---------------- □：末端抖动（夹爪快速开合 + 小臂小幅摆动） ----------------
     * 只在正常模式绑定（菜单态下 □ 是动作槽位）。打断规矩与动作键相同：
     * 抖动中再按 = 停；动作运行中按 = 先停动作。 */
    if ((events & kArmBtnSquare) != 0U) {
        if (arm_shake_is_running()) {
            arm_shake_cancel();
            return;
        }
        if (arm_action_get_state() == ActionState::kRunning) {
            arm_action_cancel();
            return;
        }
        (void)arm_shake_start(now_ms);
        return;
    }

    /* ---------------- 动作键：△ 回收缩（E2） / × 转移（E3） ----------------
     * 回收缩原绑定在十字键↑，2026/10/21 改到 △；十字键现在用于末端点动。 */
    ActionId action_id;
    if ((events & kArmBtnTriangle) != 0U) {
        action_id = ActionId::kRetract;
    } else if ((events & kArmBtnCross) != 0U) {
        action_id = ActionId::kTransit;
    } else {
        return;     // 其余按键本次不绑定动作
    }

    /* 抖动中按动作键 = 先停抖动（肘由动作接管），下次再按才启动动作 */
    if (arm_shake_is_running()) {
        arm_shake_cancel();
        return;
    }

    /* E5：动作运行中再按动作键 = 打断，臂停在当前位置（不重开新动作） */
    if (arm_action_get_state() == ActionState::kRunning) {
        arm_action_cancel();
        return;
    }

    (void)arm_action_start(action_id, now_ms);
}

}   // namespace

void arm_button_init(void)
{
    s_previous_buttons = 0U;
    s_pending_events = 0U;
    s_connected = false;
    s_sticks_valid = false;
    s_menu = false;
    s_endpoint_active = false;
    for (uint32_t i = 0; i < 4U; i++) {
        s_stick[i] = 0.0f;
    }
}

void arm_button_feed(uint16_t buttons_pressed, bool connected)
{
    if (!connected) {
        /* 断连：清掉边沿与残留事件，重连后不会补发 */
        s_previous_buttons = 0U;
        s_pending_events = 0U;
        s_connected = false;
        return;
    }

    /* 刚连上时只记录当前按键：连接前就按住的键不算"新按下" */
    if (!s_connected) {
        s_previous_buttons = buttons_pressed;
        s_connected = true;
        return;
    }

    /* 边沿 = 本周期按下 且 上周期未按下；多周期锁存直到被取走 */
    s_pending_events |= static_cast<uint16_t>(
        buttons_pressed & static_cast<uint16_t>(~s_previous_buttons));
    s_previous_buttons = buttons_pressed;
}

uint16_t arm_button_take_events(void)
{
    const uint16_t events = s_pending_events;
    s_pending_events = 0U;
    return events;
}

void arm_button_feed_sticks(float base_norm, float shoulder_norm,
                            float elbow_norm, float gripper_norm)
{
    s_stick[0] = base_norm;
    s_stick[1] = shoulder_norm;
    s_stick[2] = elbow_norm;
    s_stick[3] = gripper_norm;
    s_sticks_valid = true;
}

bool arm_button_in_menu(void)
{
    return s_menu;
}

bool arm_button_is_endpoint_jog_active(void)
{
    return s_endpoint_active;
}

void arm_button_apply(uint32_t now_ms)
{
    const bool arm_mode = (control_mode_get() == CONTROL_MODE_ARM);

    /* 不在机械臂模式：菜单态复位，回到机械臂模式时从"正常"开始 */
    if (!arm_mode) {
        s_menu = false;
    }

    /* ------------------- E5：摇杆打断自动运动 -------------------
     * 人手动推杆是最直接的意图表达，优先级最高：正在执行的动作
     * （含脱离/收缩/转移）与末端抖动立即停在当前位置。两种 UI 模式下都生效。 */
    if (arm_action_get_state() == ActionState::kRunning && sticks_beyond_interrupt()) {
        arm_action_cancel();
    }
    if (arm_shake_is_running() && sticks_beyond_interrupt()) {
        arm_shake_cancel();
    }

    /* ------------------- 十字键长按：末端柱坐标点动 -------------------
     * 仅"机械臂模式 + 正常 UI 模式 + 未卸力"时响应；菜单态下十字键是动作槽位。
     * 与"再次动作键"一样属于手动接管：按下去若正在跑动作/抖动，先停掉。 */
    const bool jog_enabled = arm_mode && !s_menu && !arm_is_relaxed();
    const float dz = jog_enabled ? pad_axis_dz() : 0.0f;
    const float dr = jog_enabled ? pad_axis_dr() : 0.0f;

    s_endpoint_active = (dz != 0.0f) || (dr != 0.0f);
    if (s_endpoint_active) {
        if (arm_action_get_state() == ActionState::kRunning) {
            arm_action_cancel();
        }
        if (arm_shake_is_running()) {
            arm_shake_cancel();
        }
        (void)arm_endpoint_jog(now_ms, dz, dr);
    } else {
        (void)arm_endpoint_jog(now_ms, 0.0f, 0.0f);   // 空闲：复位内部时间戳
    }

    /* 事件每周期都取走——即使断连、即使当前不在机械臂模式。
     * 否则底盘模式下按过的键会在切回机械臂模式时一次性触发。 */
    const uint16_t events = arm_button_take_events();

    /* ---------------- SELECT：取消一切自动运动 ----------------
     * 两种操作对象模式、两种 UI 模式下都生效：它是"停止"语义。 */
    if ((events & kArmBtnSelect) != 0U) {
        arm_action_cancel();
        arm_shake_cancel();
        return;
    }

    /* 其余按键只在机械臂模式生效（事件已取走并丢弃，不残留） */
    if (!arm_mode) {
        return;
    }

    /* ---------------- L2：卸力 ⇄ 恢复（两种 UI 模式下都有效，安全相关） ----------------
     * 放在 R1/菜单判定之前：进了动作菜单也一定要能卸力。 */
    if ((events & kArmBtnL2) != 0U) {
        handle_relax_toggle(now_ms);
        return;
    }

    /* ---------------- R1：正常 ⇄ 动作选择菜单 ----------------
     * 只用 R2 切"底盘 ⇄ 机械臂"，R1 专管机械臂内部这个二级菜单。 */
    if ((events & kArmBtnR1) != 0U) {
        s_menu = !s_menu;
        return;
    }

    /* 卸力状态下其余按键（含菜单槽位）一律忽略：舵机写入会被拒绝 */
    if (arm_is_relaxed()) {
        return;
    }

    /* ---------------- 菜单态：8 个槽位 ---------------- */
    if (s_menu) {
        dispatch_menu(events, now_ms);
        return;
    }

    /* ---------------- 正常态：原有按键语义 ---------------- */
    handle_normal_mode(events, now_ms);
}
