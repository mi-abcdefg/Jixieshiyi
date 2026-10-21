//
// Created by MI on 2026/10/17.
//

// 机械臂任务实现：本文件是机械臂子系统的唯一编排层。
//
// 摇杆分配（挖掘机式关节直控，R2 切换控制对象，上电默认底盘模式）：
//   ┌──────────────┬────────────────────────┬──────────────────────────┐
//   │ 输入         │ 底盘模式                │ 机械臂模式                │
//   ├──────────────┼────────────────────────┼──────────────────────────┤
//   │ 右摇杆 左右   │ 底盘旋转                │ 底座旋转（方位角）         │
//   │ 右摇杆 上下   │ 肩抬落（两模式都有效）   │ 肩抬落                    │
//   │ 左摇杆 上下   │ 底盘前后                │ 肘抬落                    │
//   │ 左摇杆 左右   │ 底盘平移                │ 夹爪开合（点动）           │
//   └──────────────┴────────────────────────┴──────────────────────────┘
// 按键语义（E1~E5 的绑定全在 arm_button，本文件只做 PS2 → 抽象位图的映射）：
//   ○ 夹爪两档切换 / △ 回收缩 / × 篮内外转移 / L2 卸力⇄恢复（逐路给力）/
//   L1 点动两档调速 / SELECT 取消动作（两种模式都可用）
//   十字键↑ 在机械臂模式下不绑定（留给底盘模式的定距平移）。
//
// 数据流（每周期 20ms）：
//   PS2 解码结果（RemoteCommTask 每 10ms 刷新，这里只读）
//     ├─ 按键位重排 → arm_button_feed → arm_button_apply（打断/决策）
//     ├─ 摇杆归一化 → 按模式分流 → arm_button_feed_sticks（打断判定）
//     │                            → arm_manual_update（关节/夹爪点动）
//     └─ arm_action_update（动作引擎：关键帧序列推进）
//
// 分层铁律：Arm/ 不 include PS2 与 Task。PS2 原始位 → 抽象位图的映射放在
// 这里（编排层），arm_button 及以下都不认识 PS2。
//

#include "arm_task.h"

#include "ps2_decode.h"
#include "control_mode.h"
#include "arm_action.h"
#include "arm_button.h"
#include "arm_endpoint.h"
#include "arm_manual.h"
#include "gripper.h"
#include "servo_driver.h"

/* ------------------------------ 周期 ------------------------------ */
#define ARM_TASK_PERIOD_MS        20U    // 与动作引擎 20ms 指令节拍严格一致

/* PS2 模式字节：只有这两个值代表"已连上"（判据见 TestTask 顶部说明） */
#define PS2_MODE_ANALOG           0x73u
#define PS2_MODE_DIGITAL          0x41u

/* 摇杆中位与归一化满量程（解码层死区 118~138 已折回 128） */
#define STICK_CENTER              128.0f
#define STICK_RANGE               128.0f

namespace {
    /* ---------------- PS2 原始位 → 机械臂抽象位图 ----------------
     * PS2 位定义见 ps2_decode.h；kArmBtn* 是 arm_button 的统一格式。
     * 换遥控器时只改这里，arm_button 及以下都不用动。 */
    uint16_t map_ps2_to_arm_buttons(uint8_t btn1, uint8_t btn2) {
        uint16_t pressed = 0;
        if (btn1 & PS2_BTN_SELECT) pressed |= kArmBtnSelect;
        if (btn1 & PS2_BTN_UP)     pressed |= kArmBtnPadUp;
        if (btn1 & PS2_BTN_DOWN)   pressed |= kArmBtnPadDown;
        if (btn1 & PS2_BTN_LEFT)   pressed |= kArmBtnPadLeft;
        if (btn1 & PS2_BTN_RIGHT)  pressed |= kArmBtnPadRight;
        if (btn2 & PS2_BTN_L1)       pressed |= kArmBtnL1;
        if (btn2 & PS2_BTN_L2)       pressed |= kArmBtnL2;
        if (btn2 & PS2_BTN_R1)       pressed |= kArmBtnR1;
        if (btn2 & PS2_BTN_TRIANGLE) pressed |= kArmBtnTriangle;
        if (btn2 & PS2_BTN_CIRCLE)   pressed |= kArmBtnCircle;
        if (btn2 & PS2_BTN_CROSS)    pressed |= kArmBtnCross;
        if (btn2 & PS2_BTN_SQUARE)   pressed |= kArmBtnSquare;
        return pressed;
    }

    /* 摇杆原始值 → 归一化 -1..1。
     * 方向按直觉定义：上推 / 左推 = 正。原始值上推变小，所以取负号。
     * 实测方向不顺手时，改这里的符号即可，不要动下层模块。 */
    float stick_to_norm(uint8_t raw, bool inverted) {
        float v = (static_cast<float>(raw) - STICK_CENTER) / STICK_RANGE;
        return inverted ? -v : v;
    }
}

void ArmTask::run() {
    // 舵机 PWM 启动放在任务里（调度器启动后）而不是 xxx_init()：
    // 与 RemoteCommTask 里 PS2_ConfigAnalogMode() 同理，保证所有初始化
    // 都发生在调度器启动之后，时序行为一致。
    // 初始化失败时停住本任务（舵机全无输出）；LED 任务照常指示 PS2 状态，
    // 不用 Error_Handler()——那是关中断死循环，会拖死整个系统。
    if (!servo_init()) {
        for (;;) {
            osDelay(1000);
        }
    }

    // 顺序无关（都是清空各自的状态机），但都必须在本循环之前完成：
    // + servo_init() 之后，四路舵机才有可靠的"指令角"可读回。
    gripper_init();
    arm_manual_init();
    arm_endpoint_init();
    arm_action_init();
    arm_button_init();

    while (1) {
        const uint32_t now_ms = osKernelSysTick();

        /* ---------------- 手柄连接状态 ----------------
         * 模式字节是 0x73 / 0x41 才代表链路正常（与 TestTask 的判据一致） */
        const uint8_t ps2_mode = ps2_get_mode();
        const bool connected =
            (ps2_mode == PS2_MODE_ANALOG) || (ps2_mode == PS2_MODE_DIGITAL);

        /* ---------------- 按键：喂给事件层 ----------------
         * 每周期都要喂（断连时也要喂全 0），事件层才能正确维护边沿状态。 */
        arm_button_feed(map_ps2_to_arm_buttons(ps2_get_btn1(), ps2_get_btn2()),
                        connected);

        /* ---------------- 摇杆：归一化（挖掘机式关节分配） ----------------
         * 各轴正方向 = 对应关节命令角增大方向（见 arm_manual.h）。
         * 开/合、旋转等物理方向若与直觉相反，改这里的 inverted 参数。 */
        uint8_t left[2];
        uint8_t right[2];
        ps2_get_left_analog_data(left);
        ps2_get_right_analog_data(right);

        const float base_yaw  = stick_to_norm(right[0], true);  // 右X：左推=方位角+（逆时针）
        const float shoulder  = stick_to_norm(right[1], true);  // 右Y：上推=肩抬高
        const float elbow     = stick_to_norm(left[1], true);   // 左Y：上推=肘抬起
        const float gripper   = stick_to_norm(left[0], true);   // 左X：左推=夹爪角+（开/闭方向实物标定）

        /* 模式分流：肩抬落保留"两种模式都可用"（原需求：底盘模式下也能
         * 用右摇杆上下调臂）；其余三个关节是机械臂模式专属，避免与底盘
         * 的左摇杆/右摇杆左右冲突。 */
        const bool arm_mode = (control_mode_get() == CONTROL_MODE_ARM);
        const float manual_base    = arm_mode ? base_yaw : 0.0f;
        const float manual_shoulder = shoulder;
        const float manual_elbow   = arm_mode ? elbow : 0.0f;
        const float manual_gripper = arm_mode ? gripper : 0.0f;

        /* 打断判定与点动用同一份归一化量（阈值在 arm_manual.h） */
        arm_button_feed_sticks(manual_base, manual_shoulder,
                               manual_elbow, manual_gripper);

        /* ---------------- 按键决策：打断 / 启动动作 / 夹爪 / 卸力 / 菜单 ----------------
         * 内含三件事，都必须排在点动之前：
         *   1) E5 的摇杆打断判定；
         *   2) 十字键末端点动（arm_endpoint，会直接写三关节）；
         *   3) 按键事件分派（含 R1 菜单）。
         * 本周期被取消的动作不会再有下一拍推进，臂就停在当前位置。 */
        arm_button_apply(now_ms);

        /* ---------------- 推进机械臂点动 ----------------
         * 三关节同一周期只能有一个主人，否则两次写入谁赢取决于调用顺序、
         * 关节会抖。以下三种情况摇杆点动让位（喂全 0，本模块只刷新时间戳）：
         *   - 动作运行中：动作独占；
         *   - 十字键末端点动激活：末端点动独占；
         *   - 动作选择菜单里：菜单态下"原本的平移/夹爪"按设计暂不响应。 */
        const bool action_running =
            (arm_action_get_state() == ActionState::kRunning);
        const bool manual_off =
            action_running || arm_button_is_endpoint_jog_active() ||
            arm_button_in_menu();

        arm_manual_update(now_ms,
                          manual_off ? 0.0f : manual_base,
                          manual_off ? 0.0f : manual_shoulder,
                          manual_off ? 0.0f : manual_elbow,
                          manual_off ? 0.0f : manual_gripper);

        /* ---------------- 推进动作引擎（关键帧序列） ---------------- */
        arm_action_update(now_ms);

        osDelay(ARM_TASK_PERIOD_MS);
    }
}

extern "C" {
    ArmTask arm_task;

    void arm_task_init() {
        // 栈 512 字：动作引擎 + 按键决策都是栈上小对象，余量沿用底盘任务同级
        arm_task.start("ArmTask", 512, osPriorityNormal);
    }
}
