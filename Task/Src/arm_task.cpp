//
// Created by MI on 2026/10/17.
//

// 机械臂任务实现：编排 gamepad / robot_control / arm_manual / arm_action。
//
// 摇杆分配（挖掘机式关节直控，R2 切换控制对象，上电默认底盘模式）：
//   ┌──────────────┬────────────────────────┬──────────────────────────┐
//   │ 输入         │ 底盘模式                │ 机械臂模式                │
//   ├──────────────┼────────────────────────┼──────────────────────────┤
//   │ 右摇杆 左右   │ 底盘旋转                │ 底座旋转（方位角）         │
//   │ 右摇杆 上下   │ 肩抬落（两模式都有效）   │ 肩抬落                    │
//   │ 左摇杆 上下   │ 底盘前后                │ 肘抬落                    │
//   │ 左摇杆 左右   │ 底盘平移                │ 夹爪开合                  │
//   └──────────────┴────────────────────────┴──────────────────────────┘
//   按键（机械臂模式）：○=夹爪两档切换  L2=卸力⇄恢复（恢复后自动伸展
//   脱离卡住区，途中 L2/推杆/SELECT 可停）  L1=点动两档调速
//   △/□=抓取动作  ×=放置  SELECT=取消；
//   动作键仅机械臂模式生效，SELECT 取消在两种模式下都可用。
//
// 数据流（每周期约 20ms）：
//   PS2 解码结果（RemoteCommTask 每 10ms 刷新，这里只读）
//     ├─ 按键位重排 → gamepad_update → robot_control_update（按键决策，模式门控在其内部）
//     └─ 摇杆归一化 → 按模式分流 → arm_manual_update（关节点动）
//   arm_escape_update（卸力恢复后的自动伸展脱离）
//   arm_action_update（自动动作状态机推进）

#include "arm_task.h"

#include "ps2_decode.h"
#include "control_mode.h"
#include "gamepad.h"
#include "robot_control.h"
#include "arm_manual.h"
#include "arm_action.h"
#include "servo_driver.h"

/* ------------------------------ 周期与阈值 ------------------------------ */
#define ARM_TASK_PERIOD_MS        20U    // 与动作状态机 20ms 指令节拍一致

/* PS2 模式字节：只有这两个值代表"已连上"（见 TestTask 顶部的说明） */
#define PS2_MODE_ANALOG           0x73u
#define PS2_MODE_DIGITAL          0x41u

/* 摇杆中位与归一化满量程（解码层死区 118~138 已折回 128） */
#define STICK_CENTER              128.0f
#define STICK_RANGE               128.0f

namespace {
    /* ---------------- PS2 原始位 → gamepad 抽象位图 ----------------
     * PS2 位定义见 ps2_decode.h；GAMEPAD_* 是软件自定义的统一格式。
     * 换遥控器时只改这里。 */
    uint16_t map_ps2_to_gamepad(uint8_t btn1, uint8_t btn2) {
        uint16_t pressed = 0;
        if (btn1 & PS2_BTN_SELECT)  pressed |= GAMEPAD_SELECT;
        if (btn1 & PS2_BTN_LEFT)    pressed |= GAMEPAD_PAD_LEFT;
        if (btn2 & PS2_BTN_L1)       pressed |= GAMEPAD_L1;
        if (btn2 & PS2_BTN_L2)       pressed |= GAMEPAD_L2;
        if (btn2 & PS2_BTN_TRIANGLE) pressed |= GAMEPAD_TRIANGLE;
        if (btn2 & PS2_BTN_CIRCLE)   pressed |= GAMEPAD_CIRCLE;
        if (btn2 & PS2_BTN_CROSS)    pressed |= GAMEPAD_CROSS;
        if (btn2 & PS2_BTN_SQUARE)   pressed |= GAMEPAD_SQUARE;
        return pressed;
    }

    /* 摇杆原始值 → 归一化 -1..1。
     * 方向按直觉定义：上推 = 正。原始值上推变小，所以取负号。
     * 实测方向不顺手时，改这里的符号即可，不要动下层模块。 */
    float stick_to_norm(uint8_t raw, bool inverted) {
        float v = (static_cast<float>(raw) - STICK_CENTER) / STICK_RANGE;
        return inverted ? -v : v;
    }

    bool beyond_deadzone(float v) {
        return v > ARM_MANUAL_DEADZONE || v < -ARM_MANUAL_DEADZONE;
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

    arm_action_init();   // 读回初始姿态作为动作轨迹起点
    arm_manual_init();
    gamepad_init();

    while (1) {
        const uint32_t now_ms = osKernelSysTick();

        /* ---------------- 手柄连接状态 ----------------
         * 模式字节是 0x73 / 0x41 才代表链路正常（与 TestTask 的判据一致） */
        const uint8_t ps2_mode = ps2_get_mode();
        const bool connected =
            (ps2_mode == PS2_MODE_ANALOG) || (ps2_mode == PS2_MODE_DIGITAL);

        /* ---------------- 按键：喂给事件层 ----------------
         * 每周期都要喂（断连时也要喂全 0），事件层才能正确维护边沿状态。 */
        gamepad_update(map_ps2_to_gamepad(ps2_get_btn1(), ps2_get_btn2()),
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

        /* ---------------- 仲裁：手动点动打断自动运动 ----------------
         * 与 robot_control 的"动作中不响应新动作键"互补：
         * 人手动推杆是最直接的意图表达，优先级最高——正在执行的
         * 动作序列和自动伸展序列都会被停下，保持在当前位置。 */
        const bool manual_input =
            beyond_deadzone(manual_base) || beyond_deadzone(manual_shoulder) ||
            beyond_deadzone(manual_elbow) || beyond_deadzone(manual_gripper);
        if (manual_input) {
            if (arm_action_get_state() == ARM_ACTION_STATE_RUNNING) {
                arm_action_cancel();
            }
            if (arm_escape_is_running()) {
                arm_escape_stop();
            }
        }

        /* ---------------- 推进机械臂各层 ---------------- */
        arm_manual_update(now_ms, manual_base, manual_shoulder,
                          manual_elbow, manual_gripper);
        robot_control_update(now_ms);   // 事件在内部每周期取走，不残留
        arm_escape_update(now_ms);      // 卸力恢复后的自动伸展脱离
        arm_action_update(now_ms);

        osDelay(ARM_TASK_PERIOD_MS);
    }
}

extern "C" {
    ArmTask arm_task;

    void arm_task_init() {
        // 栈 512 字：关节点动逻辑轻量，但保留与 ChasisControlTask 同级的余量
        arm_task.start("ArmTask", 512, osPriorityNormal);
    }
}
