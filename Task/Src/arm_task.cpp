//
// Created by MI on 2026/10/17.
//

// 机械臂任务实现：摇杆关节点动编排。
// 本分支（feature/arm-jog-only）已移除动作序列/按键决策/事件层，只保留
// 挖掘机式摇杆操作；动作模块待重建，交接数据见 docs/机械臂动作模块重建交接.md，
// 旧实现可用 git show main:Arm/Src/arm_action.c 取回参考。
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
//
// 数据流（每周期约 20ms）：
//   PS2 解码结果（RemoteCommTask 每 10ms 刷新，这里只读）
//     → 摇杆归一化 → 按模式分流 → arm_manual_update（关节/夹爪点动）

#include "arm_task.h"

#include "ps2_decode.h"
#include "control_mode.h"
#include "arm_manual.h"
#include "servo_driver.h"

/* ------------------------------ 周期 ------------------------------ */
#define ARM_TASK_PERIOD_MS        20U    // 点动节拍

/* 摇杆中位与归一化满量程（解码层死区 118~138 已折回 128） */
#define STICK_CENTER              128.0f
#define STICK_RANGE               128.0f

namespace {
    /* 摇杆原始值 → 归一化 -1..1。
     * 方向按直觉定义：上推 = 正。原始值上推变小，所以取负号。
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

    arm_manual_init();

    while (1) {
        const uint32_t now_ms = osKernelSysTick();

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

        /* ---------------- 推进机械臂点动 ---------------- */
        arm_manual_update(now_ms, manual_base, manual_shoulder,
                          manual_elbow, manual_gripper);

        osDelay(ARM_TASK_PERIOD_MS);
    }
}

extern "C" {
    ArmTask arm_task;

    void arm_task_init() {
        // 栈 512 字：与 ChasisControlTask 同级的余量
        arm_task.start("ArmTask", 512, osPriorityNormal);
    }
}
