//
// Created by MI on 2026/9/13.
//

#include "chasis_control_task.h"

void ChasisControlTask::run() {
    // 按键边沿检测用（只在"按下那一刻"翻转一次，否则按住会每 10ms 翻一次）
    bool l2_last_pressed = false;   // L2：电机驱动通断
    bool l1_last_pressed = false;   // L1：快速/慢速模式翻转
    bool slow_mode = false;         // 上电默认【快速模式】

    while (1) {
        const uint8_t btn2 = ps2_get_btn2();

        // ---------------- L2：电机驱动通断（上电默认关断） ----------------
        const bool l2_pressed = (btn2 & PS2_BTN_L2) != 0;
        if (l2_pressed && !l2_last_pressed) {
            motor_toggle_enable();
        }
        l2_last_pressed = l2_pressed;

        // ---------------- L1：快速/慢速模式翻转 ----------------
        const bool l1_pressed = (btn2 & PS2_BTN_L1) != 0;
        if (l1_pressed && !l1_last_pressed) {
            slow_mode = !slow_mode;
        }
        l1_last_pressed = l1_pressed;

        // 获取左摇杆和右摇杆数据
        uint8_t left_analog[2];
        uint8_t right_analog[2];
        ps2_get_left_analog_data(left_analog);
        ps2_get_right_analog_data(right_analog);

        // 将摇杆数据转换为车体速度指令：vx/vy 单位 m/s，omega 单位 rad/s
        //   左摇杆上下 -> vx 前进/后退
        //   左摇杆左右 -> vy 原地平移
        //   右摇杆左右 -> omega 原地旋转（右摇杆上下未使用）
        float vx = (left_analog[1] - 128) / 128.0f * MECANUM_MAX_SPEED;          // 前进/后退速度
        float vy = (left_analog[0] - 128) / 128.0f * MECANUM_MAX_SPEED;          // 左/右速度
        float omega = (right_analog[0] - 128) / 128.0f * MECANUM_MAX_OMEGA_RADPS; // 旋转角速度

        // 运动学解算：得到四个轮子的线速度 [m/s]
        float wheel_speeds[4];
        mecanum_get_wheel_speeds(vx, vy, omega, wheel_speeds);

        // 量纲换算：轮子线速度 [m/s] -> 速度指令 [0 ~ MOTOR_CMD_MAX]。
        // MECANUM_MAX_SPEED 是满量程轮速，对应 100% 占空比；
        // 慢速模式再整体乘一个系数（平移和旋转一起降速）。
        const float speed_factor = slow_mode ? CHASIS_SLOW_MODE_SCALE : 1.0f;
        const float cmd_scale = static_cast<float>(MOTOR_CMD_MAX) / MECANUM_MAX_SPEED * speed_factor;

        // 设置轮子速度。驱动被 L2 关断时，Motor::set_wheel_speed() 内部会自动按 0 输出，
        // 所以这里正常下发即可，不需要再判断使能状态。
        motor_set_wheel_speed(static_cast<int>(wheel_speeds[MECANUM_WHEEL_FL] * cmd_scale),
                              static_cast<int>(wheel_speeds[MECANUM_WHEEL_FR] * cmd_scale),
                              static_cast<int>(wheel_speeds[MECANUM_WHEEL_BL] * cmd_scale),
                              static_cast<int>(wheel_speeds[MECANUM_WHEEL_BR] * cmd_scale));

        osDelay(10); // 延时，避免任务占用过多CPU时间
    }
}

extern "C" {
    ChasisControlTask chasis_control_task;

    void chasis_control_task_init() {
        motor_init();   // 启动 PWM；DRV8833 保持关断，由 L2 控制使能
        chasis_control_task.start("ChasisControlTask", 512, osPriorityNormal);
    }
}
