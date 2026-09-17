//
// Created by MI on 2026/9/13.
//

#include "chasis_control_task.h"

void ChasisControlTask::run() {
    while (1) {

        // 获取左摇杆和右摇杆数据
        uint8_t left_analog[2];
        uint8_t right_analog[2];
        ps2_get_left_analog_data(left_analog);
        ps2_get_right_analog_data(right_analog);

        // 将摇杆数据转换为速度指令
        float vx = (left_analog[1] - 128) / 128.0f * MECANUM_MAX_SPEED; // 前进/后退速度
        float vy = (left_analog[0] - 128) / 128.0f * MECANUM_MAX_SPEED; // 左/右速度
        float omega = (right_analog[0] - 128) / 128.0f * MECANUM_MAX_SPEED; // 旋转速度

        // 计算轮子速度
        float wheel_speeds[4];
        mecanum_get_wheel_speeds(vx, vy, omega, wheel_speeds);

        // 设置轮子速度
        motor_set_wheel_speed(static_cast<int>(wheel_speeds[MECANUM_WHEEL_FL]),
                              static_cast<int>(wheel_speeds[MECANUM_WHEEL_FR]),
                              static_cast<int>(wheel_speeds[MECANUM_WHEEL_BL]),
                              static_cast<int>(wheel_speeds[MECANUM_WHEEL_BR]));

        osDelay(10); // 延时，避免任务占用过多CPU时间
    }
}

extern "C" {
    ChasisControlTask chasis_control_task;

    void chasis_control_task_init() {
        motor_init();
        chasis_control_task.start("ChasisControlTask", 512, osPriorityNormal);
    }
}