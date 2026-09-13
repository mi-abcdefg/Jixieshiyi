//
// 麦克纳姆轮运动学解算实现
// Created by MI on 2026/9/13.
//

#include "mecanum_kinematic.h"

#include <cmath>

void mecanum_get_wheel_speeds(float vx,
                              float vy,
                              float omega,
                              float wheel_speeds[4])
{
    if (wheel_speeds == nullptr)
    {
        return;
    }


    // 麦克纳姆轮的运动学关系：
    // 以车体坐标系 x 向前，y 向左，z 向上为正，角速度 omega 为绕 z 轴旋转。
    // 轮子编号顺序按照 [FL, FR, RL, RR] 组织。
    const float k = MECANUM_OMEGA_SCALE * (MECANUM_CHASSIS_LENGTH_M + MECANUM_CHASSIS_WIDTH_M);

    wheel_speeds[MECANUM_WHEEL_FL] = vx + vy + k * omega;
    wheel_speeds[MECANUM_WHEEL_FR] = vx - vy - k * omega;
    wheel_speeds[MECANUM_WHEEL_RL] = vx - vy + k * omega;
    wheel_speeds[MECANUM_WHEEL_RR] = vx + vy - k * omega;


    // 速度归一化
    float max_speed = std::max(std::max(std::max(wheel_speeds[MECANUM_WHEEL_FL], wheel_speeds[MECANUM_WHEEL_FR]),
    wheel_speeds[MECANUM_WHEEL_RL]), wheel_speeds[MECANUM_WHEEL_RR]);
    if (max_speed > MECANUM_MAX_SPEED)
    {

        for (uint8_t i = 0U; i < 4U; ++i)
        {
            wheel_speeds[i] *= MECANUM_MAX_SPEED / max_speed;
        }
    }
}
