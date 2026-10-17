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
    // 轮子编号顺序按照 [FL, FR, BL, BR] 组织。
    const float k = MECANUM_OMEGA_SCALE * (MECANUM_CHASSIS_LENGTH_M + MECANUM_CHASSIS_WIDTH_M);

    wheel_speeds[MECANUM_WHEEL_FL] = vx + vy + k * omega;
    wheel_speeds[MECANUM_WHEEL_FR] = vx - vy - k * omega;
    wheel_speeds[MECANUM_WHEEL_BL] = vx - vy + k * omega;
    wheel_speeds[MECANUM_WHEEL_BR] = vx + vy - k * omega;


    // 速度归一化：必须按"绝对值最大"的轮速做等比缩放。
    // 原来只取带符号的最大值，倒车/反向旋转时最大轮速是负数，
    // 饱和判断会失效，导致某个轮子单独被限幅、运动方向发生偏斜。
    float max_speed = 0.0f;
    for (uint8_t i = 0U; i < 4U; ++i)
    {
        const float abs_speed = std::fabs(wheel_speeds[i]);
        if (abs_speed > max_speed)
        {
            max_speed = abs_speed;
        }
    }

    if (max_speed > MECANUM_MAX_SPEED)
    {
        const float scale = MECANUM_MAX_SPEED / max_speed;
        for (uint8_t i = 0U; i < 4U; ++i)
        {
            wheel_speeds[i] *= scale;
        }
    }
}
