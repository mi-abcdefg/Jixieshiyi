//
// 麦克纳姆轮运动学解算实现
// Created by MI on 2026/9/13.
//

#include "mecanum_kinematic.h"

#include <cmath>

void mecanum_get_wheel_speeds(float vx_mps,
                              float vy_mps,
                              float omega_rad_s,
                              float wheel_speeds_mps[4])
{
    if (wheel_speeds_mps == nullptr)
    {
        return;
    }


    // 麦克纳姆轮的运动学关系：
    // 以车体坐标系 x 向前，y 向左，z 向上为正，角速度 omega 为绕 z 轴旋转。
    // 轮子编号顺序按照 [FL, FR, RL, RR] 组织。
    const float half_lever_arm = 0.5f * (MECANUM_CHASSIS_LENGTH_M + MECANUM_CHASSIS_WIDTH_M);

    wheel_speeds_mps[MECANUM_WHEEL_FL] = vx_mps + vy_mps + half_lever_arm * omega_rad_s;
    wheel_speeds_mps[MECANUM_WHEEL_FR] = vx_mps - vy_mps - half_lever_arm * omega_rad_s;
    wheel_speeds_mps[MECANUM_WHEEL_RL] = vx_mps - vy_mps + half_lever_arm * omega_rad_s;
    wheel_speeds_mps[MECANUM_WHEEL_RR] = vx_mps + vy_mps - half_lever_arm * omega_rad_s;


    // 速度归一化
    float max_speed = std::max(std::max(std::max(wheel_speeds_mps[MECANUM_WHEEL_FL], wheel_speeds_mps[MECANUM_WHEEL_FR]),
    wheel_speeds_mps[MECANUM_WHEEL_RL]), wheel_speeds_mps[MECANUM_WHEEL_RR]);
    if (max_speed > MECANUM_MAX_SPEED_MPS)
    {

        for (uint8_t i = 0U; i < 4U; ++i)
        {
            wheel_speeds_mps[i] *= MECANUM_MAX_SPEED_MPS / max_speed;
        }
    }
}
