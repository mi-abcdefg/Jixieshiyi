//
// 麦克纳姆轮运动学解算接口
// Created by MI on 2026/9/13.
//

#ifndef CRTC2026_ROBOTA_MECANUM_KINEMATIC_H
#define CRTC2026_ROBOTA_MECANUM_KINEMATIC_H

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 车体运动学参数
#define MECANUM_MAX_SPEED                 2.0f
#define MECANUM_CHASSIS_LENGTH_M          0.45f
#define MECANUM_CHASSIS_WIDTH_M           0.35f
#define MECANUM_OMEGA_SCALE               0.5f

// 轮子编号约定：FL=前左、FR=前右、RL=后左、RR=后右
#define MECANUM_WHEEL_FL                  0U
#define MECANUM_WHEEL_FR                  1U
#define MECANUM_WHEEL_RL                  2U
#define MECANUM_WHEEL_RR                  3U

// 车体输入：vx 为前进速度，vy 为横向速度，omega 为角速度
// 输出：wheel_speeds[4] 按 [FL, FR, RL, RR] 顺序返回四个轮子的线速度（m/s）
void mecanum_get_wheel_speeds(float vx,
                              float vy,
                              float omega,
                              float wheel_speeds[4]);

#ifdef __cplusplus
}
#endif

#endif // CRTC2026_ROBOTA_MECANUM_KINEMATIC_H