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
// 注意：LENGTH / WIDTH 指"轴距 / 轮距"（前后轮中心距、左右轮中心距），不是车体外壳尺寸。
// 实测轮距与轴距均为 12 cm。
#define MECANUM_CHASSIS_LENGTH_M          0.12f
#define MECANUM_CHASSIS_WIDTH_M           0.12f
#define MECANUM_OMEGA_SCALE               0.5f   // 标准麦克纳姆模型系数 (L + W) / 2

// 轮子线速度满量程 [m/s]：既是摇杆满量程对应的车体速度，
// 也是"轮子线速度 -> PWM 指令"的 100% 占空比参考点（见 chasis_control_task.cpp）。
// 该值在解算与换算中会约掉，只影响物理标定（后续做里程计/闭环时改成实测极速即可）。
#define MECANUM_MAX_SPEED                 2.0f

// 旋转角速度满量程 [rad/s]（右摇杆左右方向的满量程）。
//
// 【为什么不能写死成 2.0】原地旋转时四个轮子的线速度是 ±k·ω，
// 其中 k = MECANUM_OMEGA_SCALE × (L + W) = 0.5 × 0.24 = 0.12 m/rad。
// 原来写死 2.0 rad/s 时：k·ω_max = 0.12 × 2.0 = 0.24 m/s，
// 只有满量程轮速 2.0 m/s 的 12% —— 换算成占空比只有 12%，
// 表现就是"右摇杆左右推到底，车几乎不转"。
//
// 这里按物理关系直接算：ω_max = MAX_SPEED / k ≈ 16.7 rad/s，
// 使右摇杆推到底时车轮刚好达到满速，旋转权限与平移相当。
// 如果觉得原地旋转太快、不好控，把它改小即可（例如 8.0f 就只有一半权限）。
#define MECANUM_MAX_OMEGA_RADPS           (MECANUM_MAX_SPEED / \
                                           (MECANUM_OMEGA_SCALE * (MECANUM_CHASSIS_LENGTH_M + MECANUM_CHASSIS_WIDTH_M)))

// 轮子编号约定：FL=前左、FR=前右、BL=后左、BR=后右
#define MECANUM_WHEEL_FL                  0U
#define MECANUM_WHEEL_FR                  1U
#define MECANUM_WHEEL_BL                  2U
#define MECANUM_WHEEL_BR                  3U

// 车体输入：vx 为前进速度，vy 为横向速度，omega 为角速度
// 输出：wheel_speeds[4] 按 [FL, FR, BL, BR] 顺序返回四个轮子的线速度
void mecanum_get_wheel_speeds(float vx,
                              float vy,
                              float omega,
                              float wheel_speeds[4]);

#ifdef __cplusplus
}
#endif

#endif // CRTC2026_ROBOTA_MECANUM_KINEMATIC_H