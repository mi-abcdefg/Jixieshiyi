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

// ---------------- 阻力扭矩补偿（前馈，加在解算输出的轮速上） ----------------
// 传动系统存在静摩擦：占空比低于阈值时车轮完全不动，表现为小油门无响应、
// 起步迟钝。解算完成后按各轮速度方向叠加一个"恰好克服静摩擦"的等效轮速，
// 把死区线性化——推杆即动、过零即时反向，不再有起动力矩死区。
//
// 标定方法：车放在地上，逐步加大油门到四轮刚好开始转动，记下该时刻的
// 占空比 duty(%)，换算：COMP_MPS = duty / 100 × MECANUM_MAX_SPEED。
// 例：6% 占空比起步 → 0.12f。取值偏小无害（只是补偿不足），偏大则小油门起步偏猛。
// 注意：补偿以 m/s 参与解算输出，后续慢速模式的换算会等比例作用到补偿上
// （慢速模式下补偿随之减弱，若把慢速比例调得很小需复核补偿是否仍然够用）。
#define MECANUM_FRICTION_COMP_MPS         0.12f

// 速度比例补偿项（可选，默认关闭）：随轮速线性增加的附加前馈，单位同上
// （满速时的附加量），用于补偿随转速增大的粘性摩擦。开环下难以标定，保持 0 即可。
#define MECANUM_FRICTION_KV_MPS           0.0f

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