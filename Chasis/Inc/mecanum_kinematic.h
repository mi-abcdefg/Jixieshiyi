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
/* 标准麦克纳姆模型系数 (L + W) / 2 = 0.5×(轴距+轮距) = 0.12 m/rad。
 * 物理意义：绕 Z 轴转 1 rad/s 时，每个轮子要多跑 k 米/秒。 */
#define MECANUM_OMEGA_SCALE               0.5f

// 轮子线速度满量程 [m/s]：既是摇杆满量程对应的车体速度，
// 也是"轮子线速度 -> PWM 指令"的 100% 占空比参考点（见 chasis_control_task.cpp）。
// 2026/10/21 实车反馈车速太快，满量程减半（原 2.0）：满杆直线/原地旋转/
// 固定慢速档全部随之减半，占空比换算不变（满杆仍 = 100% 占空比）。
// 后续做里程计/闭环时按实测极速改此值即可。
#define MECANUM_MAX_SPEED                 1.0f

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

// ---------------- 阻力扭矩补偿（前馈，作用在解算输出的轮速上） ----------------
// 传动系统有静摩擦：占空比低于"起步占空比"时车轮完全不动，表现为小油门无响应。
// 补偿方式：若四轮里最快的那一轮仍低于 COMP，就把**整组轮速等比放大**到 COMP。
//   · 等比放大保持四轮比值（比值就是运动方向），不会把"斜着慢走"夹成"正着走"；
//   · 不做叠加：叠加会让整条曲线整体上移（满速前超调、固定慢速档被顶到 100%）；
//   · 已经更快的一律按原值走，各档位梯度与顶部速度都不受影响。
//
// 标定方法：车放在地上，逐步加大油门到四轮刚好开始转动，记下该时刻的
// 占空比 duty(%)，换算：COMP_MPS = duty / 100 × MECANUM_MAX_SPEED。
// 2026/10/21 起前馈**暂归零**（实车车速问题排查期，用户按实测再调）——
// 历史标定记录：60% 占空比起步（含旧前馈 56% 时仍起不来）。重新标定时
// 按上面公式把测得的占空比填回来即可。
#define MECANUM_FRICTION_COMP_MPS         (0.0f)

// ---------------- 四轮前馈微调（逐轮独立参数，按车分套） ----------------
// 四个轮子的前馈修正量是**独立参数**：实车上给高了调低、给低了调高，
// 逐轮各自收敛，数值之间没有任何约束关系。
// 约定：这组微调的**中位数 ≈ 0**，全局起步补偿（上方 COMP）仍是标定基准——
// 若四轮整体都提前/滞后，先改 COMP，再回来逐轮微调。
//
// 参数按车分套（A/B 车底盘阻力特性不同），切换方式同 MOTOR_INVERT_xx：
// 改 #if 的 0/1，两套配置都以注释/代码保留在文件里。
#if 1    /* ===== A 车配置（2026/10/21 全部回正常值） ===== */
#define MECANUM_COMP_TRIM_FL   (0.0f)     // 2026/10/21 实测后回退：与其他轮一致
#define MECANUM_COMP_TRIM_FR   (0.0f)
#define MECANUM_COMP_TRIM_BL   (0.0f)
#define MECANUM_COMP_TRIM_BR   (0.0f)
#else    /* ===== B 车配置（占位：B 车实测后逐轮填） ===== */
#define MECANUM_COMP_TRIM_FL   (0.0f)
#define MECANUM_COMP_TRIM_FR   (0.0f)
#define MECANUM_COMP_TRIM_BL   (0.0f)
#define MECANUM_COMP_TRIM_BR   (0.0f)
#endif

// 速度比例补偿项（可选，默认关闭）：随轮速线性增加的下限（满速时的附加量），
// 用于补偿随转速增大的粘性摩擦。开环下难以标定，保持 0 即可。
#define MECANUM_FRICTION_KV_MPS           0.0f

// 轮子编号约定：FL=前左、FR=前右、BL=后左、BR=后右
#define MECANUM_WHEEL_FL                  0U
#define MECANUM_WHEEL_FR                  1U
#define MECANUM_WHEEL_BL                  2U
#define MECANUM_WHEEL_BR                  3U

/* ============================ 坐标与符号约定 ============================
 * 解算用的是**左手系**（与教科书的标准式差一次 y 轴镜像）：
 *   x 向前为正；**y 向右为正**；z 向上；omega 正向 = **俯视顺时针**。
 * 也就是说 vy>0 = 向右平移、omega>0 = 顺时针转（右转），与操作者推杆的方向一致
 * （左摇杆右推 → vy>0 → 车向右平移；右摇杆右推 → omega>0 → 车右转）。
 *
 * 对应方程（k = 0.12 m/rad）：
 *   FL = vx + vy + k·ω      FR = vx − vy − k·ω
 *   BL = vx − vy + k·ω      BR = vx + vy − k·ω
 * 这三组符号是自洽的：把上式按逆矩阵解回去能原样得到 (vx, vy, ω)，
 * 说明它就是一辆正常麦轮车的模型，只是 y/ω 的符号取了镜像那一套。
 *
 * 【改这里之前注意】若将来接里程计/导航，标准文献的公式是
 * "y 向左、omega 逆时针"，与本模块差 (vy, ω) 同时取反；照抄公式前先把符号对齐，
 * 否则会出现"看着对、跑起来镜像"的问题。真正的物理方向（左右平移、顺逆时针）
 * 还受电机接线影响，由 motor_pwm_driver.h 的 MOTOR_INVERT_xx 修正。
 * ====================================================================== */

/**
 * @brief 车体速度 → 四轮线速度（含归一化、档位缩放、起步下限、限幅）
 *
 * 处理顺序（每一步都不能随便调换）：
 *   混算 → 归一化到满量程内 → × throttle_scale → 起步下限（整组等比放大）→ 兜底限幅
 *
 * @param vx             前进速度 [m/s]（正 = 前进）
 * @param vy             横向速度 [m/s]（正 = 向右）
 * @param omega          角速度 [rad/s]（正 = 俯视顺时针）
 * @param throttle_scale 档位缩放（1.0 = 快速档，0.8 = 中速档）。必须放在
 *                       "起步下限"之前，否则中速档会把起步下限一起缩小。
 *                       固定慢速档请直接把 vx/vy/omega 写成绝对值、这里传 1.0。
 * @param wheel_speeds   [out] 按 [FL, FR, BL, BR] 顺序返回四个轮子的线速度 [m/s]
 */
void mecanum_get_wheel_speeds(float vx,
                              float vy,
                              float omega,
                              float throttle_scale,
                              float wheel_speeds[4]);

#ifdef __cplusplus
}
#endif

#endif // CRTC2026_ROBOTA_MECANUM_KINEMATIC_H