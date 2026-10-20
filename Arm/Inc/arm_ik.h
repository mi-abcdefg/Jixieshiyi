//
// 机械臂逆运动学：柱坐标末端位置 → 三个关节角
//
// 【当前状态：闲置】机械臂已切换为挖掘机式关节直控（见 arm_manual），
// 本模块没有任何调用方，链接器会将其裁出固件。保留原因：未来若需要
// 末端坐标控制（如视觉引导）可直接复用。
//
// 纯数学，不依赖 HAL / 舵机 / 任何任务，输入输出都是值。
//
// 坐标定义（臂长参数见下方宏，单位米）：
//   - 参考系固定在底座旋转轴上、肩关节转轴的高度处；
//   - theta：末端绕 Z 轴的方位角，直接对应底座关节角（度）；
//   - r    ：末端到旋转轴的水平距离（径向伸出量）；
//   - z    ：末端相对肩关节转轴的高度，向上为正。
// 连杆平面内是标准二连杆（link1 + link2），肘部姿态默认取"肘上"解。
//
// 角度约定：link1_deg 为第一连杆相对水平面的绝对角（抬头为正），
// link2_deg 为第二连杆相对第一连杆的相对角。0° 对应什么物理姿态、
// 是否与舵机方向一致，属于实机标定范围：必要时在 servo_driver 的
// reversed / 角度范围里修正，或在本文件加固定偏移，不改动上层。
//

#ifndef CRTC2026_ROBOTA_ARM_IK_H
#define CRTC2026_ROBOTA_ARM_IK_H

#pragma once
#include <stdint.h>
#include <stdbool.h>

/* ------------------------- 臂长参数（实测值） -------------------------
 * 从下往上：肩 SG90（PB7）到肘 SG90（PB8）的连杆 11cm；
 * 肘 SG90 到腕部（夹爪舵机中心）的连杆 8.5cm。
 */
#define ARM_IK_LINK1_M      0.11f   // 第一段连杆长度 [m]
#define ARM_IK_LINK2_M      0.085f  // 第二段连杆长度 [m]

/* 底座角度相对方位角的固定偏移（舵机 0° 与径向不对齐时用），占位 0 */
#define ARM_IK_BASE_Z_OFFSET_DEG  0.0f

/* 可达半径的边界内缩量：留出行程余量，避免贴着奇异位形解算 */
#define ARM_IK_REACH_MARGIN_M    0.004f

/* 肘部解选择：1 = 肘上（elbow-up），0 = 肘下。实物装配后确定 */
#define ARM_IK_ELBOW_UP    1

/**
 * @brief 三个关节角（不含夹爪；夹爪与末端位置无关，由上层自行保持）
 */
typedef struct
{
    float base_z_deg;   // 底座（方位角）
    float link1_deg;    // 第一连杆绝对角
    float link2_deg;    // 第二连杆相对角
} ArmJointAngles_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 逆解：柱坐标末端位置 → 关节角
 *
 * r / z 超出可达范围时不报错，而是夹取到最近的可达位置（点动手感要求
 * 连续，宁可贴边也不跳变）；只有输入非有限值（NaN/Inf）才返回 false。
 *
 * @return false 仅当输入非有限值
 */
bool arm_ik_solve(float theta_deg, float r_m, float z_m,
                  ArmJointAngles_t *out);

/**
 * @brief 正解：关节角 → 柱坐标末端位置（用于点动模块从当前姿态取起点）
 */
void arm_ik_forward(float base_z_deg, float link1_deg, float link2_deg,
                    float *theta_deg, float *r_m, float *z_m);

/**
 * @brief 可达半径边界（arm_manual 积分限幅用，与 solve 内部夹取一致）
 */
void arm_ik_get_reach(float *r_min_m, float *r_max_m);

#ifdef __cplusplus
}
#endif

#endif //CRTC2026_ROBOTA_ARM_IK_H
