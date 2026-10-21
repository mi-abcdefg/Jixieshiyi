//
// 机械臂平面二连杆运动学（纯数学，无硬件依赖）
//
// 机械臂解算只涉及肩（SERVO_LINK_1，11cm 连杆）与肘（SERVO_LINK_2，8.5cm
// 连杆）两支舵机：连杆平面内是标准二连杆，底座不参与解算（十字键末端
// 点动保持底座不动），夹爪与末端位置无关。
//
// 坐标（参考点 = 肩关节转轴）：r 为水平伸出量 [m]，z 为高度 [m]，向上为正。
// 角度约定（几何角，度）：link1 = 第一连杆相对水平面的绝对角（抬头为正），
// link2 = 第二连杆相对第一连杆的带符号相对角。
//
// **几何角 ↔ 舵机命令角的换算属于实机标定，由调用方负责**
// （arm_endpoint 用 ARM_ENDPOINT_SHOULDER_OFFSET_DEG，肘两者同义）。
//
// 工作区夹取的单一所有者是调用方（arm_ik_get_reach 提供边界）；
// 本模块对越界输入是宽容的：贴边时解自动退化为全伸/全折，不会 NaN。
//

#ifndef CRTC2026_ROBOTA_ARM_IK_H
#define CRTC2026_ROBOTA_ARM_IK_H

#pragma once
#include <stdint.h>
#include <stdbool.h>

/* ------------------------- 臂长参数（实测值） ------------------------- */
#define ARM_IK_LINK1_M      0.11f   // 第一段连杆长度 [m]（肩→肘）
#define ARM_IK_LINK2_M      0.09f  // 第二段连杆长度 [m]（肘→腕）

/* 可达半径的边界内缩量：留出行程余量，避免贴着奇异位形解算 */
#define ARM_IK_REACH_MARGIN_M    0.004f

/**
 * @brief 逆解的两支结果（几何角，度）
 * @note  肘上支 link2 < 0、肘下支 link2 > 0；臂实际停在哪一支由调用方
 *        按"离当前关节角更近"选取，本模块不做选择
 */
typedef struct
{
    float link1_up_deg;    // 肘上支：第一连杆绝对角
    float link2_up_deg;    // 肘上支：第二连杆相对角（负）
    float link1_down_deg;  // 肘下支：第一连杆绝对角
    float link2_down_deg;  // 肘下支：第二连杆相对角（正）
} ArmIkBranches;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 逆解：平面末端位置 → 两支关节角（肘上/肘下各一组）
 * @return false 仅当 (r,z) 退化（ρ≈0，即目标落在肩关节转轴上）
 */
bool arm_ik_solve(float r_m, float z_m, ArmIkBranches *out);

/**
 * @brief 正解：关节角 → 平面末端位置（点动模块从当前姿态取起点）
 */
void arm_ik_forward(float link1_deg, float link2_deg,
                    float *r_m, float *z_m);

/**
 * @brief 可达半径边界（调用方夹取用，本模块不做二次夹取）
 * @note  r_min = |L1−L2| + 少量内缩：完全折起附近逆解病态；
 *        r_max = L1+L2 − 内缩：完全伸直是奇异位形
 */
void arm_ik_get_reach(float *r_min_m, float *r_max_m);

#ifdef __cplusplus
}
#endif

#endif //CRTC2026_ROBOTA_ARM_IK_H
