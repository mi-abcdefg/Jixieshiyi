//
// 机械臂手动点动：归一化摇杆速度 → 柱坐标目标积分 → 逆解 → 关节角下发
//
// 全新模块。与自动动作（arm_action）的关系由编排层（arm_task）仲裁：
// 手动输入非零时先取消动作再点动，本模块不感知动作状态机。
//
// 速度语义（输入都是 -1..1 归一化值，正方向如下）：
//   radial_norm     径向伸缩：+1 = 末端沿当前方位向外伸
//   tangential_norm 切向绕行：+1 = 末端绕 Z 轴切向平移（半径不变绕行）
//   vertical_norm   上下：    +1 = 抬高末端
//   yaw_norm        底座旋转：+1 = 底座正转（整机自转，末端走弧线）
//
// 目标以柱坐标 (theta, r, z) 维护：左摇杆给 (径向, 切向)，右摇杆给
// (自转, 上下)。切向绕行按 Δθ = v/r 折算，r 有下限保护。
//
// 首次从静止开始推杆时，用舵机指令角经正解确定目标起点（没有位置反馈，
// 指令角就是唯一的"当前位置"来源），此后在起点上连续积分，保证点动
// 永远从当前姿态平滑接续，不会跳。
//

#ifndef CRTC2026_ROBOTA_ARM_MANUAL_H
#define CRTC2026_ROBOTA_ARM_MANUAL_H

#pragma once
#include <stdint.h>

/* --------------------------- 手感参数（可调） --------------------------- */
#define ARM_MANUAL_MAX_LINEAR_MPS    0.05f  // 径向/切向平移满杆速度 [m/s]
#define ARM_MANUAL_MAX_VERTICAL_MPS  0.04f  // 上下满杆速度 [m/s]
#define ARM_MANUAL_MAX_YAW_DPS       45.0f  // 底座自转满杆速度 [°/s]
#define ARM_MANUAL_DEADZONE          0.02f  // 归一化输入死区（解码层已有 ±10 原始死区）
#define ARM_MANUAL_MAX_DT_MS         100U   // 单周期dt上限，防长阻塞后目标突跳

/* z 的软限位（相对肩关节转轴，向上为正），占位值，待实物标定 */
#define ARM_MANUAL_Z_MIN_M          -0.02f
#define ARM_MANUAL_Z_MAX_M           0.15f

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化点动模块（须在 servo_init() 之后调用）
 */
void arm_manual_init(void);

/**
 * @brief 周期调用（建议 20ms）：积分目标并下发
 *
 * 四个输入全在死区内时不下发任何指令（保持现状），并回到"待激活"状态；
 * 下次再来非零输入会重新从当前指令角取起点。
 *
 * @param now_ms          当前系统时间 [ms]
 * @param radial_norm     径向速度 -1..1
 * @param tangential_norm 切向速度 -1..1
 * @param vertical_norm   垂直速度 -1..1
 * @param yaw_norm        自转速度 -1..1
 */
void arm_manual_update(uint32_t now_ms,
                       float radial_norm, float tangential_norm,
                       float vertical_norm, float yaw_norm);

#ifdef __cplusplus
}
#endif

#endif //CRTC2026_ROBOTA_ARM_MANUAL_H
