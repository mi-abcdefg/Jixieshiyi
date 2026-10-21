//
// 机械臂手动点动（挖掘机式）：每个摇杆轴直接驱动一个关节
//
// 与早期版本的差别：本模块曾经做"柱坐标目标积分 + 逆解算"的笛卡尔点动，
// 实测手感不直觉（切向绕行是弧线、奇异区变形），已整体替换为关节空间
// 直控——每个轴一个关节、恒定角速度、所见即所得，没有奇异区也没有
// 坐标系心智负担。逆解算（arm_ik）暂无调用方，保留备用。
//
// 速度语义：四个输入都是 -1..1 归一化值，正方向 = 该关节命令角增大方向。
// 松杆（死区内）该关节保持不动；全部松杆时不写任何舵机。
//
// 限幅：每关节的步长 = 杆量 × 该关节最大角速度 × dt，命令角从配置表
// 量程（servo_get_angle_range，唯一数据源）内插值积分，到头即停。
//

#ifndef CRTC2026_ROBOTA_ARM_MANUAL_H
#define CRTC2026_ROBOTA_ARM_MANUAL_H

#pragma once
#include <stdint.h>

/* --------------------------- 手感参数（可调） --------------------------- */
/* 各关节最大角速度 [°/s]（满杆）。MG995 空载约 350°/s、SG90 约 660°/s，
 * 这里取远低于空载的值保证带载可控；嫌慢逐个加大。
 * 注意：动作引擎有**自己的巡航速度**（arm_action.cpp 的 kCruise*Dps，
 * 比本表快），与本表脱钩——调点动手感不影响动作速度。 */
#define ARM_MANUAL_BASE_MAX_DPS      90.0f  // 底座（方位角）
#define ARM_MANUAL_SHOULDER_MAX_DPS  60.0f  // 肩（连杆仰角）
#define ARM_MANUAL_ELBOW_MAX_DPS     60.0f  // 肘（相对肩连杆的夹角）

#define ARM_MANUAL_DEADZONE          0.02f  // 归一化输入死区（解码层已有 ±10 原始死区）
/* 动作打断阈值：动作运行中，杆量越过该值才视为"手动接管意图"。
 * **必须显著大于点动死区**——否则摇杆静置漂移会把刚启动的动作在第一帧
 * 就取消，实测表现为"动作永远停在第一个过渡点"（交接文档踩坑 13）。
 * 判定逻辑在 arm_button（E5 的绑定处），本宏是唯一阈值来源。 */
#define ARM_MANUAL_INTERRUPT_DEADZONE 0.15f
#define ARM_MANUAL_MAX_DT_MS         100U   // 单周期dt上限，防长阻塞后步长突跳

/* ------------------------------ 调速档位 ------------------------------ */
/* 两档循环：0 = 快速（上表速度），1 = 慢速（各关节角速度 × 比例）。
 * 上电默认快速。切档入口是 L1 边沿，绑定在 arm_button（机械臂模式）。
 * 注意：动作引擎的速度**不受**本档影响，巡航速度也是独立的（见上）。 */
#define ARM_MANUAL_SLOW_DPS_SCALE    0.35f  // 2026/10/21 实测调小（原 0.5）：慢速档灵敏度再低一点

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化点动模块（须在 servo_init() 之后调用）
 */
void arm_manual_init(void);

/**
 * @brief 点动调速档循环切换（快 → 慢 → 快……）
 * @note  由 arm_button 在 L1 边沿调用（机械臂模式下，动作运行中也允许切）
 */
void arm_manual_cycle_speed_level(void);

/**
 * @brief 周期调用（建议 20ms）：按各轴杆量驱动对应关节
 *
 * 某轴在死区内时该关节保持不动；四轴全在死区内则不下发任何指令。
 *
 * @param now_ms        当前系统时间 [ms]
 * @param base_norm     底座方位角速度 -1..1（正 = 方位角增大，绕 +Z 逆时针）
 * @param shoulder_norm 肩仰角速度   -1..1（正 = 连杆抬高）
 * @param elbow_norm    肘角速度     -1..1（正 = 相对夹角增大，肘连杆上抬）
 * @param gripper_norm  夹爪角速度   -1..1（正 = 命令角增大；开/闭的物理方向实物标定）
 */
void arm_manual_update(uint32_t now_ms,
                       float base_norm, float shoulder_norm,
                       float elbow_norm, float gripper_norm);

#ifdef __cplusplus
}
#endif

#endif //CRTC2026_ROBOTA_ARM_MANUAL_H
