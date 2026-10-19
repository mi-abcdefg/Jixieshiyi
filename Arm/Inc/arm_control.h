//
// 机械臂关节空间控制：完整姿态（四关节角）→ 四个舵机
//
// 本模块是机械臂"关节空间"的唯一入口：动作序列（arm_action）和手动点动
// （arm_manual）都只调 arm_set_pose() / arm_get_pose()，不直接碰舵机驱动，
// 这样舵机编号→结构的映射改动只影响本层。
//
// 头文件按 C 兼容写（arm_action.c 是 C 文件，会 include 本头文件）。
//

#ifndef CRTC2026_ROBOTA_ARM_CONTROL_H
#define CRTC2026_ROBOTA_ARM_CONTROL_H

#pragma once
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief 机械臂完整姿态（四个关节角，单位：度）
 */
typedef struct
{
    float gripper_deg;   // 夹爪
    float link1_deg;     // 第一段连杆
    float link2_deg;     // 第二段连杆
    float base_z_deg;    // 绕 Z 轴旋转（底座）
} ArmPose_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 设置机械臂完整姿态（四个关节一起下发）
 * @return true  四个舵机指令全部设置成功
 * @return false 参数无效或至少一个舵机设置失败
 */
bool arm_set_pose(const ArmPose_t *pose);

/**
 * @brief 读回最近一次下发的完整姿态（非实际反馈；尚未下发过目标时返回 false）
 */
bool arm_get_pose(ArmPose_t *pose);

/**
 * @brief 只设置夹爪角度，其余三个关节保持当前指令值
 * @return false 表示姿态尚未初始化或舵机设置失败
 */
bool arm_set_gripper(float angle_deg);

/**
 * @brief 机械臂整体卸力/恢复（卸力 = 四路舵机停止输出脉冲，无保持力矩）
 *
 * 卸力期间所有关节写入被拒绝；恢复后自动回到卸力前姿态。
 */
void arm_set_torque_enabled(bool enabled);

/**
 * @brief 当前是否处于有力状态（true = 正常输出）
 */
bool arm_get_torque_enabled(void);

#ifdef __cplusplus
}
#endif

#endif //CRTC2026_ROBOTA_ARM_CONTROL_H
