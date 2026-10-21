//
// 机械臂关节空间控制：底座/肩/肘三个关节角 → 对应舵机（夹爪除外）
//
// 本模块是四个舵机的下发/读回入口：手动点动（arm_manual）与动作引擎
// （arm_action）都通过 arm_set_pose() / arm_get_pose() 操作关节，不直接
// 碰舵机驱动。夹爪的角度写入经 gripper 模块，但"整臂姿态"里仍带夹爪
// 字段，方便一次读写整套姿态。
//
// 给力（力矩）有两种粒度：
//   - 整臂：arm_set_torque_enabled()（L2 卸力 ⇄ 恢复）；
//   - 逐路：arm_set_joint_torque()（"脱离"动作：大臂（肩）先给力撑直，底座与
//     其余关节继续保持卸力，随后再逐路给力）。
//   "卸力模式"是用户层面的状态（L2 按过之后），与"个别关节暂时无力"
//   分开记录：arm_is_relaxed() 反映前者，arm_get_joint_torque_mask()
//   反映后者。
//
// 头文件按 C 兼容写（工程约定：机械臂层头文件保持 C 兼容）。
//

#ifndef CRTC2026_ROBOTA_ARM_CONTROL_H
#define CRTC2026_ROBOTA_ARM_CONTROL_H

#pragma once
#include <stdint.h>
#include <stdbool.h>

#include "servo_driver.h"   // ServoId_t（arm_clamp_angle 的参数）

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

/* ------------------------- 关节位掩码（逐路操作） -------------------------
 * 位序与 ArmPose_t 的字段一一对应；一次只下发/只给力其中若干关节。 */
#define ARM_JOINT_GRIPPER    (1u << 0)
#define ARM_JOINT_SHOULDER   (1u << 1)
#define ARM_JOINT_ELBOW      (1u << 2)
#define ARM_JOINT_BASE_Z     (1u << 3)
#define ARM_JOINT_ALL        (ARM_JOINT_GRIPPER | ARM_JOINT_SHOULDER | \
                              ARM_JOINT_ELBOW   | ARM_JOINT_BASE_Z)

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
 * @brief 只下发 mask 里指定的关节（其余关节不写、保持原有脉宽）
 * @param pose       完整姿态；未选中的关节字段会被忽略
 * @param joint_mask ARM_JOINT_xxx 的按位或
 * @return false 参数无效或选中的关节里至少一个写入失败
 */
bool arm_set_pose_masked(const ArmPose_t *pose, uint32_t joint_mask);

/**
 * @brief 读回最近一次下发的完整姿态（非实际反馈；尚未下发过目标时返回 false）
 */
bool arm_get_pose(ArmPose_t *pose);

/**
 * @brief 机械臂整体卸力/恢复（卸力 = 四路舵机停止输出脉冲，无保持力矩）
 *
 * 卸力期间所有关节写入被拒绝；恢复后按卸力前记录的命令角重写脉宽。
 * @note 恢复走这条路径会把整条臂一起弹回卸力前姿态。若要"逐路、按动作
 *       顺序"给力（卸力恢复的"脱离"动作），用 arm_set_relaxed(false) 退出
 *       卸力模式，再由动作自己用 arm_set_joint_torque() 逐路给力。
 */
void arm_set_torque_enabled(bool enabled);

/**
 * @brief 是否处于"臂有力"状态（= 不处于卸力模式）
 * @note 这是用户层面的模式判断（L2 按过没有），不代表每一路此刻都有脉冲；
 *       逐路状态用 arm_get_joint_torque_mask() 查询。
 */
bool arm_get_torque_enabled(void);

/**
 * @brief 是否处于卸力模式（L2 卸力之后、恢复之前）
 */
bool arm_is_relaxed(void);

/**
 * @brief 设置卸力模式标志
 * @param relaxed true  = 进入卸力模式并四路卸力（等同 arm_set_torque_enabled(false)）
 *                false = 只退出卸力模式，**不给力**——脉宽交给随后的动作
 *                        （"脱离"）逐路恢复
 */
void arm_set_relaxed(bool relaxed);

/**
 * @brief 逐路给力/卸力（不改卸力模式标志）
 * @param joint_mask ARM_JOINT_xxx 的按位或
 * @param enabled    true = 给力（有记录角时按记录角重发脉宽），false = 该路卸力
 * @note 只对"状态需要变"的路生效，幂等。
 */
void arm_set_joint_torque(uint32_t joint_mask, bool enabled);

/**
 * @brief 当前哪些关节有力（ARM_JOINT_xxx 位掩码）
 */
uint32_t arm_get_joint_torque_mask(void);

/**
 * @brief 把命令角夹到该关节配置表里的量程内
 *
 * 目标越界时舵机自己会限幅，但读回的是夹后的值——上层若拿越界目标做
 * "逼近/解算"，就永远收敛不了（动作会卡在第一帧、末端点动会漂）。
 * 所以"目标必须先夹进量程"这条规则的实现只放在这里一处。
 */
float arm_clamp_angle(ServoId_t servo, float cmd_deg);

#ifdef __cplusplus
}
#endif

#endif //CRTC2026_ROBOTA_ARM_CONTROL_H
