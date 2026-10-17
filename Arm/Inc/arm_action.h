//
// 机械臂动作序列状态机
//
// 移植自 CRTC_RobotArm 工程 arm_action.c/h。保持 C 语言：
// 动作表用了 C99 指定初始化器（[ARM_ACTION_xxx] = ...），C++ 不支持该语法。
// 与原工程的差异只有两处，逻辑零改动：
//   1. 函数名统一为工程的 snake_case 风格；
//   2. 姿态读回改走 arm_get_pose()（原先是直接读舵机层），保持分层。
//
// 只依赖 arm_control，不感知手柄、任务和模式（那些是 robot_control / arm_task 的事）。
//

#ifndef CRTC2026_ROBOTA_ARM_ACTION_H
#define CRTC2026_ROBOTA_ARM_ACTION_H

#pragma once
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief 机械臂动作编号
 */
typedef enum {
    ARM_ACTION_GRAB_MODE_1 = 0,     // 抓取模式1
    ARM_ACTION_GRAB_MODE_2,         // 抓取模式2
    ARM_ACTION_GRAB_MODE_3,         // 抓取模式3
    ARM_ACTION_PLACE,               // 固定放置动作
    ARM_ACTION_COUNT                // 动作总数
} ArmActionId_t;

/**
 * @brief 动作状态机当前状态
 */
typedef enum {
    ARM_ACTION_STATE_IDLE = 0,  // 空闲，等待新动作
    ARM_ACTION_STATE_RUNNING,   // 正在执行动作
    ARM_ACTION_STATE_FINISHED,  // 动作序列已经完成
    ARM_ACTION_STATE_ERROR      // 动作执行错误
} ArmActionState_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化动作状态机（须在 servo_init() 之后调用，要读回初始姿态）
 */
void arm_action_init(void);

/**
 * @brief 启动一个机械臂动作
 * @param action_id 动作编号
 * @param now_ms    当前系统时间 [ms]
 * @return true  动作成功启动 / false 编号无效或动作无法启动
 */
bool arm_action_start(ArmActionId_t action_id, uint32_t now_ms);

/**
 * @brief 更新动作状态机
 *
 * 需要被周期性反复调用，建议间隔不超过 20 ms。
 * 内部每约 20 ms 输出一次插值姿态；运动结束后才开始步骤保持计时。
 *
 * @param now_ms 当前系统时间 [ms]
 */
void arm_action_update(uint32_t now_ms);

/**
 * @brief 取消当前动作
 *
 * 取消后停止继续执行后续姿态，但不会关闭舵机 PWM。
 * 已发送的目标不会回退；无反馈舵机可能继续追赶最后一个目标。
 */
void arm_action_cancel(void);

/**
 * @brief 获取当前动作状态
 */
ArmActionState_t arm_action_get_state(void);

#ifdef __cplusplus
}
#endif

#endif //CRTC2026_ROBOTA_ARM_ACTION_H
