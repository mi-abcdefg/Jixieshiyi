//
// 机械臂动作引擎：区域路径 + 关键帧编号序列
//
// 动作 = 一串命名关键帧编号（帧数据在 arm_keyframe 表中）。分两种形态：
//   1. 固定路径：所有关键帧依次推进；
//   2. 区域路径：启动时按"篮内/篮外"判定（底座用户角 ≥160° 且肩 ≤100°
//      判为篮内）选择篮内路径或篮外路径——收缩、转移动作即此形态。
// 推进方式为恒定关节角速度步进（与点动快速档一致，速度宏同源）；
// 每个关键帧到位后按帧内状态切换夹爪，停留片刻进入下一帧。
// 卸力恢复的"自动伸展脱离"已归并为本引擎的 ARM_ACTION_ESCAPE 动作。
// 运行中再次 L2 / SELECT / 手动推杆会取消动作并保持当前位置（取消语义
// 由决策层与编排层负责，本模块只提供 cancel）。
//

#ifndef CRTC2026_ROBOTA_ARM_ACTION_H
#define CRTC2026_ROBOTA_ARM_ACTION_H

#pragma once
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief 动作编号
 */
typedef enum {
    ARM_ACTION_RETRACT = 0,   // 回收缩：篮内经篮上高点、篮外经外侧高点 → 收缩位
    ARM_ACTION_ESCAPE,        // 脱离伸展：大臂小臂升到中点 → 底座转到中点
    ARM_ACTION_TRANSIT,       // 篮内外转移：篮外 经外侧→篮上高点 到篮内点；
                              //         篮内 经篮上→外侧高点 到拾取低点
    ARM_ACTION_PLACE,         // 固定放置（占位，暂无按键绑定）
    ARM_ACTION_GRAB_MODE_1,   // 抓取模式1（占位）
    ARM_ACTION_GRAB_MODE_3,   // 抓取模式3（占位）
    ARM_ACTION_COUNT          // 动作总数
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

/**
 * @brief 初始化动作状态机（须在 servo_init() 之后调用）
 */
#ifdef __cplusplus
extern "C" {
#endif

void arm_action_init(void);

/**
 * @brief 启动一个动作
 * @param action_id 动作编号
 * @param now_ms    当前系统时间 [ms]
 * @return true  动作成功启动 / false 编号无效或动作无法启动
 */
bool arm_action_start(ArmActionId_t action_id, uint32_t now_ms);

/**
 * @brief 推进动作状态机（建议 20ms 周期调用）
 * @param now_ms 当前系统时间 [ms]
 */
void arm_action_update(uint32_t now_ms);

/**
 * @brief 取消当前动作（保持当前位置；不关闭舵机输出）
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
