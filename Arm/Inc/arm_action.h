//
// 机械臂动作引擎：条件路径 + 关键帧序列
//
// 与旧版（固定步骤表 + smoothstep 插值）的区别：
//   1. 动作由关键帧组成（用户角约定，见 arm_keyframe），可含"保持当前值"
//      字段（ARM_KF_KEEP），关节互相独立地逼近各自目标；
//   2. 支持条件入口：启动时按当前姿态选择不同的过渡关键帧（如回收动作
//      先判断篮内/篮外，再经篮上高点或外侧高点过渡）；
//   3. 推进方式为恒定关节角速度步进（与点动快速档一致），卸力恢复的
//      "自动伸展脱离"已归并为本引擎的一个动作（ARM_ACTION_ESCAPE）。
//
// 运行语义：
//   - 每个关键帧：三关节步进到位 → 按帧内状态切换夹爪 → 停留 HOLD 后下一帧；
//   - 动作运行中再次 L2 / SELECT / 手动推杆都会取消（保持当前位置），
//     取消语义由决策层与编排层负责，本模块只提供 cancel。
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
