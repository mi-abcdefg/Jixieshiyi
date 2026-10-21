//
// 机械臂动作引擎：按路径表逐帧逼近关键帧
//
// 动作 = 一串 KeyframeId（帧数据全在 arm_keyframe 的表里）。两种形态：
//   1. 固定路径：所有关键帧依次推进（卸力恢复后的"脱离"动作）；
//   2. 双路径动作：启动那一刻按"篮内/篮外"位形二选一，之后不再切换
//      （回收缩、篮内外转移）。
// 本模块**不认识按键、不认识 PS2、不认识控制模式**：按键决策在 arm_button，
// 周期编排在 arm_task，关节写入统一经 arm_control。
//
// 推进细节：
//   - 20ms 固定节拍（须与 ArmTask 周期严格一致，ARM_ACTION_TICK_S 同源）；
//   - 每帧先把用户角换算为命令角目标（kKeep 字段 = 保持该关节当前值），
//     三关节各以**恒角速度**逼近（底座 90°/s、肩 60°/s、肘 60°/s，
//     与点动快速档同源，见 arm_manual.h）；
//   - 三关节全部到位后按该帧定义切换夹爪，停留 ~300ms 进下一帧；
//   - 状态机只有 IDLE / RUNNING：取消与正常完成都直接回 IDLE，不留
//     FINISHED/ERROR 中间态——否则"动作完成后按键表现异常"（交接文档
//     踩坑 12），而且完成后必须能立即触发下一个动作。
//
// 打断语义：本模块只提供 cancel()；"推杆 / SELECT / 再次动作键打断"
// 由 arm_button 判定后调用（阈值见 ARM_MANUAL_INTERRUPT_DEADZONE）。
//

#ifndef CRTC2026_ROBOTA_ARM_ACTION_H
#define CRTC2026_ROBOTA_ARM_ACTION_H

#pragma once
#include <stdint.h>

/**
 * @brief 动作编号
 */
enum class ActionId : uint8_t {
    kRetract = 0,   // 回收缩（△）：经篮上/外侧高点 → 收缩位，夹爪闭合
    kTransit,       // 篮内外转移（×）：篮外→篮内，或篮内→篮外（出篮终点夹爪闭合）
    kEscape,        // 脱离（L2 恢复后自动）：先只给大臂（肩）通电撑直，再大臂保持＋底座回中
    kCount
};

/**
 * @brief 动作状态机状态
 */
enum class ActionState : uint8_t {
    kIdle = 0,      // 空闲：可以立即启动下一个动作
    kRunning        // 正在执行
};

/**
 * @brief 初始化动作引擎（须在 servo_init() 之后调用）
 */
void arm_action_init(void);

/**
 * @brief 启动一个动作
 * @param id     动作编号
 * @param now_ms 当前系统时间 [ms]（与 update 同一时间基准）
 * @return true 成功启动；false 编号非法、已有一个动作在跑，或读回姿态失败
 * @note  双路径动作在此刻按当前位形选定路径，之后不再切换。
 */
bool arm_action_start(ActionId id, uint32_t now_ms);

/**
 * @brief 推进动作状态机（须 20ms 周期调用，与 ArmTask 周期一致）
 * @param now_ms 当前系统时间 [ms]
 */
void arm_action_update(uint32_t now_ms);

/**
 * @brief 取消当前动作：关节停在当前位置，立即回 IDLE（不关舵机输出）
 */
void arm_action_cancel(void);

/**
 * @brief 取当前状态
 */
ActionState arm_action_get_state(void);

#endif //CRTC2026_ROBOTA_ARM_ACTION_H
