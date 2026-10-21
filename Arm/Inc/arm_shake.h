//
// 机械臂末端抖动（□ 键）：夹爪快速多次开合 + 小臂小幅上下摆动
//
// 与 arm_action（关键帧序列）并列的另一种自动运动：**时间驱动**的振荡。
// 关键帧引擎是"逐帧到位 + 停留"的走走停停，做不了亚秒级的连续摆动，
// 所以抖动单独成模块——只占用肘（小臂）与夹爪，底座/肩保持不动。
//
// 运动定义（以触发那一刻的位形为基准，只记相对量）：
//   - 肘命令角 = 触发时的肘角 + 振幅 × sin(2π·f·t)，整周期结束自动回中；
//   - 夹爪在"抖动闭合档"（GRIPPER_SHAKE_CLOSE_DEG，不完全闭，留缝夹住
//     方块）与"张开档"之间按半周期切换，每周期开合一次；
//   - 结束时夹爪恢复触发前的角度/档位，肘回到触发时的角度。
//
// 打断语义与动作键同规矩（E5）：推杆 / SELECT / L2 / ○ / △ / × / 十字键
// 都会先停抖动；打断时夹爪同样恢复，肘停在当前位置。
//

#ifndef CRTC2026_ROBOTA_ARM_SHAKE_H
#define CRTC2026_ROBOTA_ARM_SHAKE_H

#pragma once
#include <stdint.h>

/* ------------------------------ 抖动参数（可调） ------------------------------ */
/* 摆动频率与振幅决定小臂峰值角速度 = 振幅 × 2π × 频率（当前 8° × 2π × 2.5Hz
 * ≈ 126°/s 峰值）。抖动是有意为之的快速小幅运动，允许超过点动的 60°/s；
 * 嫌太猛先减振幅，再降频率。 */
#define ARM_SHAKE_FREQ_HZ        2.5f    // 小臂摆动频率 [Hz]，夹爪同频开合
#define ARM_SHAKE_ELBOW_AMP_DEG  8.0f    // 小臂摆幅 [°]（肘命令角绕触发值的振幅）
#define ARM_SHAKE_CYCLES         3.0f    // 总共摆动的整周期数

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化（清状态机）
 */
void arm_shake_init(void);

/**
 * @brief 启动一次抖动（以当前肘角/夹爪角为基准）
 * @return false 表示已在抖动中，或读回当前姿态失败
 */
bool arm_shake_start(uint32_t now_ms);

/**
 * @brief 推进抖动（与 ArmTask 同周期 20ms 调用；未在抖动中时是空操作）
 */
void arm_shake_update(uint32_t now_ms);

/**
 * @brief 停止抖动（夹爪恢复触发前的角度/档位；肘停在当前位置）
 */
void arm_shake_cancel(void);

/**
 * @brief 是否正在抖动
 */
bool arm_shake_is_running(void);

#ifdef __cplusplus
}
#endif

#endif //CRTC2026_ROBOTA_ARM_SHAKE_H
