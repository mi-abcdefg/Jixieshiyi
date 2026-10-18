//
// Created by MI on 2026/9/13.
//

#ifndef CRTC2026_ROBOTA_CHASIS_CONTROL_TASK_H
#define CRTC2026_ROBOTA_CHASIS_CONTROL_TASK_H

#pragma once
#include "TaskBase.h"
#include "mecanum_kinematic.h"
#include "motor_pwm_driver.h"
#include "ps2_decode.h"

/* ---------------------------- 速度模式 ----------------------------
 * 点按 L1 在三档间循环：快速（满速）→ 中速（比例 CHASIS_SLOW_MODE_SCALE）
 * → 固定慢速（方向随摇杆、速度恒定，与杆量无关）→ 快速……上电默认快速。 */
#define CHASIS_SLOW_MODE_SCALE    0.8f

/* 固定慢速档：平移/旋转速度 = 满量程 × 下列比例，摇杆只提供方向 */
#define CHASIS_FIXED_SLOW_SPEED_RATIO   0.15f  // 平移 0.15 × 2.0 = 0.3 m/s
#define CHASIS_FIXED_SLOW_OMEGA_RATIO   0.25f  // 旋转 0.25 × 满量程角速度

/* ------------------------ 原地旋转方向翻转 ------------------------
 * 推右摇杆的旋转方向与整车实际转向相反时置 1（B 车实测需要）。
 * 只取反底盘旋转指令 omega，不影响 vx/vy，也不影响机械臂的底座自转。 */
#define CHASIS_OMEGA_INVERT       1

/* ------------------------ 十字键定距平移 ------------------------
 * 仅底盘模式。点按（按压 < CHASIS_PAD_TAP_MS）朝该方向平移一段固定
 * 距离（= 速度 × CHASIS_PAD_STEP_MS）；长按则按住期间持续移动、松手即停。
 * 上=前 下=后 左=左移 右=右移，两键同按 = 斜向。平移期间不响应旋转。 */
#define CHASIS_PAD_SPEED_MPS      0.30f
#define CHASIS_PAD_TAP_MS         300U
#define CHASIS_PAD_STEP_MS        400U   // 0.30m/s × 400ms = 12cm/步

#ifdef __cplusplus
class ChasisControlTask : public TaskBase {
public:
    void run() override;
};
#endif

#ifdef __cplusplus
extern "C" {
#endif

void chasis_control_task_init();

#ifdef __cplusplus
}
#endif

#endif //CRTC2026_ROBOTA_CHASIS_CONTROL_TASK_H