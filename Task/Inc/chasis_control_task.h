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
 * 点按 L1 在"快速/慢速"之间翻转（上电默认快速模式）。
 * 慢速模式下，摇杆推到底也只有最大速度的 CHASIS_SLOW_MODE_SCALE 倍。
 * 想改慢速比例就改这里（例如 0.3f = 30%）。
 */
#define CHASIS_SLOW_MODE_SCALE    0.8f

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