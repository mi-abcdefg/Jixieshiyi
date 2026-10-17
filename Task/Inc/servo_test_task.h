//
// Created by MI on 2026/10/17.
//

#ifndef CRTC2026_ROBOTA_SERVO_TEST_TASK_H
#define CRTC2026_ROBOTA_SERVO_TEST_TASK_H

#pragma once
#include "TaskBase.h"

/* ===================== 舵机逐台自检（台架测试用，临时任务） =====================
 * 流程：servo_init() 复位到初始姿态 → 等 2s → 从下到上逐台：
 *   底座 MG995(PB6) → 肩 SG90(PB7) → 肘 SG90(PB8) → 夹爪 SG90(PB9)
 * 每台正转 45°（限幅内）、停 0.8s、转回初始位，再测下一台。
 *
 * 用途：
 *   1. 确认四路舵机接线与 servo_driver 配置表一致；
 *   2. 观察每台"角度+"对应的物理旋转方向，为 reversed 标定做准备。
 *
 * 启用方式：Core/Src/freertos.c 的 USER CODE Init 区里用
 *   servo_test_task_init() 替换 arm_task_init()（该文件内有注释说明）。
 * 测完恢复 arm_task_init() 即可，本任务不再被创建。
 * ============================================================================ */

#ifdef __cplusplus
class ServoTestTask : public TaskBase {
public:
    void run() override;
};
#endif

#ifdef __cplusplus
extern "C" {
#endif

void servo_test_task_init();

#ifdef __cplusplus
}
#endif

#endif //CRTC2026_ROBOTA_SERVO_TEST_TASK_H
