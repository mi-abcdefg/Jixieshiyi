//
// Created by MI on 2026/10/17.
//

#ifndef CRTC2026_ROBOTA_SERVO_TEST_TASK_H
#define CRTC2026_ROBOTA_SERVO_TEST_TASK_H

#pragma once
#include "TaskBase.h"

/* ===================== 舵机逐台自检（台架测试用，临时任务） =====================
 * 流程：servo_init() 复位到初始姿态 → 等 2s → 无限循环：
 *   对每台（底座 PB6 → 肩 PB7 → 肘 PB8 → 夹爪 PB9）：
 *     PC13 灯报数（闪 k 下 = 测第 k 台）→ 正转 45° → 停 0.8s → 回位；
 *   一轮结束后灯长亮 2s，随后开始下一轮。断电即停止。
 *
 * 用途：
 *   1. 灯闪了但对应舵机没动 → 该路接线/供电问题；
 *   2. 动的台号与预期对不上 → 插头串线，改 servo_driver 配置表；
 *   3. 观察每台"角度+"的物理转向，为 reversed 标定做准备。
 *
 * 启用方式（Core/Src/freertos.c USER CODE Init 区）：
 *   - servo_test_task_init() 替换 arm_task_init()（避免抢写脉宽）；
 *   - 关闭 test_task_init()（TestTask 的状态灯语义与本任务的报数灯冲突）。
 * 标定完成后恢复这三行即可。
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
