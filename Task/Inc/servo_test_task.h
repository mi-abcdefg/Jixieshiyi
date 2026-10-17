//
// Created by MI on 2026/10/17.
//

#ifndef CRTC2026_ROBOTA_SERVO_TEST_TASK_H
#define CRTC2026_ROBOTA_SERVO_TEST_TASK_H

#pragma once
#include "TaskBase.h"

/* ===================== 舵机通道辨识扫动（台架测试用，临时任务） =====================
 * 流程：servo_init() 复位到初始姿态 → 等 2s → 无限循环：
 *   对每台（配置表通道 PB6→PB9）：
 *     PC13 灯报数（闪 k 下 = 通道 k）→ 中位 +45° → 中位 −45° → 回中位；
 *   一轮结束后灯长亮 2s，随后开始下一轮。断电即停止。
 *
 * 判读（灯闪 k 下时动的那台 = 实际接在通道 k 上的舵机）：
 *   - MG995 在闪 k 下时动 → 它真实接在通道 k，把四台对应关系告诉我，
 *     我按实际重排 servo_driver 配置表（或把插头改回与配置表一致）；
 *   - MG995 哪一轮都不动（嗡嗡响或完全死）→ 供电/舵机本体问题
 *     （MG995 堵转电流大，与 SG90 共电源时最容易被拉死）；
 *   - 四台台号与预期全对 → 接线无误，问题在运行链路，再往软件深挖。
 *
 * 启用方式（Core/Src/freertos.c USER CODE Init 区）：
 *   - servo_test_task_init() 替换 arm_task_init()（避免抢写脉宽）；
 *   - 关闭 test_task_init()（TestTask 的状态灯与本任务的报数灯冲突）。
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
