//
// Created by MI on 2026/10/2.
//
// PC13 链路状态指示灯任务（唯一的板载指示输出，见 .cpp 顶部语义表）。

#ifndef CRTC2026_ROBOTA_PS2_LINK_LED_TASK_H
#define CRTC2026_ROBOTA_PS2_LINK_LED_TASK_H

#pragma once
#include "TaskBase.h"

#ifdef __cplusplus
class Ps2LinkLedTask : public TaskBase {
public:
    void run() override;
};
#endif

#ifdef __cplusplus
extern "C" {
#endif

void ps2_link_led_task_init();

#ifdef __cplusplus
}
#endif

#endif //CRTC2026_ROBOTA_PS2_LINK_LED_TASK_H
