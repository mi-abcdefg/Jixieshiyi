//
// Created by MI on 2026/10/2.
//

#ifndef CRTC2026_ROBOTA_TEST_TASK_H
#define CRTC2026_ROBOTA_TEST_TASK_H

#pragma once
#include "TaskBase.h"
#include "stm32f1xx.h"
#ifdef __cplusplus
class TestTask : public TaskBase {
public:
    void run() override;
};
#endif

#ifdef __cplusplus
extern "C" {
#endif

    void test_task_init();

#ifdef __cplusplus
}
#endif

#endif //CRTC2026_ROBOTA_TEST_TASK_H