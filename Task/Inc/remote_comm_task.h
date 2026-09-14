//
// Created by MI on 2026/9/14.
//

#ifndef CRTC2026_ROBOTA_REMOTE_COMM_TASK_H
#define CRTC2026_ROBOTA_REMOTE_COMM_TASK_H

#pragma once
#include "TaskBase.h"
#include "ps2_decode.h"

#ifdef __cplusplus
class RemoteCommTask : public TaskBase {
public:
    void run() override;
};
#endif

#ifdef __cplusplus
extern "C" {
#endif

    void remote_comm_task_init();

#ifdef __cplusplus
}
#endif

#endif //CRTC2026_ROBOTA_REMOTE_COMM_TASK_H