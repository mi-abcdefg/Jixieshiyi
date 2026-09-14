//
// Created by MI on 2026/9/14.
//

#include "remote_comm_task.h"

void RemoteCommTask::run() {
    while (1) {
        ps2_read_and_decode_JoystickData();
        osDelay(10);
    }
}

extern "C" {

    RemoteCommTask remote_comm_task;
    void remote_comm_task_init() {
        remote_comm_task.start("RemoteCommTask", 256, osPriorityNormal);
    }

}