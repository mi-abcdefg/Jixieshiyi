//
// Created by MI on 2026/9/14.
//

#include "remote_comm_task.h"

void RemoteCommTask::run() {
    // 先让手柄切到模拟模式，否则摇杆字节不会更新（表现为"按键正常、摇杆无效"）。
    // 放在任务里而不是 PS2_Init() 里：delay_us() 依赖 FreeRTOS 的 SysTick，
    // 调度器启动之后时序才准（PS2_Init() 是在调度器启动前被调用的）。
    PS2_ConfigAnalogMode();

    while (1) {
        ps2_read_and_decode_JoystickData();
        osDelay(10);
    }
}

extern "C" {

    RemoteCommTask remote_comm_task;
    void remote_comm_task_init() {
        PS2_Init();
        remote_comm_task.start("RemoteCommTask", 256, osPriorityNormal);
    }

}