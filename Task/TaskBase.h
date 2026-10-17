//
// Created by 20852 on 2025/10/10.
//

#ifndef C8T6_TASKBASE_H
#define C8T6_TASKBASE_H

#pragma once
#include "cmsis_os.h"

#ifdef __cplusplus
class TaskBase {
public:
    // 虚析构函数，确保派生类正确析构
    virtual ~TaskBase() = default;

    // 每个任务必须实现的函数，即逻辑部分
    virtual void run() = 0;

    // 启动任务，包装 osThreadCreate。
    // 返回 false 表示创建失败——最常见的原因是 FreeRTOS 堆不够
    // （xTaskCreate 堆不足时是静默失败，只返回错误码，不会报错也不会断言）。
    bool start(char* name, uint32_t stackSize, osPriority priority) {
        // 必须零初始化：本工程同时开启静态/动态分配，osThreadCreate 会用
        // buffer / controlblock 判断是否走 xTaskCreateStatic，未初始化会导致野指针。
        osThreadDef_t taskDef = {};

        taskDef.name      = name;       // 任务名称
        taskDef.pthread   = taskEntry;  // 静态入口函数
        taskDef.tpriority = priority;   // 任务优先级
        taskDef.instances = 1;          // 任务实例数
        taskDef.stacksize = stackSize;  // 任务栈大小

        handle_ = osThreadCreate(&taskDef, this);  // this 指针传递给静态入口
        return handle_ != nullptr;                 // 创建失败（多为堆不够）时返回 false
    }

protected:
    osThreadId handle_ = nullptr;  // 保存 FreeRTOS 任务句柄

private:
    // 静态入口函数，真正调用对象的 run()
    static void taskEntry(void const* argument) {
        auto* task = static_cast<TaskBase*>(const_cast<void*>(argument));
        task->run();
    }
};
#endif

#endif //C8T6_TASKBASE_H