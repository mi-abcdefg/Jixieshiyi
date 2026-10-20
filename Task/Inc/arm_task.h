//
// Created by MI on 2026/10/17.
//

#ifndef CRTC2026_ROBOTA_ARM_TASK_H
#define CRTC2026_ROBOTA_ARM_TASK_H

#pragma once
#include "TaskBase.h"

#ifdef __cplusplus
/* 机械臂任务：本文件是机械臂子系统的唯一编排层：
 *  - PS2 摇杆归一化（按键映射随动作序列一并移除，重建时恢复）
 *  - 按控制对象模式（control_mode）分流摇杆用途
 * 下层模块（arm_manual / arm_control / gripper / arm_ik）互相之间
 * 以及与 PS2 之间都不直接依赖。 */
class ArmTask : public TaskBase {
public:
    void run() override;
};
#endif

#ifdef __cplusplus
extern "C" {
#endif

void arm_task_init();

#ifdef __cplusplus
}
#endif

#endif //CRTC2026_ROBOTA_ARM_TASK_H
