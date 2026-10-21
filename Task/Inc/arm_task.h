//
// Created by MI on 2026/10/17.
//

#ifndef CRTC2026_ROBOTA_ARM_TASK_H
#define CRTC2026_ROBOTA_ARM_TASK_H

#pragma once
#include "TaskBase.h"

#ifdef __cplusplus
/* 机械臂任务：本文件是机械臂子系统的唯一编排层：
 *  - PS2 原始按键位 → arm_button 抽象位图；摇杆归一化
 *  - 按控制对象模式（control_mode）分流摇杆用途
 *  - 每周期：喂按键/摇杆 → arm_button_apply（决策）→ arm_manual_update
 *    （点动）→ arm_action_update（动作引擎）
 * 下层模块（arm_button / arm_action / arm_keyframe / arm_manual /
 * arm_control / gripper / arm_ik）互相之间以及与 PS2 之间都不直接依赖；
 * 只有 arm_button 认识"按键语义"，只有本任务认识 PS2。 */
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
