//
// Created by MI on 2026/10/17.
//

#ifndef CRTC2026_ROBOTA_ARM_TASK_H
#define CRTC2026_ROBOTA_ARM_TASK_H

#pragma once
#include "TaskBase.h"

#ifdef __cplusplus
/* 机械臂任务：本文件是机械臂子系统的唯一编排层：
 *  - PS2 解码结果 → gamepad 位图 / 摇杆归一化（全工程只有这里做 PS2 映射）
 *  - 按控制对象模式（control_mode）分流摇杆用途
 *  - 仲裁"手动点动打断自动动作"
 * 下层模块（gamepad / robot_control / arm_manual / arm_action / arm_ik）
 * 互相之间以及与 PS2 之间都不直接依赖。 */
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
