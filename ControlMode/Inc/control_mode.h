//
// 操作对象模式：底盘 ⇄ 机械臂
//
// 独立的顶层小模块，因为它是底盘任务和机械臂任务之间的共享状态，
// 放进任何一方都会造成"下层模块 include 上层任务"的反向依赖。
//
// 并发说明：只有 ChasisControlTask 调 toggle()（R2 边沿检测的唯一所有者），
// 其余任务只读。32 位对齐变量的读写在 Cortex-M3 上是单指令的，无撕裂问题。
//

#ifndef CRTC2026_ROBOTA_CONTROL_MODE_H
#define CRTC2026_ROBOTA_CONTROL_MODE_H

#pragma once
#include <stdint.h>

typedef enum {
    CONTROL_MODE_CHASSIS = 0,   // 底盘模式（上电默认）
    CONTROL_MODE_ARM            // 机械臂模式
} ControlMode_t;

#ifdef __cplusplus
extern "C" {
#endif

/* 点按切换一次（只在 R2 边沿检测处调用） */
void control_mode_toggle(void);

ControlMode_t control_mode_get(void);

#ifdef __cplusplus
}
#endif

#endif //CRTC2026_ROBOTA_CONTROL_MODE_H
