//
// 手柄按键事件层：把"当前按住的键"转换成"新按下"事件
//
// 移植自 CRTC_RobotArm 工程 gamepad.c/h，C++ 化。
// 本模块是纯逻辑：只认 GAMEPAD_* 抽象位图，不 include 任何 PS2 头文件。
// PS2 原始按键位 → GAMEPAD_* 位图的映射放在 arm_task（编排层），
// 这样换遥控器（比如改成 UART 键值）时本模块和 robot_control 都不用动。
//
// 注意：事件是锁存的（取出才清零）。调用方必须每个周期都取走事件，
// 否则模式切换后残留事件会一次性触发（本工程由 robot_control 每周期消费）。
//

#ifndef CRTC2026_ROBOTA_GAMEPAD_H
#define CRTC2026_ROBOTA_GAMEPAD_H

#pragma once
#include <stdint.h>
#include <stdbool.h>

/*
 * 软件内部使用的按键位（统一定义格式，不代表 PS2 接收器原始位序）。
 */
typedef enum {
    GAMEPAD_TRIANGLE = (1U << 0),
    GAMEPAD_CIRCLE   = (1U << 1),
    GAMEPAD_SQUARE   = (1U << 2),
    GAMEPAD_CROSS    = (1U << 3),
    GAMEPAD_SELECT   = (1U << 4),
    GAMEPAD_PAD_LEFT = (1U << 5),  // 十字键"左"（预留）
    GAMEPAD_L1       = (1U << 6),  // 左肩键1（语义在消费端：机械臂模式=点动两档调速）
    GAMEPAD_L2       = (1U << 7)   // 左肩键2（语义在消费端：机械臂模式=卸力/恢复）
} GamepadButton_t;

#ifdef __cplusplus
extern "C" {
#endif

void gamepad_init(void);

/* 每获得一份新的按键状态，就调用一次 */
void gamepad_update(uint16_t buttons_pressed, bool connected);

/* 取出尚未处理的"新按下"事件；取出后清空 */
uint16_t gamepad_take_pressed_events(void);

bool gamepad_is_connected(void);

#ifdef __cplusplus
}
#endif

#endif //CRTC2026_ROBOTA_GAMEPAD_H
