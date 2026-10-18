//
// Created by MI on 2026/9/13.
//

#ifndef CRTC2026_ROBOTA_MOTOR_PWM_DRIVER_H
#define CRTC2026_ROBOTA_MOTOR_PWM_DRIVER_H

#pragma once
#include "stm32f1xx_hal.h"
#include "tim.h"

// 轮子PWM定时器和通道定义
#define TIM_FL_1 &htim2
#define TIM_FL_2 &htim2
#define TIM_FR_1 &htim2
#define TIM_FR_2 &htim2
#define TIM_BL_1 &htim3
#define TIM_BL_2 &htim3
#define TIM_BR_1 &htim3
#define TIM_BR_2 &htim3
#define CH_FL_1 TIM_CHANNEL_1
#define CH_FL_2 TIM_CHANNEL_2
#define CH_FR_1 TIM_CHANNEL_3
#define CH_FR_2 TIM_CHANNEL_4
#define CH_BL_1 TIM_CHANNEL_1
#define CH_BL_2 TIM_CHANNEL_2
#define CH_BR_1 TIM_CHANNEL_3
#define CH_BR_2 TIM_CHANNEL_4

// 轮速指令满量程：0 ~ MOTOR_CMD_MAX 对应 0 ~ 100% 占空比。
// 驱动内部会按定时器当前的 ARR 换算成比较值，所以改 PWM 频率（ARR）不需要动这个值。
#define MOTOR_CMD_MAX 1000

// DRV8833 使能脚（nSLEEP）：高电平 = 输出使能，低电平 = 休眠关断。
// 引脚定义在 main.h：DRV_1 = PC15，DRV_2 = PB10。
#define MOTOR_DRV_ENABLE_LEVEL    GPIO_PIN_SET
#define MOTOR_DRV_DISABLE_LEVEL   GPIO_PIN_RESET

/* ---------------------- 电机接线方向修正 ----------------------
 * 如果某个轮子的电机正负极接反（转向和预期相反），不用去改线，
 * 把对应的一项从 0 改成 1 即可，软件会把该轮的速度指令取反。
 * 顺序与 motor_set_wheel_speed() 的参数一致：FL(前左), FR(前右), BL(后左), BR(后右)
 */

//A车后右轮子需翻转
#define MOTOR_INVERT_FL           0
#define MOTOR_INVERT_FR           0
#define MOTOR_INVERT_BL           0
#define MOTOR_INVERT_BR           1
/*
//B车后右轮子、后左轮子需翻转
#define MOTOR_INVERT_FL           0
#define MOTOR_INVERT_FR           0
#define MOTOR_INVERT_BL           1
#define MOTOR_INVERT_BR           1
*/
#ifdef __cplusplus
class Motor {
public:
    Motor();
    void init();
    void set_wheel_speed(int fl_speed, int fr_speed, int bl_speed, int br_speed);

    void enable();           // 使能 DRV8833（nSLEEP 拉高）
    void disable();          // 关断 DRV8833（先把 PWM 清零再拉低 nSLEEP）
    void toggle_enable();    // 翻转使能状态
    bool is_enabled() const; // 当前是否已使能

private:
    bool enabled_ = false;   // 上电默认未使能，init() 里会自动使能
};
#endif

#ifdef __cplusplus
extern "C" {
#endif

    void motor_init();
    void motor_set_wheel_speed(int fl_speed, int fr_speed, int bl_speed, int br_speed);

    void motor_enable(void);
    void motor_disable(void);
    void motor_toggle_enable(void);
    uint8_t motor_is_enabled(void);   // 1 = 已使能，0 = 已关断

#ifdef __cplusplus
}
#endif

#endif //CRTC2026_ROBOTA_MOTOR_PWM_DRIVER_H
