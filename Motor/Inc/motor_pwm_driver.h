//
// Created by MI on 2026/9/13.
//

#ifndef CRTC2026_ROBOTA_MOTOR_PWM_DRIVER_H
#define CRTC2026_ROBOTA_MOTOR_PWM_DRIVER_H

#pragma once
#include "stm32f1xx_hal.h"
#include "tim.h"

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
#define speed_scale 10


#ifdef __cplusplus
class Motor {
public:
    Motor();
    void init();
    void set_wheel_speed(int fl_speed, int fr_speed, int bl_speed, int br_speed);
};
#endif


#ifdef __cplusplus
extern "C" {
#endif
    void motor_set_wheel_speed(int fl_speed, int fr_speed, int bl_speed, int br_speed);
#ifdef __cplusplus
}
#endif

#endif //CRTC2026_ROBOTA_MOTOR_PWM_DRIVER_H