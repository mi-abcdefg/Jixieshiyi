//
// Created by MI on 2026/9/13.
//

#include "motor_pwm_driver.h"

Motor::Motor() {

}

void Motor::init() {
    HAL_TIM_PWM_Start(TIM_FL_1, CH_FL_1);
    HAL_TIM_PWM_Start(TIM_FL_2, CH_FL_2);
    HAL_TIM_PWM_Start(TIM_FR_1, CH_FR_1);
    HAL_TIM_PWM_Start(TIM_FR_2, CH_FR_2);
    HAL_TIM_PWM_Start(TIM_BL_1, CH_BL_1);
    HAL_TIM_PWM_Start(TIM_BL_2, CH_BL_2);
    HAL_TIM_PWM_Start(TIM_BR_1, CH_BR_1);
    HAL_TIM_PWM_Start(TIM_BR_2, CH_BR_2);
}

int compare_clamp(int compare) {
    if (compare > 1000) {
        compare = 1000;
    }
    return compare;
}

void motor_cmd(TIM_HandleTypeDef* htim, uint32_t channel1, uint32_t channel2, int speed) {
    if (speed == 0) {
        __HAL_TIM_SET_COMPARE(htim, channel1, 0);
        __HAL_TIM_SET_COMPARE(htim, channel2, 0);
    }
    else if (speed > 0) {
        __HAL_TIM_SET_COMPARE(htim, channel1, compare_clamp(speed * speed_scale));
        __HAL_TIM_SET_COMPARE(htim, channel2, 0);
    }
    else {
        __HAL_TIM_SET_COMPARE(htim, channel1, 0);
        __HAL_TIM_SET_COMPARE(htim, channel2, compare_clamp(-1 * speed * speed_scale));
    }
}

void Motor::set_wheel_speed(int fl_speed, int fr_speed, int bl_speed, int br_speed) {
    motor_cmd(TIM_FL_1, CH_FL_1, CH_FL_2, fl_speed);
    motor_cmd(TIM_FR_1, CH_FR_1, CH_FR_2, fr_speed);
    motor_cmd(TIM_BL_1, CH_BL_1, CH_BL_2, bl_speed);
    motor_cmd(TIM_BR_1, CH_BR_1, CH_BR_2, br_speed);
}

extern "C" {
    Motor motor1;
    void motor_set_wheel_speed(int fl_speed, int fr_speed, int bl_speed, int br_speed) {
        motor1.set_wheel_speed(fl_speed, fr_speed, bl_speed, br_speed);
    }
}