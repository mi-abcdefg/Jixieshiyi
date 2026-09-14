//
// Created by MI on 2026/9/13.
//

#ifndef CRTC2026_ROBOTA_PS2_DECODE_H
#define CRTC2026_ROBOTA_PS2_DECODE_H

#pragma once
#include "ps2_driver.h"

#ifdef __cplusplus
class PS2 {
public:
    PS2();
    ~PS2();
    void read_and_decode_JoystickData();
    void get_left_analog_data(uint8_t data[2]);
    void get_right_analog_data(uint8_t data[2]);

private:
    JOYSTICK_TypeDef *JoystickData;
};
#endif

#ifdef __cplusplus
extern "C" {
#endif

    void ps2_read_and_decode_JoystickData();
    void ps2_get_left_analog_data(uint8_t data[2]);
    void ps2_get_right_analog_data(uint8_t data[2]);

#ifdef __cplusplus
}
#endif

#endif //CRTC2026_ROBOTA_PS2_DECODE_H