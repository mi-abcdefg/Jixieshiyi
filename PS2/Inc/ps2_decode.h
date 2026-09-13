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
    void get_and_decode_JoystickData();
private:
    JOYSTICK_TypeDef *JoystickData;
};
#endif


#endif //CRTC2026_ROBOTA_PS2_DECODE_H