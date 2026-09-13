//
// Created by MI on 2026/9/13.
//

#include "ps2_decode.h"

PS2::PS2() {
    this -> JoystickData = new JOYSTICK_TypeDef;
}

PS2::~PS2() {
    if (this->JoystickData) {
        delete this->JoystickData;
        this->JoystickData = nullptr;
    }
}

void PS2::get_and_decode_JoystickData() {
    PS2_ScanKey(this->JoystickData);
    //死区控制
    if (this->JoystickData->RJoy_LR < 10 && this->JoystickData->RJoy_LR > -10) {
        this->JoystickData->RJoy_LR = 0;
    }
    if (this->JoystickData->RJoy_UD < 10 && this->JoystickData->RJoy_UD > -10) {
        this->JoystickData->RJoy_UD = 0;
    }
    if (this->JoystickData->LJoy_LR < 10 && this->JoystickData->LJoy_LR > -10) {
        this->JoystickData->LJoy_LR = 0;
    }
    if (this->JoystickData->LJoy_UD < 10 && this->JoystickData->LJoy_UD > -10) {
        this->JoystickData->LJoy_UD = 0;
    }
}