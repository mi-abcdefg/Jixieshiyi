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

void PS2::read_and_decode_JoystickData() {
    JOYSTICK_TypeDef temp_js_data;
    PS2_ScanKey(&temp_js_data);

    //死区控制
    if (temp_js_data.RJoy_LR < 138 && temp_js_data.RJoy_LR > 118) {
        temp_js_data.RJoy_LR = 128;
    }
    if (temp_js_data.RJoy_UD < 138 && temp_js_data.RJoy_UD > 118) {
        temp_js_data.RJoy_UD = 128;
    }
    if (temp_js_data.LJoy_LR < 138 && temp_js_data.LJoy_LR > 118) {
        temp_js_data.LJoy_LR = 128;
    }
    if (temp_js_data.LJoy_UD < 138 && temp_js_data.LJoy_UD > 118) {
        temp_js_data.LJoy_UD = 128;
    }

    // 将数据存储到类成员中
    *(this->JoystickData) = temp_js_data;
}

/*
 * 获取左摇杆数据
 * data[0] = 左摇杆左右值
 * data[1] = 左摇杆上下值
 */
void PS2::get_left_analog_data(uint8_t data[2]) {
    data[0] = this->JoystickData->LJoy_LR;
    data[1] = this->JoystickData->LJoy_UD;
}

/*
 * 获取右摇杆数据
 * data[0] = 右摇杆左右值
 * data[1] = 右摇杆上下值
 */
void PS2::get_right_analog_data(uint8_t data[2]) {
    data[0] = this->JoystickData->RJoy_LR;
    data[1] = this->JoystickData->RJoy_UD;
}

extern "C" {

    PS2 ps2_instance;
    void ps2_read_and_decode_JoystickData() {
        ps2_instance.read_and_decode_JoystickData();
    }

    void ps2_get_left_analog_data(uint8_t data[2]) {
        ps2_instance.get_left_analog_data(data);
    }

    void ps2_get_right_analog_data(uint8_t data[2]) {
        ps2_instance.get_right_analog_data(data);
    }

}