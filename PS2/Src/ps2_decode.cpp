//
// Created by MI on 2026/9/13.
//

#include "ps2_decode.h"

PS2::PS2() {
    // 注意必须带括号：new JOYSTICK_TypeDef 是"默认初始化"，对 POD 结构体
    // 不会清零，第一次扫描成功前 mode/摇杆字段全是随机值（会让状态灯误判、上电乱动）。
    // 加 () 变成值初始化，全字段清零。
    this -> JoystickData = new JOYSTICK_TypeDef();
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

    /* 摇杆数据有效性检查。
     * 只有模拟模式下第 5~8 字节才会随摇杆变化；如果手柄还在数字模式（或数据无效），
     * 这 4 个字节会固定成全 0x00 或全 0xFF。如果不处理，它们会被当成"摇杆推到底"，
     * 解算出一个恒定的满速指令——现象就是"上电电机就自己转、推摇杆反而没反应"。
     * 这里把这种情况按中位处理，等 PS2_ConfigAnalogMode() 切换成功或按 MODE 键后自然恢复。 */
    const bool analog_all_low  = (temp_js_data.LJoy_LR == 0x00) && (temp_js_data.LJoy_UD == 0x00)
                              && (temp_js_data.RJoy_LR == 0x00) && (temp_js_data.RJoy_UD == 0x00);
    const bool analog_all_high = (temp_js_data.LJoy_LR == 0xFF) && (temp_js_data.LJoy_UD == 0xFF)
                              && (temp_js_data.RJoy_LR == 0xFF) && (temp_js_data.RJoy_UD == 0xFF);
    if (analog_all_low || analog_all_high) {
        temp_js_data.LJoy_LR = 128;
        temp_js_data.LJoy_UD = 128;
        temp_js_data.RJoy_LR = 128;
        temp_js_data.RJoy_UD = 128;
    }

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

/*
 * 获取按键组1（bit = 1 表示按下）
 *   bit0=SELECT  bit1=右摇杆按下  bit2=左摇杆按下  bit3=START
 *   bit4=上      bit5=右          bit6=下          bit7=左     （左侧十字键）
 */
uint8_t PS2::get_btn1(void) {
    return this->JoystickData->btn1;
}

/*
 * 获取按键组2（bit = 1 表示按下）
 *   bit0=L2      bit1=R2          bit2=L1          bit3=R1
 *   bit4=△       bit5=○           bit6=×           bit7=□
 */
uint8_t PS2::get_btn2(void) {
    return this->JoystickData->btn2;
}

/*
 * 获取手柄工作模式：0x73 = 模拟模式（摇杆有效），0x41 = 数字模式（摇杆无效）
 */
uint8_t PS2::get_mode(void) {
    return this->JoystickData->mode;
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

    uint8_t ps2_get_btn1(void) {
        return ps2_instance.get_btn1();
    }

    uint8_t ps2_get_btn2(void) {
        return ps2_instance.get_btn2();
    }

    uint8_t ps2_get_mode(void) {
        return ps2_instance.get_mode();
    }

}