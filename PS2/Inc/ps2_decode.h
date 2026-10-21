//
// Created by MI on 2026/9/13.
//

#ifndef CRTC2026_ROBOTA_PS2_DECODE_H
#define CRTC2026_ROBOTA_PS2_DECODE_H

#pragma once
#include "ps2_driver.h"

/* ----------------------------- 按键位定义 -----------------------------
 * 驱动里已经做过取反（btn = ~data），所以 bit = 1 表示该键按下。
 *
 * 按键组 1（btn1）：
 *   bit0=SELECT   bit1=右摇杆按下  bit2=左摇杆按下  bit3=START
 *   bit4=上       bit5=右          bit6=下          bit7=左      （左侧十字键）
 *
 * 按键组 2（btn2）：
 *   bit0=L2       bit1=R2          bit2=L1          bit3=R1
 *   bit4=△(上)    bit5=○(右)       bit6=×(下)       bit7=□(左)
 *
 * 注意 UP 和 TRIANGLE 的掩码都是 0x10，但分别属于 btn1 / btn2，不要混用。
 */
#define PS2_BTN_SELECT      0x01u   // btn1
#define PS2_BTN_JOYR        0x02u   // btn1 右摇杆按下
#define PS2_BTN_JOYL        0x04u   // btn1 左摇杆按下
#define PS2_BTN_START       0x08u   // btn1
#define PS2_BTN_UP          0x10u   // btn1 左侧十字键「上」
#define PS2_BTN_RIGHT       0x20u   // btn1
#define PS2_BTN_DOWN        0x40u   // btn1
#define PS2_BTN_LEFT        0x80u   // btn1

#define PS2_BTN_L2          0x01u   // btn2
#define PS2_BTN_R2          0x02u   // btn2
#define PS2_BTN_L1          0x04u   // btn2
#define PS2_BTN_R1          0x08u   // btn2
#define PS2_BTN_TRIANGLE    0x10u   // btn2 右侧功能键「上」(△)
#define PS2_BTN_CIRCLE      0x20u   // btn2 ○
#define PS2_BTN_CROSS       0x40u   // btn2 ×
#define PS2_BTN_SQUARE      0x80u   // btn2 □

/* --------------------------- 链路判据（唯一数据源） ---------------------------
 * 合法 PS2 应答的第 2 个字节恒为 0x41 / 0x73；链路断开时 DATA 上拉读到
 * 0xFF，模式字节不是这两个值。全工程（链路状态灯、ArmTask 等）判
 * "手柄已连上"一律用这两个常量与 ps2_get_mode() 比对。 */
#define PS2_MODE_ANALOG     0x73u   // 已连上 + 模拟模式（摇杆有效）
#define PS2_MODE_DIGITAL    0x41u   // 已连上 + 数字模式（按键有效，摇杆无效）

#ifdef __cplusplus
class PS2 {
public:
    PS2();
    ~PS2();
    void read_and_decode_JoystickData();
    void get_left_analog_data(uint8_t data[2]);
    void get_right_analog_data(uint8_t data[2]);
    uint8_t get_btn1(void);   // 按键组1，bit = 1 表示按下
    uint8_t get_btn2(void);   // 按键组2，bit = 1 表示按下
    uint8_t get_mode(void);   // 手柄工作模式：0x73 = 模拟模式，0x41 = 数字模式

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
    uint8_t ps2_get_btn1(void);
    uint8_t ps2_get_btn2(void);
    uint8_t ps2_get_mode(void);

#ifdef __cplusplus
}
#endif

#endif //CRTC2026_ROBOTA_PS2_DECODE_H
