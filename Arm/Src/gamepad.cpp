//
// 手柄按键事件层实现（移植自 CRTC_RobotArm 工程 gamepad.c，C++ 化）
//

#include "gamepad.h"

/* 仅供本文件使用的内部状态 */
static uint16_t previous_buttons = 0U;
static uint16_t pending_pressed_events = 0U;
static bool is_connected = false;

void gamepad_init(void)
{
    previous_buttons = 0U;
    pending_pressed_events = 0U;
    is_connected = false;
}

void gamepad_update(uint16_t buttons_pressed, bool connected)
{
    if (!connected) {
        previous_buttons = 0U;
        pending_pressed_events = 0U;
        is_connected = false;
        return;
    }

    /*
     * 刚建立连接时，先记录当前按键。
     * 如果连接前某个键已被按住，不立即把它当成新指令。
     */
    if (!is_connected) {
        previous_buttons = buttons_pressed;
        is_connected = true;
        return;
    }

    uint16_t new_pressed =
        (uint16_t)(buttons_pressed & (uint16_t)~previous_buttons);

    /* 保留事件，等待任务控制层取走 */
    pending_pressed_events |= new_pressed;
    previous_buttons = buttons_pressed;
}

uint16_t gamepad_take_pressed_events(void)
{
    uint16_t events = pending_pressed_events;
    pending_pressed_events = 0U;
    return events;
}

bool gamepad_is_connected(void)
{
    return is_connected;
}
