//
// 操作对象模式实现（唯一的状态就这一个变量）
//

#include "control_mode.h"

static ControlMode_t current_mode = CONTROL_MODE_CHASSIS;   // 上电默认底盘

void control_mode_toggle(void)
{
    current_mode = (current_mode == CONTROL_MODE_CHASSIS)
                   ? CONTROL_MODE_ARM
                   : CONTROL_MODE_CHASSIS;
}

ControlMode_t control_mode_get(void)
{
    return current_mode;
}
