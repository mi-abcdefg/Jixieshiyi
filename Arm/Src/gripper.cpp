//
// 夹爪独立模块实现
//

#include "gripper.h"
#include "servo_driver.h"

static GripperState_t gripper_state = GRIPPER_UNDEF;

void gripper_init(void)
{
    gripper_state = GRIPPER_UNDEF;
}

bool gripper_set(GripperState_t state)
{
    bool ok = false;

    switch (state) {
    case GRIPPER_OPEN:
        ok = servo_set_angle(SERVO_GRIPPER, GRIPPER_OPEN_DEG);
        break;
    case GRIPPER_CLOSE:
        ok = servo_set_angle(SERVO_GRIPPER, GRIPPER_CLOSE_DEG);
        break;
    default:
        return false;   // 未定义不是一个可下发的目标
    }

    if (ok)
    {
        gripper_state = state;
    }
    return ok;
}

bool gripper_toggle(void)
{
    float current = 0.0f;
    if (!servo_get_commanded_angle(SERVO_GRIPPER, &current))
    {
        return false;
    }

    /* 以两档中点为界：当前偏闭合档则张开，反之闭合 */
    const float midpoint = 0.5f * (GRIPPER_OPEN_DEG + GRIPPER_CLOSE_DEG);
    const GripperState_t target =
        (current >= midpoint) ? GRIPPER_OPEN : GRIPPER_CLOSE;
    return gripper_set(target);
}

GripperState_t gripper_get_state(void)
{
    return gripper_state;
}

void gripper_jog(float rate_norm, float dt_s)
{
    if (rate_norm == 0.0f)
    {
        return;
    }

    float angle = 0.0f;
    if (!servo_get_commanded_angle(SERVO_GRIPPER, &angle))
    {
        return;
    }

    float min_deg = 0.0f;
    float max_deg = 0.0f;
    if (!servo_get_angle_range(SERVO_GRIPPER, &min_deg, &max_deg))
    {
        return;
    }

    angle += rate_norm * GRIPPER_MAX_DPS * dt_s;
    if (angle > max_deg)
    {
        angle = max_deg;
    }
    else if (angle < min_deg)
    {
        angle = min_deg;
    }

    if (servo_set_angle(SERVO_GRIPPER, angle))
    {
        gripper_state = GRIPPER_UNDEF;   // 离开命名位置，状态不再可描述
    }
}
