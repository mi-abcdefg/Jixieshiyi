//
// 夹爪独立模块实现（本分支仅保留点动）
//

#include "gripper.h"
#include "servo_driver.h"

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

    (void)servo_set_angle(SERVO_GRIPPER, angle);
}
