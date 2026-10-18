//
// 机械臂关节空间控制实现（移植自 CRTC_RobotArm 工程 arm_control.c，C++ 化）
//

#include "arm_control.h"
#include "servo_driver.h"

#include <stddef.h>

bool arm_set_pose(const ArmPose_t *pose)
{
    /* 检查传入的姿态指针是否有效 */
    if (pose == NULL)
    {
        return false;
    }

    bool result = true;

    /* 舵机1：夹爪 */
    if (!servo_set_angle(SERVO_GRIPPER, pose->gripper_deg))
    {
        result = false;
    }

    /* 舵机2：第一段连杆 */
    if (!servo_set_angle(SERVO_LINK_1, pose->link1_deg))
    {
        result = false;
    }

    /* 舵机3：第二段连杆 */
    if (!servo_set_angle(SERVO_LINK_2, pose->link2_deg))
    {
        result = false;
    }

    /* 舵机4：底座旋转 */
    if (!servo_set_angle(SERVO_BASE_Z, pose->base_z_deg))
    {
        result = false;
    }

    return result;
}

bool arm_set_gripper(float angle_deg)
{
    ArmPose_t pose;
    if (!arm_get_pose(&pose))
    {
        return false;
    }
    pose.gripper_deg = angle_deg;
    return arm_set_pose(&pose);
}

bool arm_get_pose(ArmPose_t *pose)
{
    if (pose == NULL)
    {
        return false;
    }

    /* 四个关节都下发过目标才算有效，避免上层拿半套姿态做轨迹起点。 */
    return servo_get_commanded_angle(SERVO_GRIPPER, &pose->gripper_deg) &&
           servo_get_commanded_angle(SERVO_LINK_1,  &pose->link1_deg)  &&
           servo_get_commanded_angle(SERVO_LINK_2,  &pose->link2_deg)  &&
           servo_get_commanded_angle(SERVO_BASE_Z,  &pose->base_z_deg);
}
