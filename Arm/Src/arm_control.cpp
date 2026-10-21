//
// 机械臂关节空间控制实现（移植自 CRTC_RobotArm 工程 arm_control.c，C++ 化）
//
// 四个舵机都从这里下发：三关节 + 夹爪。夹爪的角度由 gripper 模块决定，
// 但整臂姿态里带夹爪字段，方便"一次读写整套姿态"。
//

#include "arm_control.h"
#include "servo_driver.h"

#include <stddef.h>

/* 用户层面的卸力模式（L2 按过之后为 true）。与"个别关节此刻是否有脉冲"
 * 分开记录：脱离动作期间模式已经是"有力"，但有些关节还没给力。 */
static bool relaxed = false;

bool arm_set_pose_masked(const ArmPose_t *pose, uint32_t joint_mask)
{
    /* 检查传入的姿态指针是否有效 */
    if (pose == NULL)
    {
        return false;
    }

    bool result = true;

    /* 舵机1：夹爪 */
    if ((joint_mask & ARM_JOINT_GRIPPER) != 0U)
    {
        if (!servo_set_angle(SERVO_GRIPPER, pose->gripper_deg))
        {
            result = false;
        }
    }

    /* 舵机2：第一段连杆 */
    if ((joint_mask & ARM_JOINT_SHOULDER) != 0U)
    {
        if (!servo_set_angle(SERVO_LINK_1, pose->link1_deg))
        {
            result = false;
        }
    }

    /* 舵机3：第二段连杆 */
    if ((joint_mask & ARM_JOINT_ELBOW) != 0U)
    {
        if (!servo_set_angle(SERVO_LINK_2, pose->link2_deg))
        {
            result = false;
        }
    }

    /* 舵机4：底座旋转 */
    if ((joint_mask & ARM_JOINT_BASE_Z) != 0U)
    {
        if (!servo_set_angle(SERVO_BASE_Z, pose->base_z_deg))
        {
            result = false;
        }
    }

    return result;
}

bool arm_set_pose(const ArmPose_t *pose)
{
    return arm_set_pose_masked(pose, ARM_JOINT_ALL);
}

float arm_clamp_angle(ServoId_t servo, float cmd_deg)
{
    float min_deg = 0.0f;
    float max_deg = 0.0f;
    if (!servo_get_angle_range(servo, &min_deg, &max_deg))
    {
        return cmd_deg;
    }
    if (cmd_deg < min_deg)
    {
        return min_deg;
    }
    if (cmd_deg > max_deg)
    {
        return max_deg;
    }
    return cmd_deg;
}

void arm_set_joint_torque(uint32_t joint_mask, bool enabled)
{
    if ((joint_mask & ARM_JOINT_GRIPPER) != 0U)
    {
        (void)servo_set_torque_one(SERVO_GRIPPER, enabled);
    }
    if ((joint_mask & ARM_JOINT_SHOULDER) != 0U)
    {
        (void)servo_set_torque_one(SERVO_LINK_1, enabled);
    }
    if ((joint_mask & ARM_JOINT_ELBOW) != 0U)
    {
        (void)servo_set_torque_one(SERVO_LINK_2, enabled);
    }
    if ((joint_mask & ARM_JOINT_BASE_Z) != 0U)
    {
        (void)servo_set_torque_one(SERVO_BASE_Z, enabled);
    }
}

uint32_t arm_get_joint_torque_mask(void)
{
    uint32_t mask = 0U;

    if (servo_is_torque_one(SERVO_GRIPPER))
    {
        mask |= ARM_JOINT_GRIPPER;
    }
    if (servo_is_torque_one(SERVO_LINK_1))
    {
        mask |= ARM_JOINT_SHOULDER;
    }
    if (servo_is_torque_one(SERVO_LINK_2))
    {
        mask |= ARM_JOINT_ELBOW;
    }
    if (servo_is_torque_one(SERVO_BASE_Z))
    {
        mask |= ARM_JOINT_BASE_Z;
    }

    return mask;
}

void arm_set_torque_enabled(bool enabled)
{
    servo_set_torque_enabled(enabled);
    relaxed = !enabled;
}

bool arm_get_torque_enabled(void)
{
    return !relaxed;
}

bool arm_is_relaxed(void)
{
    return relaxed;
}

void arm_set_relaxed(bool is_relaxed)
{
    if (is_relaxed)
    {
        /* 卸力：四路一起软下来（CCR 清零） */
        arm_set_torque_enabled(false);
    }
    else
    {
        /* 只退出模式：不给任何一路力，脉宽由随后的"脱离"动作逐路恢复 */
        relaxed = false;
    }
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
