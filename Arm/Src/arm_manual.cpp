//
// 机械臂手动点动实现（挖掘机式关节直控）
//
// 依赖：arm_control（读/写关节姿态）、servo_driver（读各关节命令角域）、
// gripper（夹爪以独立模块管理，点动经 gripper_jog）。
// 不感知 PS2 与控制模式——模式分流由 arm_task 编排层完成。
// 本文件的各关节角速度宏（ARM_MANUAL_*_MAX_DPS）同时是动作引擎的速度
// 数据源（arm_action 直接引用），改速度只需改这里一处。
//

#include "arm_manual.h"
#include "arm_control.h"
#include "gripper.h"
#include "servo_driver.h"

/* 内部状态 */
static uint32_t last_update_ms = 0U;
static bool has_last_update = false;
static uint8_t speed_level = 0U;   // 0=快速 1=慢速；上电默认快速

void arm_manual_init(void)
{
    has_last_update = false;
    speed_level = 0U;
}

void arm_manual_cycle_speed_level(void)
{
    speed_level = static_cast<uint8_t>((speed_level + 1U) % 2U);
}

/* 单关节步进：按杆量积分一个周期，并夹到该关节的命令角域内 */
static void jog_joint(float *angle_deg, float norm, float max_dps,
                      float dt_s, float min_deg, float max_deg)
{
    /* 死区：该轴松杆 = 关节保持 */
    if (norm > -ARM_MANUAL_DEADZONE && norm < ARM_MANUAL_DEADZONE)
    {
        return;
    }

    /* 慢速档整体降速 */
    const float speed_scale = (speed_level == 1U) ? ARM_MANUAL_SLOW_DPS_SCALE : 1.0f;

    *angle_deg += norm * max_dps * speed_scale * dt_s;
    if (*angle_deg > max_deg)
    {
        *angle_deg = max_deg;
    }
    else if (*angle_deg < min_deg)
    {
        *angle_deg = min_deg;
    }
}

void arm_manual_update(uint32_t now_ms,
                       float base_norm, float shoulder_norm,
                       float elbow_norm, float gripper_norm)
{
    /* 首次调用只记时间不下发（此时还没有可靠的 dt） */
    if (!has_last_update)
    {
        last_update_ms = now_ms;
        has_last_update = true;
        return;
    }
    const uint32_t dt_ms = (uint32_t)(now_ms - last_update_ms);
    last_update_ms = now_ms;

    /* 四轴全松杆：不下发任何指令（保持现状），省去无谓的舵机写 */
    const bool any_input =
        !(base_norm > -ARM_MANUAL_DEADZONE && base_norm < ARM_MANUAL_DEADZONE) ||
        !(shoulder_norm > -ARM_MANUAL_DEADZONE && shoulder_norm < ARM_MANUAL_DEADZONE) ||
        !(elbow_norm > -ARM_MANUAL_DEADZONE && elbow_norm < ARM_MANUAL_DEADZONE) ||
        !(gripper_norm > -ARM_MANUAL_DEADZONE && gripper_norm < ARM_MANUAL_DEADZONE);
    if (!any_input)
    {
        return;
    }

    /* dt 上限保护：长时间阻塞（断点/调度抖动）后不做大步积分 */
    const float dt_s = (dt_ms > ARM_MANUAL_MAX_DT_MS)
                       ? (float)ARM_MANUAL_MAX_DT_MS / 1000.0f
                       : (float)dt_ms / 1000.0f;

    /* 以当前指令角为起点逐关节步进；量程以配置表为唯一数据源 */
    ArmPose_t pose;
    if (!arm_get_pose(&pose))
    {
        return;
    }

    float base_min = 0.0f, base_max = 0.0f;
    float link1_min = 0.0f, link1_max = 0.0f;
    float link2_min = 0.0f, link2_max = 0.0f;
    if (!servo_get_angle_range(SERVO_BASE_Z,   &base_min,  &base_max)  ||
        !servo_get_angle_range(SERVO_LINK_1,   &link1_min, &link1_max) ||
        !servo_get_angle_range(SERVO_LINK_2,   &link2_min, &link2_max))
    {
        return;
    }

    jog_joint(&pose.base_z_deg,   base_norm,     ARM_MANUAL_BASE_MAX_DPS,
              dt_s, base_min,  base_max);
    jog_joint(&pose.link1_deg,    shoulder_norm, ARM_MANUAL_SHOULDER_MAX_DPS,
              dt_s, link1_min, link1_max);
    jog_joint(&pose.link2_deg,    elbow_norm,    ARM_MANUAL_ELBOW_MAX_DPS,
              dt_s, link2_min, link2_max);
    (void)arm_set_pose(&pose);

    /* 夹爪走独立模块：点动会使其状态转为"未定义" */
    if (gripper_norm <= -ARM_MANUAL_DEADZONE || gripper_norm >= ARM_MANUAL_DEADZONE)
    {
        gripper_jog(gripper_norm, dt_s);
    }
}

