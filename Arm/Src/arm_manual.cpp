//
// 机械臂手动点动实现（挖掘机式关节直控）
//
// 依赖：arm_control（读/写关节姿态）与 servo_driver（读各关节命令角域）。
// 不感知 PS2、控制模式和动作状态机——模式分流与"手动打断动作"由
// arm_task 编排层完成。
//

#include "arm_manual.h"
#include "arm_control.h"
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
    float grip_min = 0.0f, grip_max = 0.0f;
    if (!servo_get_angle_range(SERVO_BASE_Z,   &base_min,  &base_max)  ||
        !servo_get_angle_range(SERVO_LINK_1,   &link1_min, &link1_max) ||
        !servo_get_angle_range(SERVO_LINK_2,   &link2_min, &link2_max) ||
        !servo_get_angle_range(SERVO_GRIPPER,  &grip_min,  &grip_max))
    {
        return;
    }

    jog_joint(&pose.base_z_deg,   base_norm,     ARM_MANUAL_BASE_MAX_DPS,
              dt_s, base_min,  base_max);
    jog_joint(&pose.link1_deg,    shoulder_norm, ARM_MANUAL_SHOULDER_MAX_DPS,
              dt_s, link1_min, link1_max);
    jog_joint(&pose.link2_deg,    elbow_norm,    ARM_MANUAL_ELBOW_MAX_DPS,
              dt_s, link2_min, link2_max);
    jog_joint(&pose.gripper_deg,  gripper_norm,  ARM_MANUAL_GRIPPER_MAX_DPS,
              dt_s, grip_min,  grip_max);

    (void)arm_set_pose(&pose);
}

/* ================== 自动伸展脱离（卸力恢复后触发） ================== */

static float fabsf_local(float x)
{
    return x < 0.0f ? -x : x;
}

enum EscapeState : uint8_t
{
    ESCAPE_IDLE = 0,
    ESCAPE_LINKS,   // 阶段 1：大臂、小臂同时升到行程中点
    ESCAPE_BASE     // 阶段 2：底座转到行程中点
};

static EscapeState escape_state = ESCAPE_IDLE;
static uint32_t escape_last_ms = 0U;

/* 向目标角步进一步（步长封顶）；返回是否已到达 */
static bool escape_step_toward(float *angle_deg, float target_deg, float max_step)
{
    const float remain = fabsf_local(target_deg - *angle_deg);
    if (remain <= max_step)
    {
        *angle_deg = target_deg;
        return true;
    }
    *angle_deg += (*angle_deg < target_deg) ? max_step : -max_step;
    return false;
}

void arm_escape_start(uint32_t now_ms)
{
    escape_state = ESCAPE_LINKS;
    escape_last_ms = now_ms;
}

void arm_escape_stop(void)
{
    /* 保持当前位置：命令角不动，只终止序列 */
    escape_state = ESCAPE_IDLE;
}

bool arm_escape_is_running(void)
{
    return escape_state != ESCAPE_IDLE;
}

void arm_escape_update(uint32_t now_ms)
{
    if (escape_state == ESCAPE_IDLE)
    {
        return;
    }

    /* 卸力状态下不伸展（理论到不了这里：恢复力矩才会启动序列） */
    if (!arm_get_torque_enabled())
    {
        escape_state = ESCAPE_IDLE;
        return;
    }

    const uint32_t dt_ms = (uint32_t)(now_ms - escape_last_ms);
    escape_last_ms = now_ms;
    const float dt_s = (dt_ms > ARM_MANUAL_MAX_DT_MS)
                       ? (float)ARM_MANUAL_MAX_DT_MS / 1000.0f
                       : (float)dt_ms / 1000.0f;

    ArmPose_t pose;
    if (!arm_get_pose(&pose))
    {
        escape_state = ESCAPE_IDLE;
        return;
    }

    /* 速度与点动快速档相同（不受 L1 慢速档影响）：直接用各关节最大角速度 */
    if (escape_state == ESCAPE_LINKS)
    {
        /* 阶段 1：大臂、小臂同时往上伸展，两关节都到位后进入阶段 2 */
        const bool link1_done = escape_step_toward(
            &pose.link1_deg, ARM_ESCAPE_SHOULDER_DEG,
            ARM_MANUAL_SHOULDER_MAX_DPS * dt_s);
        const bool link2_done = escape_step_toward(
            &pose.link2_deg, ARM_ESCAPE_ELBOW_DEG,
            ARM_MANUAL_ELBOW_MAX_DPS * dt_s);

        if (!arm_set_pose(&pose))
        {
            escape_state = ESCAPE_IDLE;
            return;
        }
        if (link1_done && link2_done)
        {
            escape_state = ESCAPE_BASE;
        }
    }
    else if (escape_state == ESCAPE_BASE)
    {
        /* 阶段 2：底座转到行程中点，到位即结束 */
        const bool base_done = escape_step_toward(
            &pose.base_z_deg, ARM_ESCAPE_BASE_DEG,
            ARM_MANUAL_BASE_MAX_DPS * dt_s);

        if (!arm_set_pose(&pose))
        {
            escape_state = ESCAPE_IDLE;
            return;
        }
        if (base_done)
        {
            escape_state = ESCAPE_IDLE;
        }
    }
}
