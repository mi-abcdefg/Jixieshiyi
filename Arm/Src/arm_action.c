//
// 机械臂动作序列状态机实现
//
// 移植自 CRTC_RobotArm 工程 arm_action.c。保持 C 语言编译（动作表使用
// C99 指定初始化器）。相对原工程的改动：
//   - 函数名 snake_case 化（与工程导出风格一致）；
//   - 姿态读回改走 arm_get_pose()，不再直接读舵机层。
// 动作序列的角度全部是占位值，实物标定时只改下面的步骤表。
// 角度约定（与 servo_driver 配置表一致，2026/10/17 零位标定后命令角=物理角）：
//   base_z_deg = 相对 +X 逆时针方位角，180° = 臂朝 -X（车头方向）
//   link1_deg  = 肩连杆相对水平的仰角，90° = 竖直向上
//   link2_deg  = 肘相对肩连杆的夹角，域 [-120°, 60°]；-30° 对应绝对仰角 60°
// 占位期所有步骤统一为 HOME 位形（180/90/-30，即上电中位），触发动作
// 只会平稳回位不乱挥；标定时按"接近→闭合→抬起"逐步改各步骤姿态。
//

#include "arm_action.h"
#include "arm_control.h"

#include <stddef.h>

/**
 * @brief 动作中的一个关键步骤
 */
typedef struct {
    ArmPose_t pose;     // 本步骤的机械臂目标姿态
    uint32_t move_ms;   // 最短运动时间，实际时间还受速度上限约束
    uint32_t hold_ms;   // 运动结束后的保持时间
} ArmActionStep_t;

#define ARM_COMMAND_PERIOD_MS 20U
#define ARM_MAX_COMMAND_SPEED_DPS 20.0f

/* 起步及终点速度为零；曲线峰值速度为平均速度的 1.5 倍。 */
static float ArmAction_Smoothstep(float u)
{
    return u * u * (3.0f - 2.0f * u);
}

static float ArmAction_Abs(float x)
{
    return x < 0.0f ? -x : x;
}

/**
 * @brief 一套完整动作序列
 */
typedef struct {
    const ArmActionStep_t *steps;   // 动作步骤数组
    uint32_t steps_count;           // 动作包含的步骤数量
} ArmActionSequence_t;

/**
 * @brief 计算数组元素数量
 */
#define ARRAY_SIZE(array) \
    ((uint32_t)(sizeof(array) / sizeof((array)[0])))

/*
 * 抓取模式1
 *
 * 当前所有角度均为占位值。
 * 实物到手后再标定真实姿态。
 */
static const ArmActionStep_t grab_mode_1_steps[] =
{
    /* 步骤0：等待姿态 */
    {
        .pose =
        {
            .gripper_deg = 45.0f,
            .link1_deg   = 90.0f,
            .link2_deg   = -30.0f,
            .base_z_deg  = 180.0f
        },
        .move_ms = 1200U,
        .hold_ms = 500
    },

    /* 步骤1：接近物块 */
    {
        .pose =
        {
            .gripper_deg = 90.0f,
            .link1_deg   = 90.0f,
            .link2_deg   = -30.0f,
            .base_z_deg  = 180.0f
        },
        .move_ms = 1200U,
        .hold_ms = 1000
    },

    /* 步骤2：闭合夹爪 */
    {
        .pose =
        {
            .gripper_deg = 45.0f,
            .link1_deg   = 90.0f,
            .link2_deg   = -30.0f,
            .base_z_deg  = 180.0f
        },
        .move_ms = 1200U,
        .hold_ms = 500
    },

    /* 步骤3：抬起物块 */
    {
        .pose =
        {
            .gripper_deg = 90.0f,
            .link1_deg   = 90.0f,
            .link2_deg   = -30.0f,
            .base_z_deg  = 180.0f
        },
        .move_ms = 1200U,
        .hold_ms = 1000
    }
};

static const ArmActionStep_t grab_mode_2_steps[] =
{
    /* 步骤0：等待姿态 */
    {
        .pose =
        {
            .gripper_deg = 90.0f,
            .link1_deg   = 90.0f,
            .link2_deg   = -30.0f,
            .base_z_deg  = 180.0f
        },
        .move_ms = 1200U,
        .hold_ms = 500
    },

    /* 步骤1：接近物块 */
    {
        .pose =
        {
            .gripper_deg = 90.0f,
            .link1_deg   = 90.0f,
            .link2_deg   = -30.0f,
            .base_z_deg  = 180.0f
        },
        .move_ms = 1200U,
        .hold_ms = 1000
    },

    /* 步骤2：闭合夹爪 */
    {
        .pose =
        {
            .gripper_deg = 90.0f,
            .link1_deg   = 90.0f,
            .link2_deg   = -30.0f,
            .base_z_deg  = 180.0f
        },
        .move_ms = 1200U,
        .hold_ms = 500
    },

    /* 步骤3：抬起物块 */
    {
        .pose =
        {
            .gripper_deg = 90.0f,
            .link1_deg   = 90.0f,
            .link2_deg   = -30.0f,
            .base_z_deg  = 180.0f
        },
        .move_ms = 1200U,
        .hold_ms = 1000
    }
};

static const ArmActionStep_t grab_mode_3_steps[] =
{
    /* 步骤0：等待姿态 */
    {
        .pose =
        {
            .gripper_deg = 90.0f,
            .link1_deg   = 90.0f,
            .link2_deg   = -30.0f,
            .base_z_deg  = 180.0f
        },
        .move_ms = 1200U,
        .hold_ms = 500
    },

    /* 步骤1：接近物块 */
    {
        .pose =
        {
            .gripper_deg = 90.0f,
            .link1_deg   = 90.0f,
            .link2_deg   = -30.0f,
            .base_z_deg  = 180.0f
        },
        .move_ms = 1200U,
        .hold_ms = 1000
    },

    /* 步骤2：闭合夹爪 */
    {
        .pose =
        {
            .gripper_deg = 90.0f,
            .link1_deg   = 90.0f,
            .link2_deg   = -30.0f,
            .base_z_deg  = 180.0f
        },
        .move_ms = 1200U,
        .hold_ms = 500
    },

    /* 步骤3：抬起物块 */
    {
        .pose =
        {
            .gripper_deg = 90.0f,
            .link1_deg   = 90.0f,
            .link2_deg   = -30.0f,
            .base_z_deg  = 180.0f
        },
        .move_ms = 1200U,
        .hold_ms = 1000
    }
};

/*
 * 固定放置动作
 */
static const ArmActionStep_t place_steps[] =
{
    /* 步骤0：移动到放置姿态 */
    {
        .pose =
        {
            .gripper_deg = 90.0f,
            .link1_deg   = 90.0f,
            .link2_deg   = -30.0f,
            .base_z_deg  = 180.0f
        },
        .move_ms = 1200U,
        .hold_ms = 1000
    },

    /* 步骤1：打开夹爪 */
    {
        .pose =
        {
            .gripper_deg = 90.0f,
            .link1_deg   = 90.0f,
            .link2_deg   = -30.0f,
            .base_z_deg  = 180.0f
        },
        .move_ms = 1200U,
        .hold_ms = 500
    },

    /* 步骤2：返回等待姿态 */
    {
        .pose =
        {
            .gripper_deg = 90.0f,
            .link1_deg   = 90.0f,
            .link2_deg   = -30.0f,
            .base_z_deg  = 180.0f
        },
        .move_ms = 1200U,
        .hold_ms = 1000
    }
};

/**
 * @brief 所有动作的索引表
 * 指定初始化器按枚举值对号入座（C99 语法，本文件必须保持 .c 编译的原因）。
 */
static const ArmActionSequence_t action_sequences[ARM_ACTION_COUNT] = {
    [ARM_ACTION_GRAB_MODE_1] = {
        .steps = grab_mode_1_steps,
        .steps_count = ARRAY_SIZE(grab_mode_1_steps)
    },
    [ARM_ACTION_GRAB_MODE_2] = {
        .steps = grab_mode_2_steps,
        .steps_count = ARRAY_SIZE(grab_mode_2_steps)
    },
    [ARM_ACTION_GRAB_MODE_3] = {
        .steps = grab_mode_3_steps,
        .steps_count = ARRAY_SIZE(grab_mode_3_steps)
    },
    [ARM_ACTION_PLACE] = {
        .steps = place_steps,
        .steps_count = ARRAY_SIZE(place_steps)
    }
};

/*
 * 动作状态机内部变量
 */
static ArmActionState_t action_state = ARM_ACTION_STATE_IDLE;
static const ArmActionSequence_t *current_sequence = NULL;
static uint32_t current_step_index = 0U;
static uint32_t step_start_ms = 0U;
static uint32_t step_move_ms = 0U;
static uint32_t last_command_ms = 0U;
static uint32_t arrival_ms = 0U;
static bool target_sent = false;
static ArmPose_t step_start_pose;
static ArmPose_t commanded_pose;
static bool pose_valid = false;

/* 以变化量最大的轴确定同步运动所需时间，避免 smoothstep 峰值超速。 */
static uint32_t ArmAction_MotionTime(const ArmPose_t *from,
                                     const ArmPose_t *to, uint32_t requested_ms)
{
    float maximum = ArmAction_Abs(to->gripper_deg - from->gripper_deg);
    float d = ArmAction_Abs(to->link1_deg - from->link1_deg);
    if (d > maximum) maximum = d;
    d = ArmAction_Abs(to->link2_deg - from->link2_deg);
    if (d > maximum) maximum = d;
    d = ArmAction_Abs(to->base_z_deg - from->base_z_deg);
    if (d > maximum) maximum = d;

    /* +0.999f 等价于向上取整，不需要 libm 的 ceilf。 */
    const uint32_t limited_ms = (uint32_t)(
        1500.0f * maximum / ARM_MAX_COMMAND_SPEED_DPS + 0.999f);
    return requested_ms > limited_ms ? requested_ms : limited_ms;
}

static ArmPose_t ArmAction_Interpolate(const ArmPose_t *from,
                                       const ArmPose_t *to, float s)
{
    ArmPose_t p;
    p.gripper_deg = from->gripper_deg +
                    s * (to->gripper_deg - from->gripper_deg);
    p.link1_deg = from->link1_deg +
                  s * (to->link1_deg - from->link1_deg);
    p.link2_deg = from->link2_deg +
                  s * (to->link2_deg - from->link2_deg);
    p.base_z_deg = from->base_z_deg +
                   s * (to->base_z_deg - from->base_z_deg);
    return p;
}

static bool ArmAction_BeginStep(uint32_t now_ms)
{
    if (current_sequence == NULL ||
        current_step_index >= current_sequence->steps_count || !pose_valid)
    {
        return false;
    }
    step_start_pose = commanded_pose;
    step_start_ms = now_ms;
    last_command_ms = now_ms;
    target_sent = false;
    const ArmActionStep_t *step = &current_sequence->steps[current_step_index];
    step_move_ms = ArmAction_MotionTime(&step_start_pose, &step->pose,
                                        step->move_ms);
    return true;
}

void arm_action_init(void)
{
    action_state = ARM_ACTION_STATE_IDLE;
    current_sequence = NULL;
    current_step_index = 0U;
    step_start_ms = 0U;
    step_move_ms = 0U;
    last_command_ms = 0U;
    arrival_ms = 0U;
    target_sent = false;
    pose_valid = arm_get_pose(&commanded_pose);
}

bool arm_action_start(ArmActionId_t action_id, uint32_t now_ms)
{
    if ((uint32_t)action_id >= (uint32_t)ARM_ACTION_COUNT ||
        action_state == ARM_ACTION_STATE_RUNNING ||
        action_state == ARM_ACTION_STATE_ERROR)
    {
        return false;
    }

    /* 读回最近一次指令，以便取消后新动作从当前指令位置接续。 */
    pose_valid = arm_get_pose(&commanded_pose);
    if (!pose_valid)
    {
        action_state = ARM_ACTION_STATE_ERROR;
        return false;
    }
    current_sequence = &action_sequences[action_id];
    current_step_index = 0U;
    action_state = ARM_ACTION_STATE_RUNNING;
    if (!ArmAction_BeginStep(now_ms))
    {
        current_sequence = NULL;
        action_state = ARM_ACTION_STATE_ERROR;
        return false;
    }
    return true;
}

void arm_action_update(uint32_t now_ms)
{
    if (action_state != ARM_ACTION_STATE_RUNNING) return;
    if (current_sequence == NULL ||
        current_step_index >= current_sequence->steps_count)
    {
        action_state = ARM_ACTION_STATE_ERROR;
        current_sequence = NULL;
        return;
    }

    const ArmActionStep_t *step = &current_sequence->steps[current_step_index];
    const uint32_t elapsed_ms = (uint32_t)(now_ms - step_start_ms);
    if (elapsed_ms < step_move_ms)
    {
        if ((uint32_t)(now_ms - last_command_ms) < ARM_COMMAND_PERIOD_MS)
            return;
        float u = (float)elapsed_ms / (float)step_move_ms;
        ArmPose_t interpolated = ArmAction_Interpolate(
            &step_start_pose, &step->pose, ArmAction_Smoothstep(u));
        if (!arm_set_pose(&interpolated))
        {
            action_state = ARM_ACTION_STATE_ERROR;
            current_sequence = NULL;
            return;
        }
        commanded_pose = interpolated;
        last_command_ms = now_ms;
        return;
    }

    /* 运动结束后显式输出精确终点，再开始计算 hold_ms。 */
    if (!target_sent)
    {
        if (!arm_set_pose(&step->pose))
        {
            action_state = ARM_ACTION_STATE_ERROR;
            current_sequence = NULL;
            return;
        }
        commanded_pose = step->pose;
        last_command_ms = now_ms;
        arrival_ms = now_ms;
        target_sent = true;
    }
    if ((uint32_t)(now_ms - arrival_ms) < step->hold_ms) return;

    ++current_step_index;
    if (current_step_index >= current_sequence->steps_count)
    {
        current_sequence = NULL;
        action_state = ARM_ACTION_STATE_FINISHED;
        return;
    }
    if (!ArmAction_BeginStep(now_ms))
    {
        current_sequence = NULL;
        action_state = ARM_ACTION_STATE_ERROR;
    }
}

void arm_action_cancel(void)
{
    current_sequence = NULL;
    current_step_index = 0U;
    step_move_ms = 0U;
    target_sent = false;
    /* 保持最近一次指令及 PWM；没有位置反馈，舵机可能仍在追赶。 */
    action_state = ARM_ACTION_STATE_IDLE;
}

ArmActionState_t arm_action_get_state(void)
{
    return action_state;
}
