//
// 机械臂动作引擎实现（条件路径 + 关键帧编号序列）
//
// 动作被定义为"命名关键帧编号的序列"（帧数据在 arm_keyframe 表中）：
//   - 固定路径动作：依次推进所有帧；
//   - 区域路径动作：启动时按"篮内/篮外"判定选择篮内路径或篮外路径
//     （底座用户角 ≥160° 且肩用户角 ≤100° 判为篮内，阈值见宏注释）。
// 推进细节：
//   - 关键帧字段先换算为命令角目标（ARM_KF_KEEP 字段取当前命令角，
//     即该关节保持不动），三关节以各自快速档角速度步进逼近；
//   - 三关节全部到位后，若帧内夹爪状态非"未定义"则切换夹爪，
//     随后停留 ARM_ACTION_HOLD_MS 进入下一帧。
//

#include "arm_action.h"
#include "arm_keyframe.h"
#include "gripper.h"
#include "arm_manual.h"    // 各关节快速档角速度宏（与点动同一数据源）
#include "arm_control.h"
#include "servo_driver.h"
#include <stddef.h>

/* ------------------------------ 引擎参数 ------------------------------ */
#define ARM_ACTION_HOLD_MS          300U    // 关键帧到位后的停留时间
#define ARM_ACTION_TICK_S           0.020f  // 引擎推进周期 [s]，须与 ArmTask 周期一致

/* ------------------------- 篮内区判定（暂定值） -------------------------
 * 机械臂在机器人右侧工作，受舵机 180° 行程限制，需能转往背后篮子方向。
 * 判定（2026/10/19 约定，暂定值待实车复核）：
 *   底座用户角 ≥ 160° 且 肩用户角 ≤ 100°  →  机械臂被围在篮子里；
 *   其余姿态统称篮子外侧。 */
#define ARM_IN_BASKET_BASE_USER_MIN      160.0f
#define ARM_IN_BASKET_SHOULDER_USER_MAX  100.0f

/* ------------------- 动作路径（关键帧编号的序列） ------------------- */

/* 回收缩：篮内经篮上高点、篮外经外侧高点，都到最小收缩位 */
static const ArmKeyframeId_t retract_in[]  = { ARM_KF_ID_BASKET_HIGH, ARM_KF_ID_RETRACT };
static const ArmKeyframeId_t retract_out[] = { ARM_KF_ID_OUTER_HIGH,  ARM_KF_ID_RETRACT };

/* 脱离伸展：大臂小臂升到中点 → 底座转到中点 */
static const ArmKeyframeId_t escape_path[] = { ARM_KF_ID_ESCAPE_LINKS, ARM_KF_ID_ESCAPE_BASE };

/* 固定放置 / 抓取（位形占位，均为 HOME 关键帧变体） */
static const ArmKeyframeId_t place_path[] = { ARM_KF_ID_HOME, ARM_KF_ID_HOME_OPEN, ARM_KF_ID_HOME };
static const ArmKeyframeId_t grab1_path[] = { ARM_KF_ID_HOME, ARM_KF_ID_HOME_CLOSE, ARM_KF_ID_HOME };
static const ArmKeyframeId_t grab3_path[] = { ARM_KF_ID_HOME, ARM_KF_ID_HOME_CLOSE, ARM_KF_ID_HOME };

/* 篮内外转移：
 *   篮外 → 篮内：外侧高点 → 篮上高点 → 篮内点
 *   篮内 → 篮外：篮上高点 → 外侧高点 → 拾取低点 */
static const ArmKeyframeId_t transit_in[]  = { ARM_KF_ID_BASKET_HIGH, ARM_KF_ID_OUTER_HIGH, ARM_KF_ID_PICKUP_LOW };
static const ArmKeyframeId_t transit_out[] = { ARM_KF_ID_OUTER_HIGH,  ARM_KF_ID_BASKET_HIGH, ARM_KF_ID_BASKET_IN };

/**
 * @brief 动作路径定义
 * @note  path_in 为 NULL 表示固定路径动作（用 fixed）；
 *        非 NULL 表示区域路径动作（按篮内/篮外二选一，两条路径须等长）
 */
typedef struct {
    const ArmKeyframeId_t *fixed;
    uint8_t fixed_count;
    const ArmKeyframeId_t *path_in;
    const ArmKeyframeId_t *path_out;
    uint8_t zone_count;
} ArmActionSeq_t;

static const ArmActionSeq_t sequences[ARM_ACTION_COUNT] = {
    [ARM_ACTION_RETRACT]     = { NULL,         0U, retract_in,  retract_out,  2U },
    [ARM_ACTION_ESCAPE]      = { escape_path,  2U, NULL,        NULL,         0U },
    [ARM_ACTION_TRANSIT]     = { NULL,         0U, transit_in,  transit_out,  3U },
    [ARM_ACTION_PLACE]       = { place_path,   3U, NULL,        NULL,         0U },
    [ARM_ACTION_GRAB_MODE_1] = { grab1_path,   3U, NULL,        NULL,         0U },
    [ARM_ACTION_GRAB_MODE_3] = { grab3_path,   3U, NULL,        NULL,         0U },
};

/* --------------------------- 状态机内部变量 --------------------------- */

static ArmActionState_t action_state = ARM_ACTION_STATE_IDLE;
static const ArmKeyframeId_t *current_path = NULL;
static uint8_t current_len = 0U;
static int16_t step_index = 0;
static bool phase_hold = false;        // true = 已到位，正在停留
static uint32_t phase_start_ms = 0U;
static float tgt_base = 0.0f, tgt_link1 = 0.0f, tgt_link2 = 0.0f;
static bool grip_done = false;         // 当前帧夹爪是否已切换

/* 篮内区判定：底座用户角 ≥ 160° 且 肩用户角 ≤ 100°（暂定值，见宏注释） */
static bool in_basket_zone(void)
{
    float base_cmd = 0.0f, link1_cmd = 0.0f;
    if (!servo_get_commanded_angle(SERVO_BASE_Z, &base_cmd) ||
        !servo_get_commanded_angle(SERVO_LINK_1, &link1_cmd))
    {
        return false;
    }
    const float base_user     = ARM_UA_BASE_FROM_CMD(base_cmd);
    const float shoulder_user = ARM_UA_LINK1_FROM_CMD(link1_cmd);
    return (base_user >= ARM_IN_BASKET_BASE_USER_MIN) &&
           (shoulder_user <= ARM_IN_BASKET_SHOULDER_USER_MAX);
}

/* 开始逼近一个关键帧：换算目标（KEEP 字段保持当前命令角） */
static void begin_keyframe(const ArmKeyframe_t *kf)
{
    float base_cmd = 0.0f, link1_cmd = 0.0f, link2_cmd = 0.0f;
    (void)servo_get_commanded_angle(SERVO_BASE_Z, &base_cmd);
    (void)servo_get_commanded_angle(SERVO_LINK_1, &link1_cmd);
    (void)servo_get_commanded_angle(SERVO_LINK_2, &link2_cmd);

    tgt_base  = (kf->base_deg  == ARM_KF_KEEP) ? base_cmd
                                               : ARM_UA_BASE_TO_CMD(kf->base_deg);
    tgt_link1 = (kf->link1_deg == ARM_KF_KEEP) ? link1_cmd
                                               : ARM_UA_LINK1_TO_CMD(kf->link1_deg);
    tgt_link2 = (kf->link2_deg == ARM_KF_KEEP) ? link2_cmd
                                               : ARM_UA_LINK2_TO_CMD(kf->link2_deg);
    grip_done = (kf->gripper == GRIPPER_UNDEF);   // 未定义 = 无夹爪动作
    phase_hold = false;
}

/* 单关节朝目标步进；返回是否已到位 */
static bool step_joint(float *angle_deg, float target, float max_dps, float dt_s)
{
    const float remain = target - *angle_deg;
    const float step = max_dps * dt_s;
    if (remain > step)  { *angle_deg += step; return false; }
    if (remain < -step) { *angle_deg -= step; return false; }
    *angle_deg = target;
    return true;
}

void arm_action_init(void)
{
    action_state = ARM_ACTION_STATE_IDLE;
    current_path = NULL;
    current_len = 0U;
    step_index = 0;
    phase_hold = false;
}

bool arm_action_start(ArmActionId_t action_id, uint32_t now_ms)
{
    if ((uint32_t)action_id >= (uint32_t)ARM_ACTION_COUNT ||
        action_state == ARM_ACTION_STATE_RUNNING)
    {
        return false;
    }

    const ArmActionSeq_t *seq = &sequences[action_id];

    /* 区域路径动作：启动那一刻按当前姿态选定路径（之后不再切换） */
    if (seq->path_in != NULL)
    {
        current_path = in_basket_zone() ? seq->path_in : seq->path_out;
        current_len = seq->zone_count;
    }
    else
    {
        current_path = seq->fixed;
        current_len = seq->fixed_count;
    }

    step_index = 0;
    begin_keyframe(arm_keyframe_get(current_path[0]));
    phase_start_ms = now_ms;
    action_state = ARM_ACTION_STATE_RUNNING;
    return true;
}

void arm_action_cancel(void)
{
    current_path = NULL;
    current_len = 0U;
    step_index = 0;
    phase_hold = false;
    action_state = ARM_ACTION_STATE_IDLE;
}

ArmActionState_t arm_action_get_state(void)
{
    return action_state;
}

void arm_action_update(uint32_t now_ms)
{
    if (action_state != ARM_ACTION_STATE_RUNNING ||
        current_path == NULL || current_len == 0U)
    {
        return;
    }

    /* 停留阶段：时间到进入下一帧 */
    if (phase_hold)
    {
        if ((uint32_t)(now_ms - phase_start_ms) < ARM_ACTION_HOLD_MS)
        {
            return;
        }
        ++step_index;
        if (step_index >= (int16_t)current_len)
        {
            /* 完成后直接回 IDLE：若停在 FINISHED，实测会导致脱离完成后
             * 首次 L2 表现异常（需要按两次才卸力）。FINISHED 枚举保留
             * 但不再使用。 */
            arm_action_cancel();
            return;
        }
        begin_keyframe(arm_keyframe_get(current_path[step_index]));
        phase_start_ms = now_ms;
    }

    /* 逼近阶段：三关节独立步进（速度 = 点动快速档） */
    ArmPose_t pose;
    if (!arm_get_pose(&pose))
    {
        arm_action_cancel();
        action_state = ARM_ACTION_STATE_ERROR;
        return;
    }

    const bool base_done    = step_joint(&pose.base_z_deg,  tgt_base,
                                         ARM_MANUAL_BASE_MAX_DPS, ARM_ACTION_TICK_S);
    const bool link1_done   = step_joint(&pose.link1_deg,   tgt_link1,
                                         ARM_MANUAL_SHOULDER_MAX_DPS, ARM_ACTION_TICK_S);
    const bool link2_done   = step_joint(&pose.link2_deg,   tgt_link2,
                                         ARM_MANUAL_ELBOW_MAX_DPS, ARM_ACTION_TICK_S);

    if (!arm_set_pose(&pose))
    {
        arm_action_cancel();
        action_state = ARM_ACTION_STATE_ERROR;
        return;
    }

    if (base_done && link1_done && link2_done)
    {
        /* 到位：按帧内状态切换夹爪，进入停留 */
        if (!grip_done)
        {
            const ArmKeyframe_t *kf = arm_keyframe_get(current_path[step_index]);
            if (kf->gripper != GRIPPER_UNDEF)
            {
                (void)gripper_set(kf->gripper);
            }
            grip_done = true;
        }
        phase_hold = true;
        phase_start_ms = now_ms;
    }
}
