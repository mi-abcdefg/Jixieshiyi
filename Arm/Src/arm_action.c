//
// 机械臂动作引擎实现（条件路径 + 关键帧序列）
//
// 推进细节：
//   - 关键帧字段先换算为命令角目标（ARM_KF_KEEP 字段取当前命令角，
//     即该关节保持不动），三关节以各自快速档角速度步进逼近；
//   - 三关节全部到位后，若帧内夹爪状态非"未定义"则切换夹爪，
//     随后停留 ARM_ACTION_HOLD_MS 进入下一帧；
//   - 条件入口（cond_entry）：启动时读底座当前用户角，≥
//     ARM_ACTION_BASKET_USER_THR 判为"篮内区"，经篮上高点过渡；
//     否则判为"篮外区"，经外侧高点过渡。过渡帧走完后进入正式步骤表。
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
#define ARM_ACTION_BASKET_USER_THR  135.0f  // 底座用户角阈值：≥ 判为篮内区
#define ARM_ACTION_TICK_S           0.020f  // 引擎推进周期 [s]，须与 ArmTask 周期一致

/* --------------------------- 动作定义（数据） --------------------------- */

/* 回收缩：条件入口之后到达最小收缩位（角度占位，待标定） */
static const ArmKeyframe_t retract_steps[] = {
    { 90.0f, 30.0f, 30.0f, GRIPPER_UNDEF },
};

/* 脱离伸展：大臂小臂升到行程中点（用户角 90）→ 底座转到行程中点 */
static const ArmKeyframe_t escape_steps[] = {
    { ARM_KF_KEEP, 90.0f, 90.0f, GRIPPER_UNDEF },
    { 90.0f, ARM_KF_KEEP, ARM_KF_KEEP, GRIPPER_UNDEF },
};

/* 固定放置：接近 → 张开 → 返回（位形占位，均为 HOME 用户角 90,90,90） */
static const ArmKeyframe_t home_kf      = { 90.0f, 90.0f, 90.0f, GRIPPER_UNDEF };
static const ArmKeyframe_t home_open_kf = { 90.0f, 90.0f, 90.0f, GRIPPER_OPEN };

static const ArmKeyframe_t place_steps[] = {
    { 90.0f, 90.0f, 90.0f, GRIPPER_UNDEF },
    { 90.0f, 90.0f, 90.0f, GRIPPER_OPEN },
    { 90.0f, 90.0f, 90.0f, GRIPPER_UNDEF },
};

/* 抓取模式：接近 → 闭合 → 抬起（位形占位） */
static const ArmKeyframe_t grab_close_kf = { 90.0f, 90.0f, 90.0f, GRIPPER_CLOSE };

static const ArmKeyframe_t grab1_steps[] = {
    { 90.0f, 90.0f, 90.0f, GRIPPER_UNDEF },
    { 90.0f, 90.0f, 90.0f, GRIPPER_CLOSE },
    { 90.0f, 90.0f, 90.0f, GRIPPER_UNDEF },
};

static const ArmKeyframe_t grab3_steps[] = {
    { 90.0f, 90.0f, 90.0f, GRIPPER_UNDEF },
    { 90.0f, 90.0f, 90.0f, GRIPPER_CLOSE },
    { 90.0f, 90.0f, 90.0f, GRIPPER_UNDEF },
};

#define ARRAY_SIZE(array) \
    ((uint8_t)(sizeof(array) / sizeof((array)[0])))

/**
 * @brief 动作序列定义
 * @note  cond_entry = true 时，启动先按当前姿态插入"高点"过渡帧
 */
typedef struct {
    const ArmKeyframe_t *steps;
    uint8_t step_count;
    bool cond_entry;
} ArmActionSeq_t;

static const ArmActionSeq_t sequences[ARM_ACTION_COUNT] = {
    [ARM_ACTION_RETRACT]     = { retract_steps, ARRAY_SIZE(retract_steps), true },
    [ARM_ACTION_ESCAPE]      = { escape_steps,  ARRAY_SIZE(escape_steps),  false },
    [ARM_ACTION_PLACE]       = { place_steps,   ARRAY_SIZE(place_steps),   false },
    [ARM_ACTION_GRAB_MODE_1] = { grab1_steps,   ARRAY_SIZE(grab1_steps),   false },
    [ARM_ACTION_GRAB_MODE_3] = { grab3_steps,   ARRAY_SIZE(grab3_steps),   false },
};

/* --------------------------- 状态机内部变量 --------------------------- */

static ArmActionState_t action_state = ARM_ACTION_STATE_IDLE;
static const ArmActionSeq_t *current_seq = NULL;
static int16_t step_index = -1;        // -1 = 条件过渡帧，>=0 = 正式步骤表下标
static bool phase_hold = false;        // true = 已到位，正在停留
static uint32_t phase_start_ms = 0U;
static float tgt_base = 0.0f, tgt_link1 = 0.0f, tgt_link2 = 0.0f;
static bool grip_done = false;         // 当前帧夹爪是否已切换

static bool arm_action_in_basket_zone(void);

/* 取当前帧指针（含条件过渡帧的虚拟下标） */
static const ArmKeyframe_t *current_keyframe(void)
{
    if (step_index < 0)
    {
        /* 条件过渡帧：按篮内/篮外选择高点 */
        const ArmKeyframeId_t id =
            arm_action_in_basket_zone() ? ARM_KF_ID_BASKET_HIGH
                                        : ARM_KF_ID_OUTER_HIGH;
        return arm_keyframe_get(id);
    }
    return &current_seq->steps[step_index];
}

/* 底座当前是否处于篮内区（用户角阈值判断，见宏注释） */
bool arm_action_in_basket_zone(void)
{
    float base_cmd = 0.0f;
    if (!servo_get_commanded_angle(SERVO_BASE_Z, &base_cmd))
    {
        return false;
    }
    const float base_user = ARM_UA_BASE_FROM_CMD(base_cmd);
    return base_user >= ARM_ACTION_BASKET_USER_THR;
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
    current_seq = NULL;
    step_index = -1;
    phase_hold = false;
}

bool arm_action_start(ArmActionId_t action_id, uint32_t now_ms)
{
    if ((uint32_t)action_id >= (uint32_t)ARM_ACTION_COUNT ||
        action_state == ARM_ACTION_STATE_RUNNING)
    {
        return false;
    }

    current_seq = &sequences[action_id];
    step_index = current_seq->cond_entry ? -1 : 0;
    phase_hold = false;
    begin_keyframe(current_keyframe());
    phase_start_ms = now_ms;
    action_state = ARM_ACTION_STATE_RUNNING;
    return true;
}

void arm_action_cancel(void)
{
    current_seq = NULL;
    step_index = -1;
    phase_hold = false;
    action_state = ARM_ACTION_STATE_IDLE;
}

ArmActionState_t arm_action_get_state(void)
{
    return action_state;
}

void arm_action_update(uint32_t now_ms)
{
    if (action_state != ARM_ACTION_STATE_RUNNING || current_seq == NULL)
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
        if (step_index >= (int16_t)current_seq->step_count)
        {
            arm_action_cancel();
            action_state = ARM_ACTION_STATE_FINISHED;
            return;
        }
        begin_keyframe(current_keyframe());
        phase_start_ms = now_ms;
    }

    /* 逼近阶段：三关节独立步进（速度 = 点动快速档） */
    const float dt_s = ARM_ACTION_TICK_S;
    ArmPose_t pose;
    if (!arm_get_pose(&pose))
    {
        arm_action_cancel();
        action_state = ARM_ACTION_STATE_ERROR;
        return;
    }

    const bool base_done    = step_joint(&pose.base_z_deg,  tgt_base,
                                         ARM_MANUAL_BASE_MAX_DPS, dt_s);
    const bool link1_done   = step_joint(&pose.link1_deg,   tgt_link1,
                                         ARM_MANUAL_SHOULDER_MAX_DPS, dt_s);
    const bool link2_done   = step_joint(&pose.link2_deg,   tgt_link2,
                                         ARM_MANUAL_ELBOW_MAX_DPS, dt_s);

    if (!arm_set_pose(&pose))
    {
        arm_action_cancel();
        action_state = ARM_ACTION_STATE_ERROR;
        return;
    }

    if (base_done && link1_done && link2_done)
    {
        /* 到位：按帧内状态切换夹爪，进入停留 */
        const ArmKeyframe_t *kf = current_keyframe();
        if (!grip_done && kf->gripper != GRIPPER_UNDEF)
        {
            (void)gripper_set(kf->gripper);
            grip_done = true;
        }
        phase_hold = true;
        phase_start_ms = now_ms;
    }
}
