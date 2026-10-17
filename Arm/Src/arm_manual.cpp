//
// 机械臂手动点动实现
//
// 依赖关系刻意收窄：只依赖 arm_ik（运动学）和 arm_control（下发/读回），
// 不感知 PS2、控制模式和动作状态机——模式门控与"手动打断动作"由
// arm_task 编排层完成。
//

#include "arm_manual.h"
#include "arm_ik.h"
#include "arm_control.h"

#include <math.h>

#define ARM_MANUAL_PI 3.14159265f

static float fabsf_local(float x)
{
    return x < 0.0f ? -x : x;
}

/* 三者中的绝对值最大者（关节同步限速用） */
static float max_abs3(float a, float b, float c)
{
    float m = fabsf_local(a);
    if (fabsf_local(b) > m) m = fabsf_local(b);
    if (fabsf_local(c) > m) m = fabsf_local(c);
    return m;
}

/* 内部状态 */
static uint32_t last_update_ms = 0U;
static bool has_last_update = false;
static bool active_last = false;       // 上一周期是否处于"活动"（任意输入超死区）
static bool target_valid = false;      // 柱坐标目标是否已从当前姿态播种
static float tgt_theta_deg = 0.0f;     // 方位角（= 底座关节角）
static float tgt_r_m = 0.0f;           // 径向距离
static float tgt_z_m = 0.0f;           // 高度

/*
 * 把柱坐标目标夹到工作空间内。
 * r 用 IK 的可达边界；z 用本模块的软限位（唯一所有者，IK 不重复夹 z）；
 * theta 用底座舵机的行程范围——三者和舵机限幅保持一致，
 * 否则下发时被舵机层夹掉的部分会让下一拍的正解起点跳变。
 */
static void ArmManual_ClampTarget(void)
{
    float r_min;
    float r_max;
    arm_ik_get_reach(&r_min, &r_max);

    if (tgt_r_m < r_min) tgt_r_m = r_min;
    if (tgt_r_m > r_max) tgt_r_m = r_max;
    if (tgt_z_m < ARM_MANUAL_Z_MIN_M) tgt_z_m = ARM_MANUAL_Z_MIN_M;
    if (tgt_z_m > ARM_MANUAL_Z_MAX_M) tgt_z_m = ARM_MANUAL_Z_MAX_M;
    if (tgt_theta_deg < ARM_MANUAL_THETA_MIN_DEG) {
        tgt_theta_deg = ARM_MANUAL_THETA_MIN_DEG;
    }
    if (tgt_theta_deg > ARM_MANUAL_THETA_MAX_DEG) {
        tgt_theta_deg = ARM_MANUAL_THETA_MAX_DEG;
    }
}

/*
 * 从当前指令姿态播种柱坐标目标。
 * 失败（尚未下发过姿态）时 target_valid 保持 false，点动被拒绝。
 */
static void ArmManual_SeedTarget(void)
{
    ArmPose_t pose;
    if (!arm_get_pose(&pose))
    {
        target_valid = false;
        return;
    }

    float theta_deg;
    arm_ik_forward(pose.base_z_deg, pose.link1_deg, pose.link2_deg,
                   &theta_deg, &tgt_r_m, &tgt_z_m);
    tgt_theta_deg = theta_deg;

    ArmManual_ClampTarget();
    target_valid = true;
}

void arm_manual_init(void)
{
    has_last_update = false;
    active_last = false;
    target_valid = false;
}

void arm_manual_update(uint32_t now_ms,
                       float radial_norm, float tangential_norm,
                       float vertical_norm, float yaw_norm)
{
    /* 周期时间差；首调用只记时间不下发（此时还没有可靠的 dt）。 */
    if (!has_last_update)
    {
        last_update_ms = now_ms;
        has_last_update = true;
        return;
    }
    const uint32_t dt_ms = (uint32_t)(now_ms - last_update_ms);
    last_update_ms = now_ms;

    /* 死区与活动判定：全部输入都在死区内 = 松杆，保持现状并回到待激活 */
    const bool active =
        (radial_norm > ARM_MANUAL_DEADZONE) || (radial_norm < -ARM_MANUAL_DEADZONE) ||
        (tangential_norm > ARM_MANUAL_DEADZONE) || (tangential_norm < -ARM_MANUAL_DEADZONE) ||
        (vertical_norm > ARM_MANUAL_DEADZONE) || (vertical_norm < -ARM_MANUAL_DEADZONE) ||
        (yaw_norm > ARM_MANUAL_DEADZONE) || (yaw_norm < -ARM_MANUAL_DEADZONE);

    if (!active)
    {
        active_last = false;
        return;
    }

    /*
     * 静止 → 推杆的边沿：重新播种起点。
     * 这样无论上次是动作序列还是点动留下的姿态，点动永远从"当前指令角"接续，
     * 不会从陈旧目标跳变过去。
     */
    if (!active_last || !target_valid)
    {
        ArmManual_SeedTarget();
    }
    active_last = true;

    if (!target_valid)
    {
        return;   // 播种失败（正常流程不会发生），保持现状
    }

    /* dt 上限保护：长时间阻塞（断点/调度抖动）后不做大步积分 */
    const float dt_s = (dt_ms > ARM_MANUAL_MAX_DT_MS)
                       ? (float)ARM_MANUAL_MAX_DT_MS / 1000.0f
                       : (float)dt_ms / 1000.0f;

    /* 径向 + 切向 + 垂直 + 自转，分别按各自满杆速度积分 */
    tgt_r_m += radial_norm * ARM_MANUAL_MAX_LINEAR_MPS * dt_s;

    /* 切向：Δθ = v_t / r。r 有下限（IK 可达边界 > 0），不会除零 */
    float r_min;
    float r_max;
    arm_ik_get_reach(&r_min, &r_max);
    const float r_for_div = (tgt_r_m > r_min) ? tgt_r_m : r_min;
    tgt_theta_deg += tangential_norm * ARM_MANUAL_MAX_LINEAR_MPS *
                     dt_s / r_for_div * (180.0f / ARM_MANUAL_PI);

    tgt_z_m += vertical_norm * ARM_MANUAL_MAX_VERTICAL_MPS * dt_s;
    tgt_theta_deg += yaw_norm * ARM_MANUAL_MAX_YAW_DPS * dt_s;

    ArmManual_ClampTarget();

    /* 逆解；夹爪与点动无关，保持当前指令值 */
    ArmJointAngles_t angles;
    if (!arm_ik_solve(tgt_theta_deg, tgt_r_m, tgt_z_m, &angles))
    {
        return;
    }

    ArmPose_t pose;
    if (!arm_get_pose(&pose))
    {
        return;
    }

    /* ---------------- 关节速度限制 ----------------
     * 以当前指令角为基准计算本拍增量；任一关节超过上限时，
     * 三个关节的增量按同一比例缩放——同步减速、路径形状不变。
     * 这削平了奇异区（行程末端）的关节速度尖峰，消除舵机突然
     * 加速和一顿一顿的现象。 */
    const float max_step_deg = ARM_MANUAL_MAX_JOINT_DPS * dt_s;
    const float d_base  = angles.base_z_deg - pose.base_z_deg;
    const float d_link1 = angles.link1_deg  - pose.link1_deg;
    const float d_link2 = angles.link2_deg  - pose.link2_deg;
    const float max_delta = max_abs3(d_base, d_link1, d_link2);

    if (max_delta > max_step_deg && max_delta > 0.0f)
    {
        const float k = max_step_deg / max_delta;
        pose.base_z_deg += d_base  * k;
        pose.link1_deg  += d_link1 * k;
        pose.link2_deg  += d_link2 * k;
    }
    else
    {
        pose.base_z_deg = angles.base_z_deg;
        pose.link1_deg  = angles.link1_deg;
        pose.link2_deg  = angles.link2_deg;
    }

    (void)arm_set_pose(&pose);
}
