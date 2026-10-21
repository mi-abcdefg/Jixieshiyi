//
// 机械臂动作引擎实现（路径表 + 关键帧编号序列）
//
// 数据与逻辑分离：本文件只有"哪几个关键帧、按什么顺序"，没有任何角度；
// 角度全在 arm_keyframe.cpp 的表里。
//
// 本文件负责三件事（调用方都不用操心）：
//   1. 逐帧逼近：三关节各以恒角速度步进，到位后按帧切夹爪、停留 300ms；
//   2. 逐帧给力：每帧只给 power 掩码里的关节通电（卸力恢复的"脱离"动作
//      靠它先让大臂（肩）单独有力，其余关节仍软着）；
//   3. 不走回头路：第 1 段位移与第 2 段方向相反时，把第 1 段退化为"保持"，
//      净位移留到第 2 段一次走完（只作用于开头这一对，不改后续路径）。
//

#include "arm_action.h"

#include "arm_keyframe.h"
#include "arm_control.h"
#include "arm_manual.h"    // 各关节快速档角速度宏（与点动同一数据源）
#include "gripper.h"
#include "servo_driver.h"

namespace {

/* ------------------------------ 引擎参数 ------------------------------ */
constexpr uint32_t kHoldMs = 300U;     // 关键帧到位后的停留时间
constexpr float kTickS = 0.020f;       // 推进周期 [s]，须与 ArmTask 周期一致
constexpr float kMoveEpsDeg = 0.001f;  // 判定"这一帧是否真的动"的角阈值

/* ---------------------- 篮内区判定（暂定值，待实车复核） ----------------------
 * 机械臂装于车身右侧、需能转往背后篮子方向：
 *   底座用户角 ≥ 160° 且 肩用户角 ≤ 100°  →  判为"篮内"；
 *   其余位形统称篮外。
 * 阈值以**用户角**表达（篮子侧 = 底座用户角大的一端；肩侧 2026/10/21 换算
 * 镜像后 user 0 = 下推，所以肩 ≤ 100 的物理含义 = 大臂处于**压低**半区——
 * 臂垂进篮子里，与实测一致）。
 * 历史备注：肩阈值最初（旧约定 user 0 = 上推）为"≤ 100"＝抬起半区；肩换算
 * 镜像时曾机械地跟着翻成"≥ 80"，实车实测判定相反——现取关于 90° 对称的
 * 另一半区"≤ 100"，物理含义改为压低半区。再错就只动这一个数。 */
constexpr float kInBasketBaseUserMin = 160.0f;
constexpr float kInBasketShoulderUserMax = 100.0f;

/* "不走回头路"的位置门槛（用户角）：起步时**大臂 < 60° 且 小臂 < 75°**
 * 同时成立（整臂收拢贴地）→ 不做回头路优化，老老实实按路径先经过过渡
 * 高点——低位起步若省掉抬升折返，臂会贴着低矮障碍直插终点
 * （2026/10/21 实测；初版只看大臂 45°，实测改双条件）。
 * 注意肘的用户角 = 命令角 + 120（换算见 arm_keyframe.h），比较统一在
 * 用户角域做。 */
constexpr float kNoBacktrackShoulderUserMin = 60.0f;
constexpr float kNoBacktrackElbowUserMin = 75.0f;

/* ------------------- 动作路径（关键帧编号的序列） ------------------- */

/* 回收缩：篮内经篮上高点、篮外经外侧高点，都收到同一个收缩位 */
constexpr KeyframeId kRetractFromBasket[] = {
    KeyframeId::kBasketHigh, KeyframeId::kRetract
};
constexpr KeyframeId kRetractFromOutside[] = {
    KeyframeId::kOuterHigh, KeyframeId::kRetract
};

/* 篮内外转移：
 *   篮内 → 篮外（出篮）：篮上高点 → 外侧高点 → 出篮终点（到位后夹爪合上）
 *   篮外 → 篮内（入篮）：外侧高点 → 篮上高点 → 篮内点
 * 注：夹爪只在**终点帧**改状态，路径途中一律不动夹爪——在篮内先合爪会把
 *     篮子里的方块夹住、再一路带出来。 */
constexpr KeyframeId kTransitFromBasket[] = {
    KeyframeId::kBasketHigh, KeyframeId::kOuterHigh, KeyframeId::kPickupLow
};
constexpr KeyframeId kTransitFromOutside[] = {
    KeyframeId::kOuterHigh, KeyframeId::kBasketHigh, KeyframeId::kBasketIn
};

/* 脱离（卸力恢复）：先只撑直大臂（肩），再让大臂保持、底座单独回中位。
 * 逐帧的给力范围写在关键帧的 power 掩码里（见 arm_keyframe.cpp）。 */
constexpr KeyframeId kEscapePath[] = {
    KeyframeId::kEscapeShoulder, KeyframeId::kEscapeYaw
};

/* ------- R1 动作菜单的取放动作（2026/10/21 增） -------
 * 六个动作同构：先按篮内/篮外判定经过渡高点（篮内→篮上高点，篮外→外侧
 * 高点），再到各自的取放点。终点角度全在 arm_keyframe.cpp 的表里。 */
constexpr KeyframeId kCoreShelfIn[] = {
    KeyframeId::kBasketHigh, KeyframeId::kEnergyCoreShelf };
constexpr KeyframeId kCoreShelfOut[] = {
    KeyframeId::kOuterHigh, KeyframeId::kEnergyCoreShelf };

constexpr KeyframeId kUnitShelfLeftIn[] = {
    KeyframeId::kBasketHigh, KeyframeId::kEnergyUnitShelfLeft };
constexpr KeyframeId kUnitShelfLeftOut[] = {
    KeyframeId::kOuterHigh, KeyframeId::kEnergyUnitShelfLeft };

constexpr KeyframeId kCorePlatformIn[] = {
    KeyframeId::kBasketHigh, KeyframeId::kEnergyCorePlatform };
constexpr KeyframeId kCorePlatformOut[] = {
    KeyframeId::kOuterHigh, KeyframeId::kEnergyCorePlatform };

constexpr KeyframeId kUnitPlatLeftIn[] = {
    KeyframeId::kBasketHigh, KeyframeId::kEnergyUnitPlatLeft };
constexpr KeyframeId kUnitPlatLeftOut[] = {
    KeyframeId::kOuterHigh, KeyframeId::kEnergyUnitPlatLeft };

constexpr KeyframeId kUnitPlatRightIn[] = {
    KeyframeId::kBasketHigh, KeyframeId::kEnergyUnitPlatRight };
constexpr KeyframeId kUnitPlatRightOut[] = {
    KeyframeId::kOuterHigh, KeyframeId::kEnergyUnitPlatRight };

constexpr KeyframeId kFrontLowIn[] = {
    KeyframeId::kBasketHigh, KeyframeId::kFrontLow };
constexpr KeyframeId kFrontLowOut[] = {
    KeyframeId::kOuterHigh, KeyframeId::kFrontLow };

/** 一条路径 */
struct Path {
    const KeyframeId *frames;
    uint8_t count;
};

/* 由数组直接造一条路径：帧数从数组长度推导，**不要再手写计数**
 * （手写计数与数组不同步就是这个文件最容易犯的错：多读/少读一帧）。 */
template <uint8_t N>
constexpr Path make_path(const KeyframeId (&frames)[N])
{
    return Path{ frames, N };
}

/** 一个动作的路径定义
 *  @note fixed.frames == nullptr 表示双路径动作，用 inside/outside 二选一；
 *        inside = 启动时判定为"篮内"时走的路径，outside = 判定为"篮外"时走的路径。
 *        两条路径长度可以不等（引擎按选定那条的 count 推进）。 */
struct ActionSeq {
    Path fixed;
    Path inside;
    Path outside;
};

constexpr Path kNoPath = { nullptr, 0U };

const ActionSeq kSequences[] = {
    /* kRetract 回收缩 */
    { kNoPath,
      make_path(kRetractFromBasket),
      make_path(kRetractFromOutside) },
    /* kTransit 篮内外转移 */
    { kNoPath,
      make_path(kTransitFromBasket),
      make_path(kTransitFromOutside) },
    /* kEscape  脱离 */
    { make_path(kEscapePath), kNoPath, kNoPath },
    /* —— R1 动作菜单的取放动作（先经过渡高点，再到取放点）—— */
    /* kEnergyCoreShelf 能量核心_物资架 */
    { kNoPath,
      make_path(kCoreShelfIn),
      make_path(kCoreShelfOut) },
    /* kEnergyUnitShelfLeft 能量单元_物资架下左 */
    { kNoPath,
      make_path(kUnitShelfLeftIn),
      make_path(kUnitShelfLeftOut) },
    /* kEnergyCorePlatform 能量核心_高台 */
    { kNoPath,
      make_path(kCorePlatformIn),
      make_path(kCorePlatformOut) },
    /* kEnergyUnitPlatLeft 能量单元_高台左 */
    { kNoPath,
      make_path(kUnitPlatLeftIn),
      make_path(kUnitPlatLeftOut) },
    /* kEnergyUnitPlatRight 能量单元_高台右 */
    { kNoPath,
      make_path(kUnitPlatRightIn),
      make_path(kUnitPlatRightOut) },
    /* kFrontLow 前方低点 */
    { kNoPath,
      make_path(kFrontLowIn),
      make_path(kFrontLowOut) },
};

static_assert(sizeof(kSequences) / sizeof(kSequences[0]) ==
              static_cast<unsigned long>(ActionId::kCount),
              "动作表长度必须与 ActionId::kCount 一致（新增动作时同步两处）");

/* --------------------------- 状态机内部变量 --------------------------- */
ActionState s_state = ActionState::kIdle;
const KeyframeId *s_path = nullptr;     // 本次动作选定的路径
uint8_t s_len = 0U;                     // 路径帧数
uint8_t s_index = 0U;                   // 当前帧下标
uint32_t s_power = ARM_JOINT_ALL;       // 当前帧通电的关节（写/推进都只限这些）
bool s_holding = false;                 // true = 已到位，正在停留
uint32_t s_hold_start_ms = 0U;          // 停留起点
float s_tgt_base = 0.0f;                // 当前帧的目标命令角
float s_tgt_link1 = 0.0f;
float s_tgt_link2 = 0.0f;
GripperState_t s_tgt_gripper = GRIPPER_UNDEF;   // 当前帧的夹爪目标（到位时切）

/* "不走回头路"的挂起项：某一帧对某些关节退化为保持（只作用于开头那一对） */
bool s_nb_pending = false;
KeyframeId s_nb_id = KeyframeId::kBasketHigh;
uint32_t s_nb_mask = 0U;

float absf(float v)
{
    return (v < 0.0f) ? -v : v;
}

/* 把关键帧换算成三个关节的命令角目标。
 * kKeep 取 ref 里的对应值；结果已按各关节量程夹过（arm_control 的
 * arm_clamp_angle 是这条规则的唯一实现）。纯函数，不改状态机。 */
void compute_targets(const ArmKeyframe &kf, const ArmPose_t &ref,
                     float &base_cmd, float &link1_cmd, float &link2_cmd)
{
    base_cmd = arm_clamp_angle(
        SERVO_BASE_Z,
        (kf.base_deg == kKeep) ? ref.base_z_deg : base_user_to_cmd(kf.base_deg));
    link1_cmd = arm_clamp_angle(
        SERVO_LINK_1,
        (kf.shoulder_deg == kKeep) ? ref.link1_deg : link1_user_to_cmd(kf.shoulder_deg));
    link2_cmd = arm_clamp_angle(
        SERVO_LINK_2,
        (kf.elbow_deg == kKeep) ? ref.link2_deg : link2_user_to_cmd(kf.elbow_deg));
}

/* 篮内位形判定（读回 = 命令角，非反馈） */
bool in_basket_zone(void)
{
    ArmPose_t pose;
    if (!arm_get_pose(&pose)) {
        return false;
    }
    const float base_user = base_cmd_to_user(pose.base_z_deg);
    const float shoulder_user = link1_cmd_to_user(pose.link1_deg);
    return (base_user >= kInBasketBaseUserMin) &&
           (shoulder_user <= kInBasketShoulderUserMax);
}

/* 让舵机的给力状态与目标掩码一致（只动需要变的那几路，幂等） */
void apply_power_mask(uint32_t want)
{
    const uint32_t have = arm_get_joint_torque_mask();
    const uint32_t to_on = want & ~have;
    const uint32_t to_off = have & ~want;

    if (to_on != 0U) {
        arm_set_joint_torque(to_on, true);
    }
    if (to_off != 0U) {
        arm_set_joint_torque(to_off, false);
    }
}

/**
 * @brief 开始逼近一个关键帧：换算目标、切换该帧要求的给力范围
 * @return false = 读回当前命令角失败（无法解析 kKeep），调用方应取消动作
 */
bool begin_frame(KeyframeId id)
{
    const ArmKeyframe kf = get_keyframe(id);

    /* kKeep 字段要保持"该关节当前值"，从 arm_control 读回命令角 */
    ArmPose_t pose;
    if (!arm_get_pose(&pose)) {
        return false;
    }

    compute_targets(kf, pose, s_tgt_base, s_tgt_link1, s_tgt_link2);

    /* 不走回头路：开头那一对帧里被判"该保持"的关节，目标改回当前值，
     * 净位移自然留到下一帧（下一帧的目标是绝对值，不受影响）。 */
    if (s_nb_pending && id == s_nb_id) {
        if ((s_nb_mask & ARM_JOINT_BASE_Z) != 0U) {
            s_tgt_base = pose.base_z_deg;
        }
        if ((s_nb_mask & ARM_JOINT_SHOULDER) != 0U) {
            s_tgt_link1 = pose.link1_deg;
        }
        if ((s_nb_mask & ARM_JOINT_ELBOW) != 0U) {
            s_tgt_link2 = pose.link2_deg;
        }
        s_nb_pending = false;   // 只作用于开头这一对帧
    }

    s_tgt_gripper = kf.gripper;   // GRIPPER_UNDEF = 本帧不动夹爪
    s_holding = false;

    /* 逐帧给力：例如"脱离"第 1 帧只让大臂（肩）有力，其余关节仍是软的 */
    apply_power_mask(kf.power);
    s_power = kf.power;
    return true;
}

/**
 * @brief 单关节朝目标恒角速度步进一个周期
 * @return true = 已到位（本周期精确落在目标上）
 */
bool approach(float &angle_deg, float target_deg, float max_dps)
{
    const float step = max_dps * kTickS;
    const float remain = target_deg - angle_deg;
    if (remain > step) {
        angle_deg += step;
        return false;
    }
    if (remain < -step) {
        angle_deg -= step;
        return false;
    }
    angle_deg = target_deg;
    return true;
}

/* 第 1 段位移（from→mid）与第 2 段位移（mid→to）方向相反？
 * 任一段为 0 不算相反（"保持"本身没有回头路可言）。 */
bool opposite_legs(float from, float mid, float to)
{
    const float d1 = mid - from;
    const float d2 = to - mid;
    return (d1 * d2) < -kMoveEpsDeg;
}

/**
 * @brief 规划"不走回头路"（只在动作起点算一次）
 *
 * 找出路径里**第一段真正有位移**的关键帧 i（跳过只切夹爪、三关节全 KEEP 的
 * 过渡帧），再逐关节比较"进第 i 帧"与"第 i 帧→第 i+1 帧"两段位移：
 * 方向相反的关节在第 i 帧退化为"保持"，净位移由第 i+1 帧一次走完。
 * 例：进第 1 帧要逆时针 30°、进第 2 帧要顺时针 50° → 第 1 帧不动，
 * 第 2 帧顺时针 20°（终点完全一样，但不再先出去再折回来）。
 */
void plan_no_backtrack(void)
{
    s_nb_pending = false;
    s_nb_mask = 0U;

    if (s_len < 2U || s_path == nullptr) {
        return;
    }

    ArmPose_t cur;
    if (!arm_get_pose(&cur)) {
        return;
    }

    /* 大臂、小臂同时低于门槛（整臂收拢贴地）时不启用本优化：必须按
     * 完整路径先走到过渡高点，见 kNoBacktrack*UserMin 注释 */
    if (link1_cmd_to_user(cur.link1_deg) < kNoBacktrackShoulderUserMin &&
        link2_cmd_to_user(cur.link2_deg) < kNoBacktrackElbowUserMin) {
        return;
    }

    /* 找第一段有位移的帧（它之前的帧全是不动的过渡帧，位形保持不变） */
    uint8_t i = 0U;
    float base_i = 0.0f;
    float link1_i = 0.0f;
    float link2_i = 0.0f;
    for (; i + 1U < s_len; i++) {
        compute_targets(get_keyframe(s_path[i]), cur, base_i, link1_i, link2_i);
        const float moved = absf(base_i - cur.base_z_deg) +
                            absf(link1_i - cur.link1_deg) +
                            absf(link2_i - cur.link2_deg);
        if (moved > kMoveEpsDeg) {
            break;
        }
    }
    if (i + 1U >= s_len) {
        return;     // 整条路径都不动（或只剩一帧），无需规划
    }

    /* 下一帧的目标（kKeep 按"第 i 帧到位后"的位形解析） */
    ArmPose_t after = cur;
    after.base_z_deg = base_i;
    after.link1_deg = link1_i;
    after.link2_deg = link2_i;

    float base_n = 0.0f;
    float link1_n = 0.0f;
    float link2_n = 0.0f;
    compute_targets(get_keyframe(s_path[i + 1U]), after, base_n, link1_n, link2_n);

    uint32_t mask = 0U;
    if (opposite_legs(cur.base_z_deg, base_i, base_n)) {
        mask |= ARM_JOINT_BASE_Z;
    }
    if (opposite_legs(cur.link1_deg, link1_i, link1_n)) {
        mask |= ARM_JOINT_SHOULDER;
    }
    if (opposite_legs(cur.link2_deg, link2_i, link2_n)) {
        mask |= ARM_JOINT_ELBOW;
    }

    if (mask != 0U) {
        s_nb_id = s_path[i];
        s_nb_mask = mask;
        s_nb_pending = true;
    }
}

/* 回到空闲：取消与正常完成共用（都不留中间态） */
void reset_state(void)
{
    /* 收尾：动作可能把部分关节留在卸力状态（"脱离"动作的给力是渐进的），
     * 结束时补回全有力，避免留下"半个臂"的隐状态。
     * 用户处在卸力模式（L2）时不动——那正是他要的状态。 */
    if (!arm_is_relaxed()) {
        const uint32_t off = ARM_JOINT_ALL & ~arm_get_joint_torque_mask();
        if (off != 0U) {
            arm_set_joint_torque(off, true);
        }
    }

    s_state = ActionState::kIdle;
    s_path = nullptr;
    s_len = 0U;
    s_index = 0U;
    s_holding = false;
    s_power = ARM_JOINT_ALL;
    s_tgt_gripper = GRIPPER_UNDEF;
    s_nb_pending = false;
    s_nb_mask = 0U;
}

}   // namespace

void arm_action_init(void)
{
    reset_state();
}

bool arm_action_start(ActionId id, uint32_t now_ms)
{
    (void)now_ms;   // 停留计时从"到位的那一刻"开始，启动时刻不参与

    if (static_cast<uint8_t>(id) >= static_cast<uint8_t>(ActionId::kCount) ||
        s_state == ActionState::kRunning) {
        return false;
    }

    const ActionSeq &seq = kSequences[static_cast<uint8_t>(id)];

    /* 双路径动作：启动这一刻按当前位形二选一，之后不再切换
     * （运行中反复判定会随臂动而变，导致中途跳路径） */
    const Path *path = (seq.fixed.frames != nullptr)
                       ? &seq.fixed
                       : (in_basket_zone() ? &seq.inside : &seq.outside);

    if (path->frames == nullptr || path->count == 0U) {
        return false;
    }

    s_path = path->frames;
    s_len = path->count;
    s_index = 0U;

    /* 规划"不走回头路"要读当前位形，必须在进第一帧之前做：
     * begin_frame 会据此把第一段退化为保持。 */
    plan_no_backtrack();

    if (!begin_frame(s_path[0])) {
        reset_state();
        return false;
    }

    s_state = ActionState::kRunning;
    return true;
}

void arm_action_cancel(void)
{
    reset_state();
}

ActionState arm_action_get_state(void)
{
    return s_state;
}

void arm_action_update(uint32_t now_ms)
{
    if (s_state != ActionState::kRunning || s_path == nullptr) {
        return;
    }

    /* 停留阶段：时间到进下一帧；最后一帧停留结束 = 动作完成（直接回 IDLE） */
    if (s_holding) {
        if (static_cast<uint32_t>(now_ms - s_hold_start_ms) < kHoldMs) {
            return;
        }
        s_holding = false;
        ++s_index;
        if (s_index >= s_len) {
            reset_state();
            return;
        }
        if (!begin_frame(s_path[s_index])) {
            reset_state();
            return;
        }
    }

    /* 逼近阶段：只推进本帧通电的关节（其余关节仍是软的，不能被写） */
    ArmPose_t pose;
    if (!arm_get_pose(&pose)) {
        reset_state();      // 读不回来就没法继续，就地停在当前姿态
        return;
    }

    const bool base_live = (s_power & ARM_JOINT_BASE_Z) != 0U;
    const bool link1_live = (s_power & ARM_JOINT_SHOULDER) != 0U;
    const bool link2_live = (s_power & ARM_JOINT_ELBOW) != 0U;

    const bool base_done =
        !base_live || approach(pose.base_z_deg, s_tgt_base, ARM_MANUAL_BASE_MAX_DPS);
    const bool link1_done =
        !link1_live || approach(pose.link1_deg, s_tgt_link1, ARM_MANUAL_SHOULDER_MAX_DPS);
    const bool link2_done =
        !link2_live || approach(pose.link2_deg, s_tgt_link2, ARM_MANUAL_ELBOW_MAX_DPS);

    /* 只写本帧通电的关节；夹爪不走这条路（由 gripper 模块负责） */
    if (!arm_set_pose_masked(&pose, s_power & ~ARM_JOINT_GRIPPER)) {
        reset_state();      // 写入被拒绝（例如整臂又被卸力）：收摊，不留半截状态
        return;
    }

    if (base_done && link1_done && link2_done) {
        /* 三关节到位：按本帧定义切夹爪，然后停留 */
        if (s_tgt_gripper != GRIPPER_UNDEF) {
            (void)gripper_set(s_tgt_gripper);
        }
        s_holding = true;
        s_hold_start_ms = now_ms;
    }
}
