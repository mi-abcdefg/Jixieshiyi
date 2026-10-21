//
// 机械臂动作引擎实现（路径表 + 关键帧编号序列）
//
// 数据与逻辑分离：本文件只有"哪几个关键帧、按什么顺序"，没有任何角度；
// 角度全在 arm_keyframe.cpp 的表里。
//
// 本文件负责三件事（调用方都不用操心）：
//   1. 逐帧逼近：三关节做斜坡平滑逼近（期望速度=误差×增益、实际速度限
//      加速度——远端恒速巡航、近端比例减速，起步/急停都是渐变，恒速急停
//      会让舵机抖动），到位后按帧切夹爪、停留 300ms；
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
constexpr uint32_t kHoldMs = 220U;     // 关键帧到位后的停留时间（原 300，提速）
constexpr float kTickS = 0.020f;       // 推进周期 [s]，须与 ArmTask 周期一致

/* 动作巡航角速度 [°/s]：**动作引擎专用**，与点动快速档（90/60/60，摇杆用）
 * 脱钩——2026/10/21 实测平滑后偏慢，整体提速约 1.4×；点动手感不受影响。
 * 平滑逼近的速度上限就是这个值（远端恒速巡航），嫌慢/嫌快只动这里。 */
constexpr float kCruiseBaseDps = 125.0f;      // 底座（原借用点动 90）
constexpr float kCruiseShoulderDps = 85.0f;   // 大臂（原借用点动 60）
constexpr float kCruiseElbowDps = 85.0f;      // 小臂（原借用点动 60）
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

/* "不走回头路"的位置门槛（用户角），2026/10/21 实测迭代后：
 *   1. 大臂 < 35°：无条件走完整路径（不再看基座在哪个半区）；
 *   2. 基座在篮子侧半区（> 90°）且满足以下任一 → 也走完整路径：
 *      a) 大臂 < 60° 且 小臂 < 75°（整臂收拢贴地）；
 *      b) 大臂 < 45°（大臂压得比较低，单条件触发）。
 * 其余位形正常启用回头路优化。注意：肘的用户角 = 命令角 + 120、底座的
 * 用户角 = 命令角 − 90（换算见 arm_keyframe.h），比较统一在用户角域做。
 * 若实测仍不理想，把 plan_no_backtrack() 开头的规划整体停用即可
 * （路径原样逐帧走，该优化本来就是纯省时的）。 */
constexpr float kNoBacktrackShoulderVeryLowDeg = 35.0f;   // 无条件触发线
constexpr float kNoBacktrackShoulderUserMin = 60.0f;
constexpr float kNoBacktrackElbowUserMin = 75.0f;
constexpr float kNoBacktrackShoulderLowDeg = 45.0f;
constexpr float kNoBacktrackBaseUserMin = 90.0f;

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

/* 各关节的当前速度 [°/s]（动作平滑用，见 approach）。
 * 每帧开始/非本帧通电的关节都清零——动作的节奏是"到位→停留"，帧与帧
 * 之间速度本来就归零，清零不影响观感，还能防陈旧速度引起跳变。 */
float s_vel_base = 0.0f;
float s_vel_link1 = 0.0f;
float s_vel_link2 = 0.0f;

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

    /* 每帧从静止开始：速度状态清零（到位吸附时本来就≈0，这里兜底） */
    s_vel_base = 0.0f;
    s_vel_link1 = 0.0f;
    s_vel_link2 = 0.0f;

    /* 逐帧给力：例如"脱离"第 1 帧只让大臂（肩）有力，其余关节仍是软的 */
    apply_power_mask(kf.power);
    s_power = kf.power;
    return true;
}

/* ------------------------------ 动作平滑 ------------------------------
 * 简单实用的梯形过渡：每关节维护一个速度状态，
 *   期望速度 = 位置误差 × 增益（误差大 = 恒速巡航，误差小 = 比例减速），
 *   实际速度以有限加速度逼近期望速度。
 * 原实现是恒角速度步进、到位瞬间吸附终点——速度阶跃直接砸在舵机上，
 * 急停抖动明显。斜坡化之后起步/到位都是渐变，HOLD 停留时长不变。
 * 参数取值：增益 5/s ⇒ 距目标 v巡航/5 时开始减速；加速度 450°/s² ⇒
 * 肩/肘约 0.19s、底座约 0.28s 从静止到巡航。 */
constexpr float kApproachGain = 5.0f;      // 位置误差 → 期望速度 [1/s]
constexpr float kApproachAccel = 450.0f;   // 速度斜坡的加速度上限 [°/s²]
constexpr float kArriveEpsDeg = 0.4f;      // 到位吸附门槛 [°]

/**
 * @brief 单关节朝目标平滑逼近一个周期
 * @return true = 已到位（误差收进门槛，已吸附终点，速度清零）
 */
bool approach(float &angle_deg, float target_deg, float max_dps, float &vel_deg)
{
    const float remain = target_deg - angle_deg;

    /* 期望速度：远端恒速巡航，近端比例减速（负误差自然给出反向速度） */
    float v_des = remain * kApproachGain;
    if (v_des > max_dps) {
        v_des = max_dps;
    } else if (v_des < -max_dps) {
        v_des = -max_dps;
    }

    /* 实际速度以有限加速度逼近期望速度：起步/急停都是斜坡 */
    const float dv_max = kApproachAccel * kTickS;
    const float dv = v_des - vel_deg;
    if (dv > dv_max) {
        vel_deg += dv_max;
    } else if (dv < -dv_max) {
        vel_deg -= dv_max;
    } else {
        vel_deg = v_des;
    }

    angle_deg += vel_deg * kTickS;

    /* 到位：误差收进门槛即吸附终点（比例逼近的积分尾巴到此截止） */
    if (absf(target_deg - angle_deg) < kArriveEpsDeg) {
        angle_deg = target_deg;
        vel_deg = 0.0f;
        return true;
    }
    return false;
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

    const float shoulder_user = link1_cmd_to_user(cur.link1_deg);
    const float elbow_user = link2_cmd_to_user(cur.link2_deg);

    /* 大臂特别低（< 35°）：无条件走完整路径，基座位置不再参与判断 */
    if (shoulder_user < kNoBacktrackShoulderVeryLowDeg) {
        return;
    }

    /* 基座在篮子侧半区，且触发任一臂位门槛时，不启用本优化：
     * 必须按完整路径先走到过渡高点，见 kNoBacktrack* 注释 */
    if (base_cmd_to_user(cur.base_z_deg) > kNoBacktrackBaseUserMin &&
        ((shoulder_user < kNoBacktrackShoulderUserMin &&
          elbow_user < kNoBacktrackElbowUserMin) ||
         shoulder_user < kNoBacktrackShoulderLowDeg)) {
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
    s_vel_base = 0.0f;
    s_vel_link1 = 0.0f;
    s_vel_link2 = 0.0f;
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

    bool base_done = true;
    bool link1_done = true;
    bool link2_done = true;
    if (base_live) {
        base_done = approach(pose.base_z_deg, s_tgt_base,
                             kCruiseBaseDps, s_vel_base);
    } else {
        s_vel_base = 0.0f;      // 本帧不通电的关节不做平滑，速度状态清零
    }
    if (link1_live) {
        link1_done = approach(pose.link1_deg, s_tgt_link1,
                              kCruiseShoulderDps, s_vel_link1);
    } else {
        s_vel_link1 = 0.0f;
    }
    if (link2_live) {
        link2_done = approach(pose.link2_deg, s_tgt_link2,
                              kCruiseElbowDps, s_vel_link2);
    } else {
        s_vel_link2 = 0.0f;
    }

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
