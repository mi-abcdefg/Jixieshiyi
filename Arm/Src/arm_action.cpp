//
// 机械臂动作引擎实现（路径表 + 关键帧编号序列）
//
// 数据与逻辑分离：本文件只有"哪几个关键帧、按什么顺序"，没有任何角度；
// 角度全在 arm_keyframe.cpp 的表里。
//
// 本文件负责两件事（调用方都不用操心）：
//   1. 逐帧逼近：底座恒角速度直插（实测无需平滑）；大臂/小臂做斜坡平滑
//      逼近（期望速度=误差×增益、实际速度限加速度——远端恒速巡航、近端
//      比例减速，起步/急停都是渐变），到位后按帧切夹爪、停留 50ms；
//   2. 逐帧给力：每帧只给 power 掩码里的关节通电（卸力恢复的"脱离"动作
//      靠它先让大臂（肩）单独有力，其余关节仍软着）。
//
#include "arm_action.h"

#include "arm_keyframe.h"
#include "arm_control.h"
#include "arm_manual.h"    // 各关节快速档角速度宏（与点动同一数据源）
#include "gripper.h"
#include "servo_driver.h"

namespace {

/* ------------------------------ 引擎参数 ------------------------------ */
constexpr uint32_t kHoldMs = 50U;      // 关键帧到位后的停留时间（实测从 220 再压）
constexpr float kTickS = 0.020f;       // 推进周期 [s]，须与 ArmTask 周期一致

/* 动作巡航角速度 [°/s]：**动作引擎专用**，与点动快速档（90/60/60，摇杆用）
 * 脱钩——调点动手感不影响动作速度。平滑逼近的速度上限就是这个值。 */
constexpr float kCruiseBaseDps = 125.0f;      // 底座：不做平滑，恒角速度步进
constexpr float kCruiseShoulderDps = 85.0f;   // 大臂：平滑逼近巡航值
constexpr float kCruiseElbowDps = 85.0f;      // 小臂：平滑逼近巡航值

/* 平滑参数（按关节）：
 *   大臂 250°/s²（2026/10/21 实测从 450 调低 = 平滑度再提高，斜坡更缓）；
 *   小臂 450°/s² 保持；
 *   底座不做平滑（实测无需——MG995 带载本身就没有速度阶跃的抖动问题）。 */
constexpr float kShoulderAccel = 250.0f;   // 大臂速度斜坡加速度 [°/s²]
constexpr float kElbowAccel = 450.0f;      // 小臂速度斜坡加速度 [°/s²]
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

/* × 键"篮外低姿起步"变体的大臂门槛（用户角；肩换算镜像后 = 命令角）：
 * 判篮外且大臂低于此角 → 先到 kOutsideLift 过渡点再走原路线。 */
constexpr float kOutsideLiftShoulderUserDeg = 50.0f;

/* ------------------- 动作路径（关键帧编号的序列） ------------------- */

/* 回收缩：篮内经篮上高点、篮外经外侧高点，都收到同一个收缩位 */
constexpr KeyframeId kRetractFromBasket[] = {
    KeyframeId::kBasketHigh, KeyframeId::kRetract
};
constexpr KeyframeId kRetractFromOutside[] = {
    KeyframeId::kOuterHigh, KeyframeId::kRetract
};

/* 篮内外转移：
 *   篮内 → 篮外（出篮）：篮上高点 → 外侧高点 → 出篮终点（夹爪保持当前开合）
 *   篮外 → 篮内（入篮）：外侧高点 → 篮上高点 → 篮内点
 * 注：夹爪只在**终点帧**改状态，路径途中一律不动夹爪——在篮内先合爪会把
 *     篮子里的方块夹住、再一路带出来。 */
constexpr KeyframeId kTransitFromBasket[] = {
    KeyframeId::kBasketHigh, KeyframeId::kOuterHigh, KeyframeId::kPickupLow
};
constexpr KeyframeId kTransitFromOutside[] = {
    KeyframeId::kOuterHigh, KeyframeId::kBasketHigh, KeyframeId::kBasketIn
};
/* 篮外变体（2026/10/21）：判篮外且**大臂用户角 < 50°**（臂压得很低）时，
 * 先抬到篮外低姿起步过渡点再走原路线——低姿直接伸向篮上高点会扫到
 * 周围障碍。只作用于 × 键的篮外方向（kTransitFromOutside 的前插帧）。 */
constexpr KeyframeId kTransitFromOutsideLow[] = {
    KeyframeId::kOutsideLift, KeyframeId::kOuterHigh,
    KeyframeId::kBasketHigh, KeyframeId::kBasketIn
};

/* 脱离（卸力恢复）：先只撑直大臂（肩），再让大臂保持、底座单独回中位。
 * 逐帧的给力范围写在关键帧的 power 掩码里（见 arm_keyframe.cpp）。 */
constexpr KeyframeId kEscapePath[] = {
    KeyframeId::kEscapeShoulder, KeyframeId::kEscapeYaw
};

/* ------- R1 动作菜单的取放动作（2026/10/21 增，2026/10/23 调整路线） -------
 *   篮内起步：仍需**出篮**——篮上高点 → 外侧高点 → 终点；
 *   篮外起步：防扫前缀（低姿起步点 / 防扫收拢帧，按大臂高度二选一）之后
 *             **直奔终点**，不再绕外侧高点（防扫前缀已负责安全抬臂）。
 * 终点角度全在 arm_keyframe.cpp 的表里。 */
constexpr KeyframeId kFrontHighIn[] = {
    KeyframeId::kBasketHigh, KeyframeId::kOuterHigh, KeyframeId::kFrontHigh };
constexpr KeyframeId kFrontHighOut[] = {
    KeyframeId::kFrontHigh };

constexpr KeyframeId kSideRearIn[] = {
    KeyframeId::kBasketHigh, KeyframeId::kOuterHigh, KeyframeId::kPickupSideRear };
constexpr KeyframeId kSideRearOut[] = {
    KeyframeId::kPickupSideRear };

constexpr KeyframeId kSideFrontIn[] = {
    KeyframeId::kBasketHigh, KeyframeId::kOuterHigh, KeyframeId::kPickupSideFront };
constexpr KeyframeId kSideFrontOut[] = {
    KeyframeId::kPickupSideFront };

constexpr KeyframeId kCorePlatformIn[] = {
    KeyframeId::kBasketHigh, KeyframeId::kOuterHigh, KeyframeId::kEnergyCorePlatform };
constexpr KeyframeId kCorePlatformOut[] = {
    KeyframeId::kEnergyCorePlatform };

constexpr KeyframeId kUnitPlatLeftIn[] = {
    KeyframeId::kBasketHigh, KeyframeId::kOuterHigh, KeyframeId::kEnergyUnitPlatLeft };
constexpr KeyframeId kUnitPlatLeftOut[] = {
    KeyframeId::kEnergyUnitPlatLeft };

constexpr KeyframeId kUnitPlatRightIn[] = {
    KeyframeId::kBasketHigh, KeyframeId::kOuterHigh, KeyframeId::kEnergyUnitPlatRight };
constexpr KeyframeId kUnitPlatRightOut[] = {
    KeyframeId::kEnergyUnitPlatRight };

constexpr KeyframeId kFrontLowIn[] = {
    KeyframeId::kBasketHigh, KeyframeId::kOuterHigh, KeyframeId::kFrontLow };
constexpr KeyframeId kFrontLowOut[] = {
    KeyframeId::kFrontLow };

/* ------- 篮外起步前缀变体（2026/10/21 增，2026/10/23 菜单动作不再绕外侧） -------
 * 篮外执行动作时（× **进篮**除外，它只有低姿起步变体），第一帧按大臂
 * 高度二选一：
 *   大臂 < 50°：前缀 = 篮外低姿起步过渡点 kOutsideLift；
 *   大臂 ≥ 50°：前缀 = 防扫收拢帧 kOutsideTuck（tuck 语义：大臂只抬不
 *               降、小臂只折不展、底座保持——防止动作扫到障碍）。
 * 菜单动作的前缀之后**直奔终点**（防扫前缀已负责安全抬臂）；
 * 回收缩（△）不是菜单动作，仍经外侧高点。 */
constexpr KeyframeId kRetractLift[] = {
    KeyframeId::kOutsideLift, KeyframeId::kOuterHigh, KeyframeId::kRetract };
constexpr KeyframeId kRetractTuck[] = {
    KeyframeId::kOutsideTuck, KeyframeId::kOuterHigh, KeyframeId::kRetract };

constexpr KeyframeId kFrontHighLift[] = {
    KeyframeId::kOutsideLift, KeyframeId::kFrontHigh };
constexpr KeyframeId kFrontHighTuck[] = {
    KeyframeId::kOutsideTuck, KeyframeId::kFrontHigh };

constexpr KeyframeId kSideRearLift[] = {
    KeyframeId::kOutsideLift, KeyframeId::kPickupSideRear };
constexpr KeyframeId kSideRearTuck[] = {
    KeyframeId::kOutsideTuck, KeyframeId::kPickupSideRear };

constexpr KeyframeId kSideFrontLift[] = {
    KeyframeId::kOutsideLift, KeyframeId::kPickupSideFront };
constexpr KeyframeId kSideFrontTuck[] = {
    KeyframeId::kOutsideTuck, KeyframeId::kPickupSideFront };

constexpr KeyframeId kCorePlatformLift[] = {
    KeyframeId::kOutsideLift, KeyframeId::kEnergyCorePlatform };
constexpr KeyframeId kCorePlatformTuck[] = {
    KeyframeId::kOutsideTuck, KeyframeId::kEnergyCorePlatform };

constexpr KeyframeId kUnitPlatLeftLift[] = {
    KeyframeId::kOutsideLift, KeyframeId::kEnergyUnitPlatLeft };
constexpr KeyframeId kUnitPlatLeftTuck[] = {
    KeyframeId::kOutsideTuck, KeyframeId::kEnergyUnitPlatLeft };

constexpr KeyframeId kUnitPlatRightLift[] = {
    KeyframeId::kOutsideLift, KeyframeId::kEnergyUnitPlatRight };
constexpr KeyframeId kUnitPlatRightTuck[] = {
    KeyframeId::kOutsideTuck, KeyframeId::kEnergyUnitPlatRight };

constexpr KeyframeId kFrontLowLift[] = {
    KeyframeId::kOutsideLift, KeyframeId::kFrontLow };
constexpr KeyframeId kFrontLowTuck[] = {
    KeyframeId::kOutsideTuck, KeyframeId::kFrontLow };

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
 *  @note fixed.frames == nullptr 表示区域路径动作：inside = 启动时判定
 *        "篮内"走的路径；篮外方向按大臂高度再分两份——
 *        outside_low = 大臂 < kOutsideLiftShoulderUserDeg 时的前缀路径、
 *        outside_tuck = 大臂 ≥ 门槛时的防扫收拢前缀路径（二者都在前缀帧
 *        之后直奔终点；outside_tuck 空 = 无该变体，回退 outside——× 进篮
 *        就靠留空豁免收拢前缀）。 */
struct ActionSeq {
    Path fixed;
    Path inside;
    Path outside;
    Path outside_low;
    Path outside_tuck;
};

constexpr Path kNoPath = { nullptr, 0U };

const ActionSeq kSequences[] = {
    /* kRetract 回收缩 */
    { kNoPath,
      make_path(kRetractFromBasket),
      make_path(kRetractFromOutside),
      make_path(kRetractLift),
      make_path(kRetractTuck) },
    /* kTransit 篮内外转移（进篮方向豁免防扫收拢：只有低姿起步变体） */
    { kNoPath,
      make_path(kTransitFromBasket),
      make_path(kTransitFromOutside),
      make_path(kTransitFromOutsideLow),
      kNoPath },
    /* kEscape  脱离 */
    { make_path(kEscapePath), kNoPath, kNoPath, kNoPath, kNoPath },
    /* —— R1 动作菜单的取放动作（篮外起步前缀见上方注释）—— */
    /* kFrontHigh 前方高点 */
    { kNoPath,
      make_path(kFrontHighIn),
      make_path(kFrontHighOut),
      make_path(kFrontHighLift),
      make_path(kFrontHighTuck) },
    /* kPickupSideRear 侧后拾取位 */
    { kNoPath,
      make_path(kSideRearIn),
      make_path(kSideRearOut),
      make_path(kSideRearLift),
      make_path(kSideRearTuck) },
    /* kPickupSideFront 侧前拾取位 */
    { kNoPath,
      make_path(kSideFrontIn),
      make_path(kSideFrontOut),
      make_path(kSideFrontLift),
      make_path(kSideFrontTuck) },
    /* kEnergyCorePlatform 能量核心_高台 */
    { kNoPath,
      make_path(kCorePlatformIn),
      make_path(kCorePlatformOut),
      make_path(kCorePlatformLift),
      make_path(kCorePlatformTuck) },
    /* kEnergyUnitPlatLeft 能量单元_高台左 */
    { kNoPath,
      make_path(kUnitPlatLeftIn),
      make_path(kUnitPlatLeftOut),
      make_path(kUnitPlatLeftLift),
      make_path(kUnitPlatLeftTuck) },
    /* kEnergyUnitPlatRight 能量单元_高台右 */
    { kNoPath,
      make_path(kUnitPlatRightIn),
      make_path(kUnitPlatRightOut),
      make_path(kUnitPlatRightLift),
      make_path(kUnitPlatRightTuck) },
    /* kFrontLow 前方低点 */
    { kNoPath,
      make_path(kFrontLowIn),
      make_path(kFrontLowOut),
      make_path(kFrontLowLift),
      make_path(kFrontLowTuck) },
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
    /* 防扫收拢帧（kf.tuck，只用于路径首帧）：底座保持；大臂只抬不降
     * （低于表值抬到表值，已高于则保持当前）；小臂只折不展（高于表值
     * 折到表值，已低于则保持当前）。表值 = 帧内的用户角字段。 */
    if (kf.tuck) {
        base_cmd = ref.base_z_deg;
        const float sh_table = arm_clamp_angle(
            SERVO_LINK_1, link1_user_to_cmd(kf.shoulder_deg));
        link1_cmd = (ref.link1_deg > sh_table) ? ref.link1_deg : sh_table;
        const float el_table = arm_clamp_angle(
            SERVO_LINK_2, link2_user_to_cmd(kf.elbow_deg));
        link2_cmd = (ref.link2_deg < el_table) ? ref.link2_deg : el_table;
        return;
    }

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

/* 大臂用户角是否低于"篮外低姿起步"门槛（× 键变体的选择条件）。
 * 读不回来按"不低"处理：走原路线，安全侧。 */
bool shoulder_below_lift_threshold(void)
{
    ArmPose_t pose;
    if (!arm_get_pose(&pose)) {
        return false;
    }
    return link1_cmd_to_user(pose.link1_deg) < kOutsideLiftShoulderUserDeg;
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
 * 参数取值（2026/10/21 实测调整）：增益 5/s ⇒ 距目标 v巡航/5 时开始减速；
 * 加速度按关节分——大臂 250°/s²（平滑度优先）、小臂 450°/s²；底座不做
 * 平滑，恒角速度 125°/s 直插（MG995 带载实测无速度阶跃抖动问题）。 */
constexpr float kApproachGain = 5.0f;      // 位置误差 → 期望速度 [1/s]
constexpr float kArriveEpsDeg = 0.4f;      // 到位吸附门槛 [°]

/**
 * @brief 单关节恒角速度步进一个周期（底座用：不做平滑）
 * @return true = 已到位（本周期精确落在目标上）
 */
bool approach_const(float &angle_deg, float target_deg, float max_dps)
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

/**
 * @brief 单关节朝目标平滑逼近一个周期（大臂/小臂用）
 * @return true = 已到位（误差收进门槛，已吸附终点，速度清零）
 */
bool approach(float &angle_deg, float target_deg, float max_dps,
              float accel, float &vel_deg)
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
    const float dv_max = accel * kTickS;
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

    /* 路径选择：启动这一刻按当前位形定，之后不再切换
     * （运行中反复判定会随臂动而变，导致中途跳路径）。
     * 篮外方向：大臂 < 门槛 → 低姿起步前缀；否则带防扫收拢前缀的动作
     * 改走收拢前缀；都没有 → 基础篮外路线（× 进篮即此，豁免收拢）。 */
    const Path *path;
    if (seq.fixed.frames != nullptr) {
        path = &seq.fixed;
    } else if (in_basket_zone()) {
        path = &seq.inside;
    } else if (seq.outside_low.frames != nullptr &&
               shoulder_below_lift_threshold()) {
        path = &seq.outside_low;
    } else if (seq.outside_tuck.frames != nullptr) {
        path = &seq.outside_tuck;
    } else {
        path = &seq.outside;
    }

    if (path->frames == nullptr || path->count == 0U) {
        return false;
    }

    s_path = path->frames;
    s_len = path->count;
    s_index = 0U;

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
        base_done = approach_const(pose.base_z_deg, s_tgt_base, kCruiseBaseDps);
    } else {
        s_vel_base = 0.0f;      // 本帧不通电的关节不做推进，速度状态清零
    }
    if (link1_live) {
        link1_done = approach(pose.link1_deg, s_tgt_link1,
                              kCruiseShoulderDps, kShoulderAccel, s_vel_link1);
    } else {
        s_vel_link1 = 0.0f;
    }
    if (link2_live) {
        link2_done = approach(pose.link2_deg, s_tgt_link2,
                              kCruiseElbowDps, kElbowAccel, s_vel_link2);
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
