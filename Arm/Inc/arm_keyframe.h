//
// 机械臂关键帧层：用户角约定、用户角→命令角换算、命名关键帧表
//
// 本层是**纯数据**层——整个机械臂唯一存放具体角度的地方：标定角度只改
// arm_keyframe.cpp 的关键帧表，动作引擎只认 KeyframeId，不认角度。
//
// 用户角约定（0~180，与舵机安装方向解耦）：
//   底座 base      user 0 = 工作侧端  user 180 = 篮子侧端（两车的物理含义
//                  相同；命令角映射按车镜像，见下方 base_*_to_cmd 的 #if）
//   肩   shoulder  user 0 = 下推极限（收拢）     user 180 = 上推极限（竖直）
//   肘   elbow     user 0 = 下推极限（cmd −120）user 180 = 上推极限（cmd +60）
//   爪   gripper   不用角度，以"开/合/不变"参与关键帧（见 gripper.h）
//
// 命令角 = 舵机配置表里的角度域单位（见 servo_driver.cpp 的配置表）；
// 关键帧一律以用户角存储，换算只在下面这几个 constexpr 里发生。
// 某关节实测方向镜像时，**改这里的一行换算**，不要改关键帧数值
// （交接文档踩坑 9：改宏不改表）。
//

#ifndef CRTC2026_ROBOTA_ARM_KEYFRAME_H
#define CRTC2026_ROBOTA_ARM_KEYFRAME_H

#pragma once
#include <stdint.h>

#include "arm_control.h"   // ARM_JOINT_xxx 关节位掩码（关键帧的 power 字段用）
#include "gripper.h"       // GripperState_t：夹爪以开/合/未定义参与关键帧

/* ------------------------- 关键帧字段哨兵 -------------------------
 * 关键帧某字段等于 kKeep 表示"该关节保持当前命令角不动"。
 * 255 在 0~180 用户角之外，不会与真实角度混淆（比较为精确相等，
 * 依赖的是同一个字面量 255.0f，见 arm_keyframe.cpp 的说明）。 */
constexpr float kKeep = 255.0f;

/* ------------------------- 用户角 → 命令角 -------------------------
 * 底座方向**按车分套**（2026/10/21）：A 车的基座舵机与 B 车安装方向镜像，
 * 动作关键帧以用户角表达物理方位，因此 A 车的动作换算需镜像——摇杆点动
 * 直接工作在命令角域，两车手感一致，**不参与镜像**。
 *   B 车（原标定版）：cmd = 270 − user，即 user 0 → cmd 270（工作侧端）
 *   A 车（镜像版）  ：cmd =  90 + user，即 user 180 → cmd 270（篮子侧端）
 * 两车共用同一套关键帧数值和篮内阈值（"篮子侧 = user 180"的约定在两车
 * 上都成立），只翻这里。切换方式同 MOTOR_INVERT_xx：改 #if 的 0/1。 */
#if 1    /* ===== A 车配置（动作镜像） ===== */
constexpr float base_user_to_cmd(float user_deg)
{
    return 90.0f + user_deg;
}
#else    /* ===== B 车配置 ===== */
constexpr float base_user_to_cmd(float user_deg)
{
    return 270.0f - user_deg;
}
#endif

/* 命令角 → 用户角：必须与 base_user_to_cmd 严格互逆（篮内判定用读回
 * 反算），翻转正变换时这两行必须同时翻。 */
#if 1    /* ===== A 车配置 ===== */
constexpr float base_cmd_to_user(float cmd_deg)
{
    return cmd_deg - 90.0f;
}
#else    /* ===== B 车配置 ===== */
constexpr float base_cmd_to_user(float cmd_deg)
{
    return 270.0f - cmd_deg;
}
#endif

/* 肩方向标定（2026/10/21 实车复核后**已镜像一次**）：原式 cmd = 180 − user
 * 使用户角上下颠倒（动作按"肩抬起"给的用户角实际落到收拢端）。翻转为
 * cmd = user，映射关系为
 *   user 0   → cmd   0（限幅 15） → 下推极限（收拢）
 *   user 90  → cmd  90            → 竖直附近
 *   user 180 → cmd 180（限幅 165）→ 上推极限（竖直）
 * 以后再遇到镜像，同样只翻这两行（`link1_user_to_cmd / link1_cmd_to_user`），
 * 不要改关键帧数值。注意：篮内判定里的肩阈值以用户角表达——本次镜像后
 * 实测判定方向反了，已取关于 90° 对称的另一半区（肩 ≤ 100 = 压低半区，
 * 见 arm_action.cpp）。 */
constexpr float link1_user_to_cmd(float user_deg)
{
    return user_deg;
}

constexpr float link2_user_to_cmd(float user_deg)
{
    return user_deg - 120.0f;
}

/* ------------------------- 命令角 → 用户角 -------------------------
 * 位形判定（篮内/篮外）用读回的命令角反算用户角。
 * 底座的反变换已并入上方按车分套的块里（与正变换同处，保证同步翻转）；
 * 肩/肘的反变换如下，必须与正变换严格互逆：改一个必须同时改另一个。 */
constexpr float link1_cmd_to_user(float cmd_deg)
{
    return cmd_deg;
}

constexpr float link2_cmd_to_user(float cmd_deg)
{
    return cmd_deg + 120.0f;
}

/**
 * @brief 一个静止目标位形（关键帧）
 * @note  三个角度字段均为用户角（0~180）或 kKeep；
 *        gripper 为 GRIPPER_UNDEF 时表示本帧不动夹爪；
 *        power 是本帧要**通电**的关节（默认整臂），只有卸力恢复的"脱离"
 *        动作会用到部分给力——没通电的关节在本帧既不写也不推进。
 */
struct ArmKeyframe {
    float base_deg;            // 底座用户角
    float shoulder_deg;        // 肩用户角
    float elbow_deg;           // 肘用户角
    GripperState_t gripper;    // 夹爪状态（GRIPPER_UNDEF = 保持当前开合）
    uint32_t power = ARM_JOINT_ALL;   // 本帧通电的关节（ARM_JOINT_xxx 掩码）
};

/**
 * @brief 命名关键帧编号
 */
enum class KeyframeId : uint8_t {
    kBasketHigh = 0,   // 篮上高点：篮子上方过渡位
    kOuterHigh,        // 外侧高点：初始位附近过渡位
    kRetract,          // 收缩位：回收缩终点
    kBasketIn,         // 篮内点：转移动作的篮侧端点
    kPickupLow,        // 拾取低点：转移动作的外侧端点（出篮终点，夹爪闭合）
    kEscapeShoulder,   // 脱离1：只撑直大臂（肩），底座/肘/夹爪仍卸力
    kEscapeYaw,        // 脱离2：大臂保持，底座回中位
    /* —— 动作菜单的取放点（R1 菜单，2026/10/21 增）—— */
    kEnergyCoreShelf,     // 能量核心_物资架 (60,70,160,开)
    kEnergyUnitShelfLeft, // 能量单元_物资架下左 (60,60,45,开)
    kEnergyCorePlatform,  // 能量核心_高台 (30,75,30,开)
    kEnergyUnitPlatLeft,  // 能量单元_高台左 (30,75,20,开)
    kEnergyUnitPlatRight, // 能量单元_高台右 (90,75,20,开)
    kFrontLow,            // 前方低点 (0,45,60,夹爪不变)
    kOutsideLift,         // 篮外低姿起步过渡点 (50,40,50,夹爪不变)：
                          //   按 × 判篮外且大臂 <50° 时先到此再走原路线
    kCount
};

/**
 * @brief 取命名关键帧（纯数据，无副作用）
 * @param id 关键帧编号
 * @return 关键帧副本；id 非法时返回"三关节全保持、夹爪不变"的空帧，
 *         使调用方无需到处判空（非法编号属于编程错误，不该崩在车上）
 */
ArmKeyframe get_keyframe(KeyframeId id);

#endif //CRTC2026_ROBOTA_ARM_KEYFRAME_H
