//
// 机械臂关键帧层：用户角约定、用户角→命令角换算、命名关键帧表
//
// 本层是**纯数据**层——整个机械臂唯一存放具体角度的地方：标定角度只改
// arm_keyframe.cpp 的关键帧表，动作引擎只认 KeyframeId，不认角度。
//
// 用户角约定（0~180，与舵机安装方向解耦）：
//   底座 base      user 0 = 工作侧端（cmd 270）  user 180 = 篮子侧端（cmd 90）
//   肩   shoulder  user 0 = 上推极限           user 180 = 下推极限（收拢）
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
 * 底座方向标定（2026/10/21 实车复核后**已镜像一次**）：原式 cmd = 90 + user
 * 使 user 0 落在篮子上，与"user 0 = 工作侧端"的约定相反；实测确认后翻转为
 * cmd = 270 − user，映射关系为
 *   user 0   → cmd 270 → 2500µs → 工作侧端
 *   user 90  → cmd 180 → 1500µs → 初始方位（臂朝车头）
 *   user 180 → cmd  90 →  500µs → 篮子侧端
 * 以后再遇到镜像，同样只翻这两行（`base_user_to_cmd / base_cmd_to_user`），
 * 不要改关键帧数值；篮内判定阈值以用户角表达（见 arm_action.cpp），
 * 约定本身不变（篮子侧 = 用户角大的一端），因此不需要跟着动。 */
constexpr float base_user_to_cmd(float user_deg)
{
    return 270.0f - user_deg;
}

constexpr float link1_user_to_cmd(float user_deg)
{
    return 180.0f - user_deg;
}

constexpr float link2_user_to_cmd(float user_deg)
{
    return user_deg - 120.0f;
}

/* ------------------------- 命令角 → 用户角 -------------------------
 * 位形判定（篮内/篮外）用读回的命令角反算用户角。
 * 必须与上面的正变换严格互为逆：改一个必须同时改另一个。 */
constexpr float base_cmd_to_user(float cmd_deg)
{
    return 270.0f - cmd_deg;
}

constexpr float link1_cmd_to_user(float cmd_deg)
{
    return 180.0f - cmd_deg;
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
