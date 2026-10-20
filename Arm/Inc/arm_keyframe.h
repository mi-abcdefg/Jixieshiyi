//
// 机械臂关键帧层：用户角约定、用户角→命令角换算、命名关键帧表
//
// 用户角约定（0~180，摇杆基准，与标定的命令角域解耦）：
//   每个关节 0 = 点动时向"负方向"（摇杆向左或向下）推杆能到达的极限，
//   180 = 反方向极限。关键帧一律以用户角存储，换算只在关键帧层发生。
//
// 换算（由点动方向推导；若某关节实测方向镜像，改对应宏一行即可）：
//   舵机1 底座：cmd = 270 − user   （左推极限=方位角270，右推极限=90）
//   舵机2 肩：  cmd = user         （下推极限=0，上推极限=180，即仰角）
//   舵机3 肘：  cmd = user − 120   （下推极限=−120，上推极限=+60，相对角）
//   舵机4 爪：  以状态描述（开/合/不定义），不使用角度，见 gripper 模块。
//
// 关键帧字段可取 ARM_KF_KEEP（255）表示"该关节保持当前值不动"。
//

#ifndef CRTC2026_ROBOTA_ARM_KEYFRAME_H
#define CRTC2026_ROBOTA_ARM_KEYFRAME_H

#pragma once
#include <stdint.h>
#include "gripper.h"    // GripperState_t（夹爪以状态参与关键帧）

/* ------------------------- 用户角 → 命令角 -------------------------
 * 各关节的"用户角 0 在哪一端"以实机实测为准（2026/10/20 × 键实测修订）：
 *   舵机1 底座：实测镜像——左推极限 = user 180（cmd 270），
 *               右推极限 = user 0（cmd 90）
 *   舵机2 肩：  实测与点动约定镜像——下推极限 = user 180（cmd 0），
 *               上推极限 = user 0（cmd 180）
 *   舵机3 肘：  下推极限 = user 0（cmd −120），上推极限 = user 180（+60）
 *   舵机4 爪：  以状态描述（开/合/不定义），不使用角度。 */
#define ARM_UA_BASE_TO_CMD(u)    (90.0f + (u))
#define ARM_UA_LINK1_TO_CMD(u)   (180.0f - (u))
#define ARM_UA_LINK2_TO_CMD(u)   ((u) - 120.0f)

/* 命令角 → 用户角（篮内/篮外区判断用） */
#define ARM_UA_BASE_FROM_CMD(c)   ((c) - 90.0f)
#define ARM_UA_LINK1_FROM_CMD(c)  (180.0f - (c))

/* 关键帧字段哨兵：保持当前值 */
#define ARM_KF_KEEP              255.0f

/**
 * @brief 关键帧：一个静止目标位形
 * @note  base/link1/link2 为用户角（0~180 或 ARM_KF_KEEP）；
 *        gripper 为状态（GRIPPER_UNDEF = 保持当前开合不动）
 */
typedef struct {
    float base_deg;
    float link1_deg;
    float link2_deg;
    GripperState_t gripper;
} ArmKeyframe_t;

/**
 * @brief 命名关键帧编号（按需扩充；新增时同步 keyframes 表）
 */
typedef enum {
    ARM_KF_ID_BASKET_HIGH = 0,   // 篮上高点：篮子上方的过渡位
    ARM_KF_ID_OUTER_HIGH,        // 外侧高点：接近初始位的过渡位
    ARM_KF_ID_RETRACT,           // 最小收缩位 (0,165,0,合)
    ARM_KF_ID_BASKET_IN,         // 篮内点 (175,70,25,开)：篮内作业位
    ARM_KF_ID_PICKUP_LOW,        // 拾取低点 (90,75,40,开)：篮外低位拾取位
    ARM_KF_ID_ESCAPE_LINKS,      // 脱离阶段1：大臂小臂升到中点（底座保持）
    ARM_KF_ID_ESCAPE_BASE,       // 脱离阶段2：底座转到中点（连杆保持）
    ARM_KF_ID_HOME,              // HOME 位形 (90,90,90)
    ARM_KF_ID_HOME_OPEN,         // HOME 位形 + 夹爪张开（占位动作用）
    ARM_KF_ID_HOME_CLOSE,        // HOME 位形 + 夹爪闭合（占位动作用）
    ARM_KF_ID_COUNT
} ArmKeyframeId_t;

/**
 * @brief 取命名关键帧（返回 NULL 表示编号非法）
 */
const ArmKeyframe_t *arm_keyframe_get(ArmKeyframeId_t id);

#endif //CRTC2026_ROBOTA_ARM_KEYFRAME_H
