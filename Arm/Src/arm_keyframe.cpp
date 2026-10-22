//
// 机械臂关键帧表实现（纯数据，无副作用）
//
// 本文件是全工程**唯一**的机械臂角度数据源：
//   - 关键帧一律以"用户角"（0~180）书写，换算宏在 arm_keyframe.h；
//   - 夹爪以状态（开/合/不变）参与，不用角度；
//   - 标定后改数值即可，动作引擎与按键层都不需要动。
//
// 表的下标必须与 KeyframeId 的声明顺序严格一致（末尾有 static_assert 兜长度）。
//

#include "arm_keyframe.h"

/*
 * 命名关键帧（底座/肩/肘 = 用户角；夹爪 = 开/合/不变）
 *
 * 篮上高点 (180, 90, 90)：底座转到篮子一侧端部，肩竖直、肘 90——悬在
 *   篮子上方的过渡位，转移与回收缩都经过它。
 * 外侧高点 (90, 100, 80)：底座回初始方位（臂朝车头），肩 100、肘 80，
 *   与上电初始位形相近的外侧过渡位。
 * 收缩位 (0, 165, 0, 合)：底座收到工作侧端部、肩抬起、肘收拢，夹爪闭合——
 *   整车运输/避让的最小包络位形。
 * 篮内点 (175, 70, 25, 开)：转移动作的篮侧端点。
 * 出篮终点 (90, 75, 40, 合)：转移动作的出篮路径终点（2026/10/21 起夹爪为闭合态，
 *   只是"到位后把夹爪合上"这个终端状态；路径途中不动夹爪，免得在篮子里把方块夹住带出来）。
 * 脱离两帧（power 掩码由窄到宽，这是"卸力恢复"的给力顺序，
 * 2026/10/21 实车实测确认第一段给力的是**大臂（肩）**）：
 *   先只给大臂通电、把它撑直到 90（底座/肘/夹爪仍是软的，臂能自己让位）；
 *   再让大臂保持 90、底座单独回中位；肘（小臂）与夹爪在这一步仍是软的，
 *   动作收尾时（arm_action 的 reset_state）才一起给力。
 * 术语：大臂 = 肩 = SERVO_LINK_1 = 第一段连杆（11cm）；
 *       小臂 = 肘 = SERVO_LINK_2 = 第二段连杆（8.5cm）。
 *
 * kKeep 字段表示该关节保持当前命令角；power 字段默认整臂通电，见 arm_keyframe.h。
 */
static const ArmKeyframe kKeyframes[] = {
    /* kBasketHigh  篮上高点 */
    { 180.0f,        90.0f,          90.0f,          GRIPPER_UNDEF },
    /* kOuterHigh   外侧高点 */
    { 90.0f,         80.0f,         100.0f,          GRIPPER_UNDEF },
    /* kRetract     收缩位 */
    { 0.0f,          172.0f,         0.0f,           GRIPPER_UNDEF },
    /* kBasketIn    篮内点 */
    { 175.0f,        70.0f,          50.0f,          GRIPPER_OPEN  },
    /* kPickupLow   出篮终点（到位合爪） */
    { 90.0f,         75.0f,          40.0f,          GRIPPER_CLOSE },
    /* kEscapeShoulder 脱离1：只撑直大臂（其余关节仍卸力） */
    { kKeep,         90.0f,          kKeep,          GRIPPER_UNDEF,
      ARM_JOINT_SHOULDER },
    /* kEscapeYaw   脱离2：大臂保持 90，底座回中位（肘/夹爪仍卸力） */
    { 90.0f,         kKeep,          kKeep,          GRIPPER_UNDEF,
      ARM_JOINT_SHOULDER | ARM_JOINT_BASE_Z },
    /* —— 动作菜单的取放点（R1 菜单，2026/10/21 增；角度为操作手给的占位值，
     *    实车微调只改这里的数字）—— */
    /* kEnergyCoreShelf      能量核心_物资架 */
    { 60.0f,         70.0f,          160.0f,         GRIPPER_OPEN  },
    /* kEnergyUnitShelfLeft  能量单元_物资架下左 */
    { 60.0f,         60.0f,          45.0f,          GRIPPER_OPEN  },
    /* kEnergyCorePlatform   能量核心_高台 */
    { 30.0f,         75.0f,          30.0f,          GRIPPER_OPEN  },
    /* kEnergyUnitPlatLeft   能量单元_高台左 */
    { 30.0f,         75.0f,          20.0f,          GRIPPER_OPEN  },
    /* kEnergyUnitPlatRight  能量单元_高台右 */
    { 90.0f,         75.0f,          20.0f,          GRIPPER_OPEN  },
    /* kFrontLow             前方低点（夹爪保持当前开合不动） */
    { 0.0f,          45.0f,          60.0f,          GRIPPER_UNDEF },
    /* kOutsideLift          篮外低姿起步过渡点（× 键：大臂 <50° 时先到此） */
    { 50.0f,         40.0f,          50.0f,          GRIPPER_UNDEF },
};

static_assert(sizeof(kKeyframes) / sizeof(kKeyframes[0]) ==
              static_cast<unsigned long>(KeyframeId::kCount),
              "关键帧表长度必须与 KeyframeId::kCount 一致（新增帧时同步两处）");

ArmKeyframe get_keyframe(KeyframeId id)
{
    /* 编号非法属于编程错误：返回空帧（三关节全保持、夹爪不变）比崩在车上好，
     * 引擎会把它当成"原地不动"的帧，很容易在台架上看出来。 */
    if (static_cast<uint8_t>(id) >= static_cast<uint8_t>(KeyframeId::kCount)) {
        return ArmKeyframe{ kKeep, kKeep, kKeep, GRIPPER_UNDEF };
    }
    return kKeyframes[static_cast<uint8_t>(id)];
}
