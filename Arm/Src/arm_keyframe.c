//
// 机械臂关键帧表实现（纯数据）
//

#include "arm_keyframe.h"

/*
 * 命名关键帧（用户角；夹爪为状态）
 *
 * 篮上高点 (180, 90, 90)：底座转到用户角 180（篮子一侧端部），肩竖直、
 *   肘绝对仰角 60°，悬于篮子上方的过渡位。
 * 外侧高点 (90, 100, 80)：底座回初始方位，肩 100、肘 80——绝对仰角 60°，
 *   与上电初始位形相近的外侧过渡位。
 * 最小收缩位 (0, 165, 0, 合)：底座收到用户角 0（远离篮子一端）、肩抬起
 *   165、肘收拢到 0，夹爪闭合。
 */
static const ArmKeyframe_t keyframes[ARM_KF_ID_COUNT] = {
    [ARM_KF_ID_BASKET_HIGH] = { 180.0f, 90.0f,  90.0f, GRIPPER_UNDEF },
    [ARM_KF_ID_OUTER_HIGH]  = { 90.0f,  100.0f, 80.0f, GRIPPER_UNDEF },
    [ARM_KF_ID_RETRACT]     = { 0.0f,   165.0f, 0.0f,  GRIPPER_CLOSE },
};

const ArmKeyframe_t *arm_keyframe_get(ArmKeyframeId_t id)
{
    if ((uint32_t)id >= (uint32_t)ARM_KF_ID_COUNT)
    {
        return 0;
    }
    return &keyframes[id];
}
