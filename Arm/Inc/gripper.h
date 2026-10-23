//
// 夹爪独立模块：末端执行器，按"开 / 合 / 未定义"三态管理
//
// 层次：gripper 是比关节更基础的一层，包装 SERVO_GRIPPER；角度域限幅由
// servo_driver 配置表负责（当前 [0°,180°] 全行程，90° 为中位脉宽），
// 夹爪不作为普通舵机暴露给上层——关键帧以"状态"引用它，点动走本模块。
//
// 两条使用路径：
//   1. 两档切换（○ 键 / 动作关键帧）：gripper_set / gripper_toggle，
//      目标角是本文件的 GRIPPER_OPEN_DEG / GRIPPER_CLOSE_DEG；
//   2. 连续点动（左摇杆 X）：gripper_jog，一旦移动状态转为"未定义"。
//
// 标定：点动找到真实张开/闭合角后改下面两个宏；若开/合方向颠倒，
// 翻转 gripper_toggle 里的中位判断（交接文档 §10）。
//

#ifndef CRTC2026_ROBOTA_GRIPPER_H
#define CRTC2026_ROBOTA_GRIPPER_H

#pragma once
#include <stdint.h>
#include <stdbool.h>

/* ------------------------- 夹爪参数（占位待标定） ------------------------- */
#define GRIPPER_OPEN_DEG    60.0f    // 张开档命令角
#define GRIPPER_CLOSE_DEG   120.0f   // 闭合档命令角（2026/10/21 实测：90 偏松，
                                     // 再紧 30° 回到 120）
#define GRIPPER_SHAKE_CLOSE_DEG 108.0f  // 抖动闭合档：比闭合档留一点缝（不完全闭）；
                                        // 随闭合档调整后已接近全开侧，待一并标定
#define GRIPPER_MAX_DPS     60.0f    // 点动角速度 [°/s]（满杆）

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 夹爪状态
 */
typedef enum {
    GRIPPER_UNDEF = 0,   // 未定义：上电初始，或点动后离开命名档位
    GRIPPER_OPEN,        // 张开档
    GRIPPER_CLOSE        // 闭合档
} GripperState_t;

/**
 * @brief 初始化（状态置为未定义；实际角度由 servo_init 的初始角决定）
 */
void gripper_init(void);

/**
 * @brief 切到指定档位（立即下发目标角，舵机以自身最大速度摆过去）
 * @return false 表示状态非法（GRIPPER_UNDEF）或舵机写入被拒绝（如卸力中）
 */
bool gripper_set(GripperState_t state);

/**
 * @brief 依当前角度切到相反档位（角度偏闭合档则张开，反之闭合）
 */
bool gripper_toggle(void);

/**
 * @brief 下发任意命令角（两档之外的中间位置，如抖动的"不完全闭"档）
 * @note  下发成功后状态转为"未定义"；角度会按配置表量程限幅
 */
bool gripper_set_angle(float cmd_deg);

/**
 * @brief 查询当前状态（点动后变为未定义）
 */
GripperState_t gripper_get_state(void);

/**
 * @brief 点动（正 = 命令角增大方向；开/合的物理方向实物标定）
 * @param rate_norm 归一化速率 -1..1（调用方先做死区过滤）
 * @param dt_s      本周期时长 [s]
 */
void gripper_jog(float rate_norm, float dt_s);

#ifdef __cplusplus
}
#endif

#endif //CRTC2026_ROBOTA_GRIPPER_H
