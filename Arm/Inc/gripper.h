//
// 夹爪独立模块：夹爪是末端执行器，按"开 / 合 / 未定义"三态管理，
// 不作为普通舵机暴露给上层（关键帧以状态引用夹爪，点动走本模块）。
//
// 层次：gripper 是比关节更基础的一层，包装 SERVO_GRIPPER；
// 角度域限幅仍由 servo_driver 配置表负责（[15°,165°]，两端各留 15°）。
//

#ifndef CRTC2026_ROBOTA_GRIPPER_H
#define CRTC2026_ROBOTA_GRIPPER_H

#pragma once
#include <stdint.h>
#include <stdbool.h>

/* ------------------------- 夹爪参数（占位待标定） ------------------------- */
#define GRIPPER_OPEN_DEG    60.0f   // 张开档命令角
#define GRIPPER_CLOSE_DEG   120.0f  // 闭合档命令角
#define GRIPPER_MAX_DPS     60.0f   // 点动角速度

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 夹爪状态
 */
typedef enum {
    GRIPPER_UNDEF = 0,   // 未定义：上电初始，或点动后离开命名位置
    GRIPPER_OPEN,        // 张开档
    GRIPPER_CLOSE        // 闭合档
} GripperState_t;

/**
 * @brief 初始化（状态置为未定义；舵机角度由 servo_init 的初始角决定）
 */
void gripper_init(void);

/**
 * @brief 切换到指定档位（立即下发目标角，舵机以自身最大速度摆过去）
 * @return false 表示状态非法或舵机写入被拒绝（如卸力中）
 */
bool gripper_set(GripperState_t state);

/**
 * @brief 依当前角度切到相反档位（角度偏闭合档则张开，反之闭合）
 */
bool gripper_toggle(void);

/**
 * @brief 查询当前状态（点动后变为未定义）
 */
GripperState_t gripper_get_state(void);

/**
 * @brief 点动（正 = 命令角增大方向）；一旦移动，状态转为未定义
 * @param rate_norm 归一化速率 -1..1（调用方先做死区过滤）
 * @param dt_s      本周期时长 [s]
 */
void gripper_jog(float rate_norm, float dt_s);

#ifdef __cplusplus
}
#endif

#endif //CRTC2026_ROBOTA_GRIPPER_H
