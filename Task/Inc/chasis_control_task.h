//
// Created by MI on 2026/9/13.
//

#ifndef CRTC2026_ROBOTA_CHASIS_CONTROL_TASK_H
#define CRTC2026_ROBOTA_CHASIS_CONTROL_TASK_H

#pragma once
#include "TaskBase.h"
#include "mecanum_kinematic.h"
#include "motor_pwm_driver.h"
#include "ps2_decode.h"

/* ---------------------------- 速度模式 ----------------------------
 * 点按 L1 在三档间循环：快速（满速）→ 中速（比例 CHASIS_SLOW_MODE_SCALE）
 * → 固定慢速（方向随摇杆、速度恒定，与杆量无关）→ 快速……上电默认快速。 */
#define CHASIS_SLOW_MODE_SCALE    0.8f

/* 固定慢速档：平移/旋转速度 = 满量程 × 下列比例，摇杆只提供方向。
 *
 * 【2026/10/21 实测三轮】0.15/0.25 起不来（≈21% 占空比）；抬到 0.5/0.5 仍起不来
 * （50%，加上旧的前馈也只到 56%）。按要求改为满量程的 0.7：
 *   平移 0.7 × 2.0 m/s = 1.4 m/s、旋转 0.7 × 满量程角速度 —— 即 70% 占空比。
 * 阻力前馈现在是**起步下限**而不是叠加（见 mecanum_kinematic.h），
 * 所以 70% 就是这一档的实际占空比，不会被前馈顶到 100%。 */
#define CHASIS_FIXED_SLOW_SPEED_RATIO   0.7f   // 平移 0.7 × 2.0 = 1.4 m/s（70% 占空比）
#define CHASIS_FIXED_SLOW_OMEGA_RATIO   0.7f   // 旋转 0.7 × 满量程角速度

/* ------------------------ 原地旋转方向翻转 ------------------------
 * 推右摇杆的旋转方向与整车实际转向相反时置 1。
 * 2026/10/21 按要求把底盘顺/逆时针调换：改为 0（关闭取反）。
 * 只取反底盘旋转指令 omega，不影响 vx/vy，也不影响机械臂的底座自转。 */
#define CHASIS_OMEGA_INVERT       0

/* ------------------------ 十字键平移（虚拟摇杆） ------------------------
 * 仅底盘模式。语义 = **虚拟摇杆**：按住 = 该方向以固定比例平移，速度约定
 * 与摇杆完全同源——比例 × 满量程（MECANUM_MAX_SPEED），经同一条解算管线
 * （归一化 / L1 档位缩放 / 起步前馈），不另设一套绝对速度。
 *   上=前 下=后 左=左移 右=右移（正负号与左摇杆推杆方向严格同号），
 *   两键同按 = 斜向。平移期间不响应旋转。
 *
 * 点按（按压 < CHASIS_PAD_TAP_MS）补完一步（= 速度 × CHASIS_PAD_STEP_MS，
 * 当前约 12cm），长按按住期间持续移动、松手即停。
 *
 * 【2026/10/21 重写】旧实现用绝对速度 1.0 m/s：满量程减半后它变成 100%
 * 占空比满速突跳，且不受档位约束，与全工程速度约定脱节。现改为满量程
 * 比例 0.5（= 50% 占空比）：L1 中速档自动 ×0.8；固定慢速档不影响十字键
 * （它本身就是低速微调用途）。
 * 注意：摩擦前馈（MECANUM_FRICTION_COMP_MPS）当前为 0，若实车起步占空比
 * 高于 50%，十字键会推不动——属"前馈归零待实测"的预期现象，标定 COMP 后
 * 自然恢复，与摇杆小油门是同一个问题。 */
#define CHASIS_PAD_SPEED_RATIO    0.5f   // 平移速度 = 0.5 × 满量程（50% 占空比）
#define CHASIS_PAD_TAP_MS         100U   // 短于此算"点按"（= 补完整一步）
#define CHASIS_PAD_STEP_MS        240U   // 0.5 × 满量程 × 240ms ≈ 12cm/步

#ifdef __cplusplus
class ChasisControlTask : public TaskBase {
public:
    void run() override;
};
#endif

#ifdef __cplusplus
extern "C" {
#endif

void chasis_control_task_init();

#ifdef __cplusplus
}
#endif

#endif //CRTC2026_ROBOTA_CHASIS_CONTROL_TASK_H