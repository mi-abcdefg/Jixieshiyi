//
// 夹爪独立模块：末端执行器，经本模块点动（左摇杆X轴）。
//
// 层次：gripper 是比关节更基础的一层，包装 SERVO_GRIPPER；
// 角度域限幅由 servo_driver 配置表负责（当前 [0°,180°]，90° 为中位脉宽）。
//
// 开/合两档（命令角 60/120，占位值）、三态状态机（开/合/未定义）与
// ○ 键两档切换随动作序列一并移除，待动作模块重建时恢复——
// 数据与教训见 docs/机械臂动作模块重建交接.md。
//

#ifndef CRTC2026_ROBOTA_GRIPPER_H
#define CRTC2026_ROBOTA_GRIPPER_H

#pragma once

/* ------------------------------ 夹爪参数 ------------------------------ */
#define GRIPPER_MAX_DPS     60.0f   // 点动角速度 [°/s]（满杆）

#ifdef __cplusplus
extern "C" {
#endif

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
