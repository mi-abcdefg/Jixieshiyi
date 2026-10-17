//
// 机械臂任务控制（决策层）：按键事件 → 动作启动/取消
//
// 移植自 CRTC_RobotArm 工程 robot_control.c/h，C++ 化，并加入模式门控。
//
// 门控规则（按键有限，R2 切换操作对象后的约定）：
//   - 事件每周期都取走（取出即清零）。即使当前不在机械臂模式，也必须
//     消费掉，否则底盘模式下按过的键会在切回机械臂模式时一次性触发。
//   - SELECT（取消）在两种模式下都生效：它是"停止"语义，正在跑的动作
//     不能因为切了模式就失去停止手段。
//   - △/○/×/□（抓取/放置）只在机械臂模式生效。
//
// 只依赖 gamepad、arm_action、control_mode；不感知 PS2 和摇杆
// （摇杆走 arm_manual，与按键决策互不相干）。
//

#ifndef CRTC2026_ROBOTA_ROBOT_CONTROL_H
#define CRTC2026_ROBOTA_ROBOT_CONTROL_H

#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 处理一批新按下的按键事件；须被周期性调用（建议 ≤20ms） */
void robot_control_update(uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif //CRTC2026_ROBOTA_ROBOT_CONTROL_H
