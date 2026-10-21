//
// 机械臂按键输入层：PS2 位图 → 边沿事件 → 按键决策
//
// 本层是机械臂按键语义的**唯一**归属地（验收项 E1~E5 的绑定都在这里）：
//
//   正常模式（默认）
//     ○        夹爪开/合两档切换（E1；两档角度见 gripper.h）
//     △        回收缩：任意位形经过渡点收到收缩位，夹爪闭合（E2）
//     ×        篮内外转移：走向按起始位形自动判定（E3）
//     L2       卸力 ⇄ 恢复；恢复时按"大臂（肩）先撑直 → 再大臂+底座"的顺序逐路给力（E4）
//     L1       点动快/慢两档循环（只影响手动点动，不影响动作）
//     十字键↑↓ 末端持续上升 / 下降（长按；柱坐标点动，走 arm_endpoint）
//     十字键←→ 末端持续向前(远离底座) / 向后(靠近底座)
//     SELECT   取消当前动作（两种操作对象模式下都生效，"停止"语义）
//
//   动作选择菜单（机械臂模式下按 R1 进入，再按 R1 退出）
//     左侧↑↓←→ + 右侧△○×□ 共 8 个键各对应一个动作槽位；
//     槽位内容见 arm_button.cpp 的 kMenuSlots[]（**当前全部占位**，待新增动作）。
//     菜单态下摇杆点动、夹爪切换、△/× 正常绑定、十字键末端点动一律暂不响应，
//     退出菜单（R1）后恢复。L2（卸力）与 SELECT（停止）在菜单态仍然有效——安全相关。
//
// 注：回收缩原绑定在十字键↑，2026/10/21 改到 △——底盘模式下 ↑ 要留给定距
// 平移，两个模式共用同一颗键容易误触发。十字键在机械臂模式下现在被末端点动占用。
//
// 通用打断（E5）：动作运行中，推杆越过打断阈值 / SELECT / 再次动作键
// 都会取消动作并停在当前位置。打断阈值显著大于点动死区
// （ARM_MANUAL_INTERRUPT_DEADZONE = 0.15 vs ARM_MANUAL_DEADZONE = 0.02），
// 否则摇杆静置漂移会把刚启动的动作在第一帧误杀（交接文档踩坑 13）。
//
// 分工：
//   - feed / take_events 是纯事件层：只认抽象位图，不 include 任何 PS2 头文件，
//     PS2 原始位 → 本层位图的映射在 arm_task（编排层）——换遥控器只改那里；
//   - apply 是决策层：按键 + 摇杆 + 模式 + 力矩状态 → arm_action / gripper /
//     arm_manual 的调用；
//   - 本层不碰关节角，也不做周期推进。
//
// 事件是锁存的（取出才清零），apply 每周期都取走，避免模式切换后残留事件
// 一次性触发。
//

#ifndef CRTC2026_ROBOTA_ARM_BUTTON_H
#define CRTC2026_ROBOTA_ARM_BUTTON_H

#pragma once
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief 机械臂按键位图（软件内部统一格式，不代表 PS2 接收器原始位序）
 */
enum ArmButtonBit : uint16_t {
    kArmBtnSelect = 1u << 0,    // SELECT：取消动作（两种模式都生效）
    kArmBtnPadUp = 1u << 1,     // 十字键↑：末端上升（长按）/ 菜单槽位
    kArmBtnCross = 1u << 2,     // ×：篮内外转移 / 菜单槽位
    kArmBtnCircle = 1u << 3,    // ○：夹爪两档切换 / 菜单槽位
    kArmBtnL1 = 1u << 4,        // L1：点动两档调速
    kArmBtnL2 = 1u << 5,        // L2：卸力 ⇄ 恢复（逐路给力）
    kArmBtnPadLeft = 1u << 6,   // 十字键←：末端向前（长按）/ 菜单槽位
    kArmBtnTriangle = 1u << 7,  // △：回收缩 / 菜单槽位
    kArmBtnSquare = 1u << 8,    // □：菜单槽位（正常模式未绑定动作）
    kArmBtnPadDown = 1u << 9,   // 十字键↓：末端下降（长按）/ 菜单槽位
    kArmBtnPadRight = 1u << 10, // 十字键→：末端向后（长按）/ 菜单槽位
    kArmBtnR1 = 1u << 11        // R1：机械臂模式下切换"正常 ⇄ 动作选择菜单"
};

/**
 * @brief 初始化按键输入层（清空边沿与事件锁存）
 */
void arm_button_init(void);

/**
 * @brief 喂入一份当前按键状态（每周期调用一次，断连时传全 0 也要喂）
 * @param buttons_pressed 当前按下的按键位图（ArmButtonBit 的按位或）
 * @param connected       遥控链路是否正常
 */
void arm_button_feed(uint16_t buttons_pressed, bool connected);

/**
 * @brief 取出尚未处理的"新按下"事件；取出后清空
 * @note  由 apply 每周期消费；单独暴露供台架/单元测试使用
 */
uint16_t arm_button_take_events(void);

/**
 * @brief 喂入摇杆归一化量（本周期），供打断阈值判定
 * @param base_norm     底座轴 -1..1
 * @param shoulder_norm 肩轴   -1..1
 * @param elbow_norm    肘轴   -1..1
 * @param gripper_norm  夹爪轴 -1..1
 * @note  与 arm_manual_update 的入参同源（arm_task 归一化后同时喂两边）
 */
void arm_button_feed_sticks(float base_norm, float shoulder_norm,
                            float elbow_norm, float gripper_norm);

/**
 * @brief 处理按键决策（每周期调用一次，建议在喂按键/摇杆之后）
 *
 * 除按键事件外，本函数还负责**长按型**输入：十字键的末端点动（内部调
 * arm_endpoint_jog），所以必须每周期调用，不能只在有事件时调。
 *
 * @param now_ms 当前系统时间 [ms]
 */
void arm_button_apply(uint32_t now_ms);

/**
 * @brief 是否处于"动作选择菜单"（机械臂模式下按 R1 进入）
 */
bool arm_button_in_menu(void);

/**
 * @brief 本周期末端点动是否在用十字键（arm_task 据此让摇杆点动让位）
 *
 * 两条手动通道都会写三关节，同一周期只能有一个主人：十字键末端点动
 * 激活时，摇杆点动必须让位，否则两者互相覆盖、关节发抖。
 */
bool arm_button_is_endpoint_jog_active(void);

#endif //CRTC2026_ROBOTA_ARM_BUTTON_H
