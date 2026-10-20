//
// 机械臂任务控制实现（移植自 CRTC_RobotArm 工程 robot_control.c + 模式门控）
//

#include "robot_control.h"

#include "arm_action.h"
#include "arm_control.h"
#include "arm_manual.h"
#include "gripper.h"
#include "gamepad.h"
#include "control_mode.h"

void robot_control_update(uint32_t now_ms)
{
    if (!gamepad_is_connected()) {
        return;
    }

    uint16_t events = gamepad_take_pressed_events();

    if (events == 0U) {
        return;
    }

    /* SELECT：取消一切自动运动（动作序列含脱离伸展），两种模式均生效 */
    if ((events & GAMEPAD_SELECT) != 0U) {
        arm_action_cancel();
        return;
    }

    /* 抓取/放置等按键只在机械臂模式生效。
     * 事件已取出并丢弃，不会残留到切回机械臂模式之后。 */
    if (control_mode_get() != CONTROL_MODE_ARM) {
        return;
    }

    /* L2：卸力 ⇄ 恢复 + 自动脱离。
     * 卸力通常发生在臂卡住时，因此恢复力矩后不原地待着，而是启动
     * "脱离"动作（大臂小臂升到行程中点 → 底座转到行程中点），远离
     * 卡住区域。动作运行中按 L2 = 停止并保持当前位置。 */
    if ((events & GAMEPAD_L2) != 0U) {
        if (arm_action_get_state() == ARM_ACTION_STATE_RUNNING) {
            arm_action_cancel();   // 途中停止自动运动，保持当前位置
            return;
        }
        if (arm_get_torque_enabled()) {
            arm_set_torque_enabled(false);              // 卸力（无脉冲输出）
        } else {
            arm_set_torque_enabled(true);               // 恢复到卸力前姿态
            (void)arm_action_start(ARM_ACTION_ESCAPE, now_ms);
        }
        return;
    }

    /* 动作运行中：调速键仍可用（影响点动），其余按键不响应 */
    const bool action_running =
        (arm_action_get_state() == ARM_ACTION_STATE_RUNNING);

    if (!action_running) {
        /* 卸力状态下其余按键全部忽略：动作启动/夹爪切换都会因舵机写入
         * 被拒绝而无效，提前拦截避免状态被悄悄改变。 */
        if (!arm_get_torque_enabled()) {
            return;
        }

        /* L1：点动调速两档循环（快 ⇄ 慢），仅作用于手动点动 */
        if ((events & GAMEPAD_L1) != 0U) {
            arm_manual_cycle_speed_level();
            return;
        }
    }

    /* 圆圈键：夹爪在张开/闭合两档间快速切换（独立夹爪模块）。
     * 动作运行中先取消——动作关键帧会覆盖夹爪指令。 */
    if ((events & GAMEPAD_CIRCLE) != 0U) {
        if (action_running) {
            arm_action_cancel();
        }
        (void)gripper_toggle();
        return;
    }

    /* 动作键：运行中/出错时不响应（事件已取出，不排队） */
    if (action_running ||
        arm_action_get_state() == ARM_ACTION_STATE_ERROR) {
        return;
    }

    ArmActionId_t action_id;

    /* 按判断顺序决定同时按键时的优先级（○ 已改配夹爪切换） */
    if ((events & GAMEPAD_PAD_UP) != 0U) {
        action_id = ARM_ACTION_RETRACT;
    } else if ((events & GAMEPAD_CROSS) != 0U) {
        action_id = ARM_ACTION_TRANSIT;
    } else if ((events & GAMEPAD_TRIANGLE) != 0U) {
        action_id = ARM_ACTION_GRAB_MODE_1;
    } else if ((events & GAMEPAD_SQUARE) != 0U) {
        action_id = ARM_ACTION_GRAB_MODE_3;
    } else {
        return;
    }

    (void)arm_action_start(action_id, now_ms);
}
