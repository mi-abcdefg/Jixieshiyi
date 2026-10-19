//
// 机械臂任务控制实现（移植自 CRTC_RobotArm 工程 robot_control.c + 模式门控）
//

#include "robot_control.h"

#include "arm_action.h"
#include "arm_control.h"
#include "arm_manual.h"
#include "gamepad.h"
#include "control_mode.h"

/* 夹爪当前档位记忆（true = 处于闭合档） */
static bool gripper_at_close = false;

void robot_control_update(uint32_t now_ms)
{
    if (!gamepad_is_connected()) {
        return;
    }

    uint16_t events = gamepad_take_pressed_events();

    if (events == 0U) {
        return;
    }

    /* 取消键优先于同一批次的其他按键，且不受模式限制（停止语义） */
    if ((events & GAMEPAD_SELECT) != 0U) {
        arm_action_cancel();
        return;
    }

    /* 抓取/放置动作键只在机械臂模式生效。
     * 事件已取出并丢弃，不会残留到切回机械臂模式之后。 */
    if (control_mode_get() != CONTROL_MODE_ARM) {
        return;
    }

    /* 十字键"左"：夹爪在张开/闭合两档间快速切换。
     * 若动作正在执行先取消——动作每拍整姿下发，会把这里的夹爪指令覆盖掉；
     * 取消与手动推杆打断是同一优先级策略。 */
    if ((events & GAMEPAD_PAD_LEFT) != 0U) {
        if (arm_action_get_state() == ARM_ACTION_STATE_RUNNING) {
            arm_action_cancel();
        }
        gripper_at_close = !gripper_at_close;
        (void)arm_set_gripper(gripper_at_close ? ARM_GRIPPER_CLOSE_DEG
                                               : ARM_GRIPPER_OPEN_DEG);
        return;
    }

    /* L2：机械臂卸力 ⇄ 恢复。
     * 卸力时若动作正在执行先取消（动作每拍写舵机会被卸力保护拒绝，
     * 会让状态机误入 ERROR）；恢复后臂自动回到卸力前姿态。 */
    if ((events & GAMEPAD_L2) != 0U) {
        if (arm_action_get_state() == ARM_ACTION_STATE_RUNNING) {
            arm_action_cancel();
        }
        arm_set_torque_enabled(!arm_get_torque_enabled());
        return;
    }

    /* 卸力状态下其余按键全部忽略：动作启动/夹爪切换都会因舵机写入
     * 被拒绝而无效，提前拦截可避免夹爪档位记忆等状态被悄悄改变。 */
    if (!arm_get_torque_enabled()) {
        return;
    }

    /* L1：点动调速两档循环（快 ⇄ 慢），仅作用于手动点动，不影响动作序列 */
    if ((events & GAMEPAD_L1) != 0U) {
        arm_manual_cycle_speed_level();
        return;
    }

    /*
     * 当前先不允许普通动作抢占。
     * 本次事件已取出，因此不会排队等待。
     */
    if (arm_action_get_state() == ARM_ACTION_STATE_RUNNING) {
        return;
    }

    /* 出错后不自动执行下一套动作 */
    if (arm_action_get_state() == ARM_ACTION_STATE_ERROR) {
        return;
    }

    ArmActionId_t action_id;

    /* 按判断顺序决定同时按键时的优先级 */
    if ((events & GAMEPAD_CROSS) != 0U) {
        action_id = ARM_ACTION_PLACE;
    } else if ((events & GAMEPAD_TRIANGLE) != 0U) {
        action_id = ARM_ACTION_GRAB_MODE_1;
    } else if ((events & GAMEPAD_CIRCLE) != 0U) {
        action_id = ARM_ACTION_GRAB_MODE_2;
    } else if ((events & GAMEPAD_SQUARE) != 0U) {
        action_id = ARM_ACTION_GRAB_MODE_3;
    } else {
        return;
    }

    (void)arm_action_start(action_id, now_ms);
}
