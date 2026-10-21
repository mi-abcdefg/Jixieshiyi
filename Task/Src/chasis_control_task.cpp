//
// Created by MI on 2026/9/13.
//

#include "chasis_control_task.h"
#include "control_mode.h"

#include <cmath>

void ChasisControlTask::run() {
    // 按键边沿检测用（只在"按下那一刻"翻转一次，否则按住会每 10ms 翻一次）
    bool l2_last_pressed = false;   // L2：电机驱动通断
    bool l1_last_pressed = false;   // L1：三档调速循环
    bool r2_last_pressed = false;   // R2：底盘/机械臂操作对象切换
    uint8_t speed_level = 0;        // 0=快速 1=中速(比例) 2=固定慢速；上电默认快速

    // 十字键定距平移状态（仅底盘模式）
    bool pad_was_active = false;
    uint32_t pad_start_ms = 0U;        // 本次按下时刻
    uint32_t pad_step_until_ms = 0U;   // 当前这段平移要走到的时间点
    float pad_vx_cmd = 0.0f;           // 松杆后补完点按步时要用的方向记忆
    float pad_vy_cmd = 0.0f;

    while (1) {
        const uint32_t now_ms = osKernelSysTick();
        const uint8_t btn1 = ps2_get_btn1();
        const uint8_t btn2 = ps2_get_btn2();

        // ---------------- L2 / L1：仅底盘模式生效 ----------------
        // 机械臂模式下这对键交给机械臂子系统（L2 卸力、L1 点动调速），
        // 两个任务各自检测边沿、各自按模式门控，同一时刻只有一方响应。
        const bool chassis_mode = (control_mode_get() == CONTROL_MODE_CHASSIS);

        const bool l2_pressed = (btn2 & PS2_BTN_L2) != 0;
        if (l2_pressed && !l2_last_pressed && chassis_mode) {
            motor_toggle_enable();
        }
        l2_last_pressed = l2_pressed;

        // ---------------- L1：三档调速循环（快 → 中 → 固定慢速） ----------------
        const bool l1_pressed = (btn2 & PS2_BTN_L1) != 0;
        if (l1_pressed && !l1_last_pressed && chassis_mode) {
            speed_level = static_cast<uint8_t>((speed_level + 1U) % 3U);
        }
        l1_last_pressed = l1_pressed;

        // ---------------- R2：操作对象 底盘 ⇄ 机械臂 ----------------
        // 模式本体在 ControlMode 模块（机械臂任务也要读），本任务是
        // 切换动作的唯一所有者；其余地方一律只读。
        const bool r2_pressed = (btn2 & PS2_BTN_R2) != 0;
        if (r2_pressed && !r2_last_pressed) {
            control_mode_toggle();
        }
        r2_last_pressed = r2_pressed;

        // 获取左摇杆和右摇杆数据
        uint8_t left_analog[2];
        uint8_t right_analog[2];
        ps2_get_left_analog_data(left_analog);
        ps2_get_right_analog_data(right_analog);

        // 将摇杆数据转换为车体速度指令：vx/vy 单位 m/s，omega 单位 rad/s
        //   左摇杆上下 -> vx 前进/后退
        //   左摇杆左右 -> vy 原地平移
        //   右摇杆左右 -> omega 原地旋转（右摇杆上下：机械臂肩关节，见 ArmTask）
        // 符号约定：手柄上推/左推时原始值变小，所以"上推"算出 vx<0、"左推"算出 vy<0，
        // 全工程的 vx/vy 正负都以这里为准（十字键定距平移必须同号，见下面的 pad_*_cmd）。
        // 物理上是"上推=前进"还是"上推=后退"，取决于电机接线与 MOTOR_INVERT_xx（已按实车修正），
        // 不要在别处再定义一套方向。
        float vx = (left_analog[1] - 128) / 128.0f * MECANUM_MAX_SPEED;          // 前进/后退速度
        float vy = (left_analog[0] - 128) / 128.0f * MECANUM_MAX_SPEED;          // 左/右速度
        float omega = (right_analog[0] - 128) / 128.0f * MECANUM_MAX_OMEGA_RADPS; // 旋转角速度

        // 原地旋转方向标定（B 车实测反向）：只翻转底盘旋转，见 chasis_control_task.h
        if (CHASIS_OMEGA_INVERT) {
            omega = -omega;
        }

        // ---------------- 固定慢速档 ----------------
        // 方向仍随摇杆，但大小恒定：归一化量越过死区即取满档位速度。
        // 解码层死区外最小归一化值约 0.086，阈值取 0.05 只区分"推没推"。
        if (speed_level == 2U) {
            if (std::fabs(vx) > 0.05f * MECANUM_MAX_SPEED) {
                vx = (vx > 0.0f ? 1.0f : -1.0f) *
                     MECANUM_MAX_SPEED * CHASIS_FIXED_SLOW_SPEED_RATIO;
            } else {
                vx = 0.0f;
            }
            if (std::fabs(vy) > 0.05f * MECANUM_MAX_SPEED) {
                vy = (vy > 0.0f ? 1.0f : -1.0f) *
                     MECANUM_MAX_SPEED * CHASIS_FIXED_SLOW_SPEED_RATIO;
            } else {
                vy = 0.0f;
            }
            if (std::fabs(omega) > 0.05f * MECANUM_MAX_OMEGA_RADPS) {
                omega = (omega > 0.0f ? 1.0f : -1.0f) *
                        MECANUM_MAX_OMEGA_RADPS * CHASIS_FIXED_SLOW_OMEGA_RATIO;
            } else {
                omega = 0.0f;
            }
        }

        // ---------------- 十字键定距平移（仅底盘模式） ----------------
        // 点按（< TAP_MS）走一段固定距离，长按按住期间持续走、松手即停。
        const bool pad_enabled = (control_mode_get() == CONTROL_MODE_CHASSIS);
        const bool pad_up    = pad_enabled && (btn1 & PS2_BTN_UP)    != 0;
        const bool pad_down  = pad_enabled && (btn1 & PS2_BTN_DOWN)  != 0;
        const bool pad_left  = pad_enabled && (btn1 & PS2_BTN_LEFT)  != 0;
        const bool pad_right = pad_enabled && (btn1 & PS2_BTN_RIGHT) != 0;
        const bool pad_active = pad_up || pad_down || pad_left || pad_right;

        if (pad_active) {
            if (!pad_was_active) {
                pad_start_ms = now_ms;   // 新按下：开始计时
            }
            // 按住期间不断续期"走到这个时刻"，即持续移动
            pad_step_until_ms = now_ms + CHASIS_PAD_STEP_MS;
            // 方向记忆：松杆补完点按步时保持原方向。
            // **必须与左摇杆同号**：摇杆上推时原始值变小，所以 vx<0 才是摇杆算出来的
            // "前"；左右同理（LR 变小 = vx<0 那一侧）。原来这里写成
            // "上=+vx、左=+vy"，与摇杆正好相反——按 ↑ 实际往后、按 ← 实际往右，
            // 和"十字键不走"的现象是一起来的（2026/10/21 修）。
            pad_vx_cmd = (pad_down ? 1.0f : 0.0f) - (pad_up ? 1.0f : 0.0f);
            pad_vy_cmd = (pad_right ? 1.0f : 0.0f) - (pad_left ? 1.0f : 0.0f);
        } else if (pad_was_active) {
            // 松开：点按则补完整一步，长按则立即停
            pad_step_until_ms = (now_ms - pad_start_ms < CHASIS_PAD_TAP_MS)
                                ? pad_start_ms + CHASIS_PAD_STEP_MS
                                : now_ms;
        }
        pad_was_active = pad_active;

        // 生效判定：按住中，或处于"补完点按"阶段（now < until，用减法做回绕安全比较）
        const bool pad_driving = pad_active ||
                                 (static_cast<int32_t>(now_ms - pad_step_until_ms) < 0);
        if (pad_driving) {
            // 十字键优先于摇杆；平移期间旋转置零
            vx    = pad_vx_cmd * CHASIS_PAD_SPEED_MPS;
            vy    = pad_vy_cmd * CHASIS_PAD_SPEED_MPS;
            omega = 0.0f;
        }

        // ---------------- 模式门控 ----------------
        // 操作对象切到机械臂时，轮速指令一律清零（注意仍在解算后正常下发 0，
        // 让 Motor 内部状态保持一致）。切走瞬间即使摇杆还推着，车也立刻停；
        // 切回底盘时若摇杆仍推着会立即恢复运动——与 L2 使能关断同款特性。
        if (control_mode_get() == CONTROL_MODE_ARM) {
            vx = 0.0f;
            vy = 0.0f;
            omega = 0.0f;
        }

        // 运动学解算：得到四个轮子的线速度 [m/s]。
        // 档位缩放交给解算层（throttle_scale），**不能**再乘在解算之后的指令上：
        // 那样会把"起步下限（阻力前馈）"一起缩小（0.8×60% = 48%），
        // 中速档小油门又会出现"推了不动"。固定慢速档（==2）的速度已在上面
        // 直接按档位速度生成，所以它的 throttle 是 1.0。
        const float throttle = (speed_level == 1U) ? CHASIS_SLOW_MODE_SCALE : 1.0f;

        float wheel_speeds[4];
        mecanum_get_wheel_speeds(vx, vy, omega, throttle, wheel_speeds);

        // 量纲换算：轮子线速度 [m/s] -> 速度指令 [0 ~ MOTOR_CMD_MAX]。
        // MECANUM_MAX_SPEED 是满量程轮速，对应 100% 占空比。
        const float cmd_scale = static_cast<float>(MOTOR_CMD_MAX) / MECANUM_MAX_SPEED;

        // 设置轮子速度。驱动被 L2 关断时，Motor::set_wheel_speed() 内部会自动按 0 输出，
        // 所以这里正常下发即可，不需要再判断使能状态。
        motor_set_wheel_speed(static_cast<int>(wheel_speeds[MECANUM_WHEEL_FL] * cmd_scale),
                              static_cast<int>(wheel_speeds[MECANUM_WHEEL_FR] * cmd_scale),
                              static_cast<int>(wheel_speeds[MECANUM_WHEEL_BL] * cmd_scale),
                              static_cast<int>(wheel_speeds[MECANUM_WHEEL_BR] * cmd_scale));

        osDelay(10); // 延时，避免任务占用过多CPU时间
    }
}

extern "C" {
    ChasisControlTask chasis_control_task;

    void chasis_control_task_init() {
        motor_init();   // 启动 PWM；DRV8833 保持关断，由 L2 控制使能
        chasis_control_task.start("ChasisControlTask", 512, osPriorityNormal);
    }
}
