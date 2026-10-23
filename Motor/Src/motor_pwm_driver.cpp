//
// Created by MI on 2026/9/13.
//

#include "motor_pwm_driver.h"
#include "main.h"   // DRV_1_Pin / DRV_2_Pin 定义在这里

Motor::Motor() {

}

void Motor::init() {
    // 先把 8 路 PWM 启动起来（此时 CCR 都是 0，输出全低）
    HAL_TIM_PWM_Start(TIM_FL_1, CH_FL_1);
    HAL_TIM_PWM_Start(TIM_FL_2, CH_FL_2);
    HAL_TIM_PWM_Start(TIM_FR_1, CH_FR_1);
    HAL_TIM_PWM_Start(TIM_FR_2, CH_FR_2);
    HAL_TIM_PWM_Start(TIM_BL_1, CH_BL_1);
    HAL_TIM_PWM_Start(TIM_BL_2, CH_BL_2);
    HAL_TIM_PWM_Start(TIM_BR_1, CH_BR_1);
    HAL_TIM_PWM_Start(TIM_BR_2, CH_BR_2);

    // 上电【不】使能：DRV8833 保持休眠（nSLEEP 低），电机不会自己转，
    // 由 L2 键翻转使能（见 ChasisControlTask）。
    // MX_GPIO_Init() 已经把 DRV_1/DRV_2 置低，这里再显式关断一次，
    // 顺便把 PWM 指令清零，保证上电状态是确定的。
    slow_decay_ = true;   // 上电默认慢衰减（底盘模式 R1 可切快衰减）
    disable();
}

/* ------------------------- DRV8833 使能控制 ------------------------- */

void Motor::enable() {
    HAL_GPIO_WritePin(DRV_1_GPIO_Port, DRV_1_Pin, MOTOR_DRV_ENABLE_LEVEL);
    HAL_GPIO_WritePin(DRV_2_GPIO_Port, DRV_2_Pin, MOTOR_DRV_ENABLE_LEVEL);
    enabled_ = true;
}

void Motor::disable() {
    // 先把 PWM 指令清零再拉低 nSLEEP。否则重新使能的瞬间会带着上一次
    // 残留的占空比突然窜车。（set_wheel_speed 在未使能时会强制写 0，
    // 所以在 init() 里第一次调用也安全。）
    set_wheel_speed(0, 0, 0, 0);

    HAL_GPIO_WritePin(DRV_1_GPIO_Port, DRV_1_Pin, MOTOR_DRV_DISABLE_LEVEL);
    HAL_GPIO_WritePin(DRV_2_GPIO_Port, DRV_2_Pin, MOTOR_DRV_DISABLE_LEVEL);
    enabled_ = false;
}

void Motor::toggle_enable() {
    if (enabled_) {
        disable();
    } else {
        enable();
    }
}

bool Motor::is_enabled() const {
    return enabled_;
}

/* ------------------------- 衰减模式（快/慢） -------------------------
 * DRV8833 的 PWM 控制真值表（x = 各 H 桥的两路输入）：
 *   IN1=PWM, IN2=0 → 正转，快速衰减    IN1=1, IN2=PWM → 正转，慢速衰减
 *   IN1=0, IN2=PWM → 反转，快速衰减    IN1=PWM, IN2=1 → 反转，慢速衰减
 * 慢衰减的正/反转把"常高"放在另一脚、PWM 占空比取反（= 1 − |速度|），
 * 等效驱动占空比仍为 |速度|——控制层（0~1000）完全不用改。
 * 满速时慢衰减的波形与快衰减重合（PWM 占空比 0）。 */

void Motor::set_slow_decay(bool slow) {
    slow_decay_ = slow;
}

void Motor::toggle_decay_mode() {
    slow_decay_ = !slow_decay_;
}

bool Motor::is_slow_decay() const {
    return slow_decay_;
}

/* --------------------------- PWM 输出 --------------------------- */

// 把 0 ~ MOTOR_CMD_MAX 的指令换算成当前定时器 ARR 下的比较值。
// 这样无论 PWM 频率设成 1 kHz 还是 20 kHz，MOTOR_CMD_MAX 始终等于满占空比。
static uint32_t cmd_to_compare(TIM_HandleTypeDef* htim, int cmd) {
    if (cmd < 0) {
        cmd = 0;
    }
    if (cmd > MOTOR_CMD_MAX) {
        cmd = MOTOR_CMD_MAX;
    }
    const uint32_t period = __HAL_TIM_GET_AUTORELOAD(htim) + 1U;
    return static_cast<uint32_t>(static_cast<uint64_t>(cmd) * period / MOTOR_CMD_MAX);
}

void motor_cmd(TIM_HandleTypeDef* htim, uint32_t channel1, uint32_t channel2,
               int speed, bool slow_decay) {
    if (speed == 0) {
        // 两脚都低 = 滑行（快/慢衰减模式下一致，不做刹车）
        __HAL_TIM_SET_COMPARE(htim, channel1, 0);
        __HAL_TIM_SET_COMPARE(htim, channel2, 0);
        return;
    }

    const uint32_t pwm = cmd_to_compare(htim, (speed < 0) ? -speed : speed);
    const uint32_t high = __HAL_TIM_GET_AUTORELOAD(htim) + 1U;   // 常高 = 100%

    if (slow_decay) {
        // 慢衰减：常高一脚 + 反相 PWM（占空比 = 1 − |速度|）
        if (speed > 0) {
            __HAL_TIM_SET_COMPARE(htim, channel1, high);       // IN1=1
            __HAL_TIM_SET_COMPARE(htim, channel2, high - pwm); // IN2=PWM
        } else {
            __HAL_TIM_SET_COMPARE(htim, channel1, high - pwm); // IN1=PWM
            __HAL_TIM_SET_COMPARE(htim, channel2, high);       // IN2=1
        }
    } else {
        // 快衰减：一脚 PWM、另一脚常低
        if (speed > 0) {
            __HAL_TIM_SET_COMPARE(htim, channel1, pwm);
            __HAL_TIM_SET_COMPARE(htim, channel2, 0);
        } else {
            __HAL_TIM_SET_COMPARE(htim, channel1, 0);
            __HAL_TIM_SET_COMPARE(htim, channel2, pwm);
        }
    }
}

void Motor::set_wheel_speed(int fl_speed, int fr_speed, int bl_speed, int br_speed) {
    // 接线方向修正：某个轮子电机正负极接反时，把 motor_pwm_driver.h 里
    // 对应的 MOTOR_INVERT_xx 改成 1 即可，不用改线。
#if MOTOR_INVERT_FL
    fl_speed = -fl_speed;
#endif
#if MOTOR_INVERT_FR
    fr_speed = -fr_speed;
#endif
#if MOTOR_INVERT_BL
    bl_speed = -bl_speed;
#endif
#if MOTOR_INVERT_BR
    br_speed = -br_speed;
#endif

    // 驱动未使能时禁止输出：无论上层下发什么，都按 0 输出。
    // 这样即使上层忘了判断使能状态，也不会在关断状态下留下占空比。
    if (!enabled_) {
        fl_speed = 0;
        fr_speed = 0;
        bl_speed = 0;
        br_speed = 0;
    }

    motor_cmd(TIM_FL_1, CH_FL_1, CH_FL_2, fl_speed, slow_decay_);
    motor_cmd(TIM_FR_1, CH_FR_1, CH_FR_2, fr_speed, slow_decay_);
    motor_cmd(TIM_BL_1, CH_BL_1, CH_BL_2, bl_speed, slow_decay_);
    motor_cmd(TIM_BR_1, CH_BR_1, CH_BR_2, br_speed, slow_decay_);
}

extern "C" {
    Motor motor1;
    void motor_init() {
        motor1.init();
    }
    void motor_set_wheel_speed(int fl_speed, int fr_speed, int bl_speed, int br_speed) {
        motor1.set_wheel_speed(fl_speed, fr_speed, bl_speed, br_speed);
    }

    void motor_enable(void) {
        motor1.enable();
    }
    void motor_disable(void) {
        motor1.disable();
    }
    void motor_toggle_enable(void) {
        motor1.toggle_enable();
    }
    uint8_t motor_is_enabled(void) {
        return motor1.is_enabled() ? 1U : 0U;
    }

    void motor_set_slow_decay(bool slow) {
        motor1.set_slow_decay(slow);
    }
    void motor_toggle_decay_mode(void) {
        motor1.toggle_decay_mode();
    }
    uint8_t motor_is_slow_decay(void) {
        return motor1.is_slow_decay() ? 1U : 0U;
    }
}
