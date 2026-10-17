//
// Created by MI on 2026/10/17.
//

// 舵机逐台自检任务实现（台架测试用）。
// 只依赖 servo_driver，不碰 arm_* 任何模块，避免测试期间被动作/点动抢写脉宽。

#include "servo_test_task.h"
#include "servo_driver.h"

/* ------------------------------ 测试参数 ------------------------------ */
#define SERVO_TEST_STARTUP_SETTLE_MS  2000U  // 上电复位后的等待时间
#define SERVO_TEST_SWING_DEG          45.0f  // 每台正转的角度
#define SERVO_TEST_STEP_DEG           3.0f   // 平滑步进的角步长
#define SERVO_TEST_STEP_PERIOD_MS     15U    // 步进周期（3°/15ms ≈ 200°/s 上限，实际被满量程限幅）
#define SERVO_TEST_HOLD_MS            800U   // 转到测试位后的停留时间
#define SERVO_TEST_BETWEEN_MS         400U   // 回位后到下一台之间的间隔

namespace {

    /* 缓慢步进到目标角度（servo_set_angle 内部会按舵机限幅再夹一次）。
     * 不用一步跳变：带载的 MG995 直接跳 45° 会甩臂，步进也方便肉眼确认轨迹。 */
    void move_slow(ServoId_t id, float target_deg) {
        float current = target_deg;
        servo_get_commanded_angle(id, &current);   // init 后必有指令角，失败则原地不动

        while (current < target_deg) {
            current += SERVO_TEST_STEP_DEG;
            if (current > target_deg) current = target_deg;
            servo_set_angle(id, current);
            osDelay(SERVO_TEST_STEP_PERIOD_MS);
        }
        while (current > target_deg) {
            current -= SERVO_TEST_STEP_DEG;
            if (current < target_deg) current = target_deg;
            servo_set_angle(id, current);
            osDelay(SERVO_TEST_STEP_PERIOD_MS);
        }
    }

    /* 测一台：正转 SWING_DEG → 停 → 回初始位 */
    void test_one_servo(ServoId_t id) {
        float startup_deg = 0.0f;
        servo_get_commanded_angle(id, &startup_deg);

        move_slow(id, startup_deg + SERVO_TEST_SWING_DEG);
        osDelay(SERVO_TEST_HOLD_MS);
        move_slow(id, startup_deg);
        osDelay(SERVO_TEST_BETWEEN_MS);
    }
}

void ServoTestTask::run() {
    // 复位：写初始角度并启动 TIM4 PWM（上电瞬间各舵机摆到初始姿态）
    if (!servo_init()) {
        for (;;) {
            osDelay(1000);   // PWM 启动失败（正常不会发生）：停住本任务，不影响其它任务
        }
    }

    // 等 2 秒：舵机在初始位稳定，也避开上电电源爬坡
    osDelay(SERVO_TEST_STARTUP_SETTLE_MS);

    /* 从下到上逐台测试（顺序即物理布局，勿改） */
    test_one_servo(SERVO_BASE_Z);    // PB6 底座 MG995（绕 Z）
    test_one_servo(SERVO_LINK_1);    // PB7 肩 SG90（11cm 臂）
    test_one_servo(SERVO_LINK_2);    // PB8 肘 SG90（8.5cm 臂）
    test_one_servo(SERVO_GRIPPER);   // PB9 夹爪 SG90

    // 测试完成：保持初始姿态待机（回位已由 test_one_servo 完成）
    for (;;) {
        osDelay(1000);
    }
}

extern "C" {
    ServoTestTask servo_test_task;

    void servo_test_task_init() {
        servo_test_task.start("ServoTestTask", 256, osPriorityNormal);
    }
}
