//
// Created by MI on 2026/10/17.
//

// 舵机逐台自检任务实现（台架测试用）。
// 只依赖 servo_driver 和 PC13 指示灯，不碰 arm_* 任何模块，
// 避免测试期间被动作/点动抢写脉宽。
//
// PC13 的状态灯任务（TestTask）在本测试模式下被关闭（见 freertos.c），
// 指示灯由本任务独占，用作"进度报数"。

#include "servo_test_task.h"
#include "servo_driver.h"
#include "stm32f1xx.h"

/* ------------------------------ 测试参数 ------------------------------ */
#define SERVO_TEST_STARTUP_SETTLE_MS  2000U  // 上电复位后的等待时间
#define SERVO_TEST_SWING_DEG          45.0f  // 每台正转的角度
#define SERVO_TEST_STEP_DEG           3.0f   // 平滑步进的角步长
#define SERVO_TEST_STEP_PERIOD_MS     15U    // 步进周期
#define SERVO_TEST_HOLD_MS            800U   // 转到测试位后的停留时间
#define SERVO_TEST_BETWEEN_MS         400U   // 回位后到下一台之间的间隔
#define SERVO_TEST_ROUND_GAP_MS       1000U  // 一轮结束到下一轮的间隔

/* PC13 低电平点亮（与 TestTask 的定义一致） */
#define LED_PORT       GPIOC
#define LED_PIN        GPIO_PIN_13
#define LED_ON()       HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET)
#define LED_OFF()      HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET)

#define LED_BLINK_ON_MS      150U   // 报数闪烁：亮 150ms
#define LED_BLINK_OFF_MS     150U   //            灭 150ms
#define LED_ROUND_ON_MS      2000U  // 一轮结束的"长亮"标志

namespace {

    /* 报数：连闪 count 下（第几台就闪几下） */
    void led_blink(uint32_t count) {
        for (uint32_t i = 0; i < count; i++) {
            LED_ON();
            osDelay(LED_BLINK_ON_MS);
            LED_OFF();
            osDelay(LED_BLINK_OFF_MS);
        }
    }

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
        // PWM 启动失败（正常不会发生）：长亮报警并停住本任务
        LED_ON();
        for (;;) {
            osDelay(1000);
        }
    }

    // 等 2 秒：舵机在初始位稳定，也避开上电电源爬坡
    osDelay(SERVO_TEST_STARTUP_SETTLE_MS);

    /* 从下到上逐台测试（顺序即物理布局，勿改）。
     * 无限循环，方便反复观察；断电即可停止。 */
    const ServoId_t test_order[SERVO_COUNT] = {
        SERVO_BASE_Z,    // 第1台：PB6 底座 MG995（绕 Z）
        SERVO_LINK_1,    // 第2台：PB7 肩 SG90（11cm 臂）
        SERVO_LINK_2,    // 第3台：PB8 肘 SG90（8.5cm 臂）
        SERVO_GRIPPER    // 第4台：PB9 夹爪 SG90
    };

    for (;;) {
        for (uint32_t i = 0; i < (uint32_t)SERVO_COUNT; i++) {
            /* 先报数再动作：闪 (i+1) 下 → 第 (i+1) 台动。
             * 灯闪了但舵机没动 = 该路硬件（接线/供电）问题；
             * 灯不往下闪 = 程序问题（正常不会发生）。 */
            led_blink(i + 1U);
            osDelay(400);
            test_one_servo(test_order[i]);
        }

        /* 一轮结束标志：灯长亮 2 秒，随后开始下一轮 */
        LED_ON();
        osDelay(LED_ROUND_ON_MS);
        LED_OFF();
        osDelay(SERVO_TEST_ROUND_GAP_MS);
    }
}

extern "C" {
    ServoTestTask servo_test_task;

    void servo_test_task_init() {
        servo_test_task.start("ServoTestTask", 256, osPriorityNormal);
    }
}
