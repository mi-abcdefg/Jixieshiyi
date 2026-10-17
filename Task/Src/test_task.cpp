//
// Created by MI on 2026/10/2.
//

#include "test_task.h"
#include "main.h"
#include "ps2_decode.h"

/* ====================== PC13 状态指示灯 ======================
 * 这个任务现在充当"PS2 链路状态灯"，不再做按键测试。
 *
 *   PS2 模式字节      含义                       PC13 表现
 *   -------------    ------------------------   ---------------------
 *   0x73             已连上 + 模拟模式           常亮（摇杆可用，能开车）
 *   0x41             已连上 + 数字模式           慢闪 1 Hz（摇杆无效）
 *   其它值           没收到合法应答 / 没连上     快闪 5 Hz
 *
 * 判断依据：真正的 PS2 手柄/接收器应答里，第 2 个字节恒为
 * 0x41(数字模式) 或 0x73(模拟模式)。接收器没接上、手柄没配对、
 * 或链路断开时 DATA 线被上拉，读到的是 0xFF，模式字节就不是这两个值，
 * 于是判为"未连接"。
 * =========================================================== */

#define LED_PORT       GPIOC
#define LED_PIN        GPIO_PIN_13
#define LED_ON()       HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET)  // PC13 低电平点亮
#define LED_OFF()      HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET)

#define PS2_MODE_ANALOG        0x73u   // 模拟模式
#define PS2_MODE_DIGITAL       0x41u   // 数字模式

#define LED_TASK_PERIOD_MS     20u     // 本任务轮询周期
#define LED_SLOW_HALF_TICKS    25u     // 25 × 20ms = 500ms 半周期 → 1 Hz
#define LED_FAST_HALF_TICKS    5u      //  5 × 20ms = 100ms 半周期 → 5 Hz

void TestTask::run() {
    // 手柄数据由 RemoteCommTask 每 10ms 轮询并解析一次（ps2_read_and_decode_JoystickData），
    // 这里只读解析结果。不要在本任务里再调一次读取函数：那是位操作时序，
    // 两个任务同时读会互相打断总线、得到错误数据。
    uint32_t tick = 0;   // 闪烁计数（单位 = LED_TASK_PERIOD_MS）

    while (1) {
        const uint8_t mode = ps2_get_mode();

        if (mode == PS2_MODE_ANALOG) {
            // 已连上 + 模拟模式：常亮
            LED_ON();
        } else {
            // 0x41 = 已连上但处于数字模式（慢闪）；其它 = 未连接（快闪）
            const uint32_t half = (mode == PS2_MODE_DIGITAL) ? LED_SLOW_HALF_TICKS
                                                             : LED_FAST_HALF_TICKS;
            if (((tick / half) & 1u) == 0u) {
                LED_ON();
            } else {
                LED_OFF();
            }
        }

        ++tick;
        osDelay(LED_TASK_PERIOD_MS);
    }
}



extern "C" {
    TestTask test_task;
    void test_task_init() {
        test_task.start("TestTask", 256, osPriorityAboveNormal);
    }
}
