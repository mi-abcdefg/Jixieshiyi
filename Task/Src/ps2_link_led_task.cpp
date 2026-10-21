//
// Created by MI on 2026/10/2.
//

// PC13 链路状态指示灯：全车唯一的板载状态输出，语义如下——
//
//   PS2 模式字节      含义                       PC13 表现
//   -------------    ------------------------   ---------------------
//   0x73             已连上 + 模拟模式           常亮（摇杆可用，能开车）
//   0x41             已连上 + 数字模式           慢闪 1 Hz（摇杆无效）
//   其它值           没收到合法应答 / 没连上     快闪 5 Hz
//
// 判据常量（PS2_MODE_ANALOG/DIGITAL）的唯一数据源在 ps2_decode.h。

#include "ps2_link_led_task.h"
#include "main.h"
#include "ps2_decode.h"

/* --------------------------- PC13（低电平点亮） --------------------------- */
#define LED_PORT       GPIOC
#define LED_PIN        GPIO_PIN_13
#define LED_ON()       HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET)
#define LED_OFF()      HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET)

/* ------------------------------ 闪烁节拍 ------------------------------ */
#define LED_TASK_PERIOD_MS     20u     // 本任务轮询周期
#define LED_SLOW_HALF_TICKS    25u     // 25 × 20ms = 500ms 半周期 → 1 Hz
#define LED_FAST_HALF_TICKS    5u      //  5 × 20ms = 100ms 半周期 → 5 Hz

void Ps2LinkLedTask::run() {
    // 手柄数据由 RemoteCommTask 每 10ms 解码一次，这里只读解码结果。
    // 不要在本任务再调读取函数：位操作时序不允许两个任务同时上总线。
    uint32_t tick = 0;   // 闪烁计数（单位 = LED_TASK_PERIOD_MS）

    while (1) {
        const uint8_t mode = ps2_get_mode();

        if (mode == PS2_MODE_ANALOG) {
            LED_ON();                     // 已连上 + 模拟模式：常亮
        } else {
            // 0x41 = 已连上但数字模式（慢闪）；其它 = 未连接（快闪）
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
    Ps2LinkLedTask ps2_link_led_task;

    void ps2_link_led_task_init() {
        ps2_link_led_task.start("Ps2LinkLedTask", 256, osPriorityAboveNormal);
    }
}
