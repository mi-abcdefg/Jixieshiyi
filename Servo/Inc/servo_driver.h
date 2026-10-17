//
// 机械臂舵机驱动（TIM4 四通道 → PB6~PB9，50Hz，CCR 直接写微秒值）
//
// 移植自 CRTC_RobotArm 工程 servo.c/h，改动两处：
//   1. 定时器 htim3 → htim4：车上的 TIM3 已被 BL/BR 两个电机占用，
//      舵机物理接线也必须接到车的 PB6~PB9（原机械臂板是 PA6/PA7/PB0/PB1）。
//   2. 对外导出名统一为工程的 snake_case 风格（motor_init / ps2_get_btn1 同款）。
//
// 注意：TIM4 的 PWM 由 servo_init() 内部 HAL_TIM_PWM_Start() 启动，
// 在此之前全工程没有任何地方启动过 TIM4。
//

#ifndef CRTC2026_ROBOTA_SERVO_DRIVER_H
#define CRTC2026_ROBOTA_SERVO_DRIVER_H

#pragma once
#include <stdint.h>
#include <stdbool.h>

/* --------------------------- 舵机功能编号 ---------------------------
 * 编号与机械臂结构对应，上层（arm_control / arm_action / arm_manual）
 * 一律用编号而非定时器通道，硬件改动只落在本驱动内部。
 * ------------------------------------------------------------------ */
typedef enum {
    SERVO_GRIPPER = 0,   // 舵机1：夹爪
    SERVO_LINK_1,        // 舵机2：第一段连杆
    SERVO_LINK_2,        // 舵机3：第二段连杆
    SERVO_BASE_Z,        // 舵机4：绕 Z 轴旋转（底座）
    SERVO_COUNT          // 舵机总数，当前为 4
} ServoId_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化四个舵机：先写初始角度，再启动 TIM4 四路 PWM
 * @return true 初始化成功 / false 失败（HAL 启动 PWM 失败）
 */
bool servo_init(void);

/**
 * @brief 直接设置舵机 PWM 高电平时间
 * @param servo    舵机编号
 * @param pulse_us 高电平时间 [µs]，超出该舵机安全范围会被限幅
 * @return false 表示编号无效
 */
bool servo_set_pulse(ServoId_t servo, uint16_t pulse_us);

/**
 * @brief 设置舵机目标角度（会先按该舵机的角度范围限幅，再换算成脉宽）
 * @return false 表示编号无效或角度为 NaN
 */
bool servo_set_angle(ServoId_t servo, float angle_deg);

/**
 * @brief 读取最近一次下发的目标角度（普通三线舵机没有实际角度反馈）
 * @return false 表示编号无效、指针为空或该舵机尚未下发过目标
 */
bool servo_get_commanded_angle(ServoId_t servo, float *angle_deg);

#ifdef __cplusplus
}
#endif

#endif //CRTC2026_ROBOTA_SERVO_DRIVER_H
