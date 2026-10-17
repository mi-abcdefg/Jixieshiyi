//
// 舵机驱动实现（移植自 CRTC_RobotArm 工程 servo.c，htim3 → htim4）
//
// 50Hz PWM 下 CCR 值 = 脉宽微秒数（PSC=71, ARR=19999，TIM4 时钟 72MHz）。
//

#include "servo_driver.h"
#include "tim.h"
#include <stddef.h>

/**
 * @brief 单个舵机的配置参数
 */
typedef struct
{
    uint32_t channel;          // 对应的 TIM4 通道
    uint16_t min_pulse_us;     // 最小安全脉宽
    uint16_t max_pulse_us;     // 最大安全脉宽
    float min_angle_deg;       // 最小安全角度
    float max_angle_deg;       // 最大安全角度
    float startup_angle_deg;   // 上电初始角度
    bool reversed;             // 是否反向（安装方向相反时置 1，不用改线）
} ServoConfig_t;

/**
 * @brief 四个舵机的配置表（编号 → 通道/限幅，按实际接线排序）
 *
 * 实际接线（2026-10-17 通道辨识测试确认）：PB6 夹爪 SG90、PB7 肩 SG90
 * （11cm 臂）、PB8 肘 SG90（8.5cm 臂）、PB9 底座 MG995（绕 Z）。
 * 注意机构从下往上仍是 底座→肩→肘→夹爪，但 PB6 与 PB9 的插头和最初
 * 描述正好互换——上层全用 ServoId_t，通道差异只体现在本配置表。
 *
 * 脉宽范围按数据手册标称值统一 500~2500µs（MG995 标称 500~2500µs；
 * SG90 要求标准 PWM：周期 20ms、高电平 0.5~2.5ms。TIM4 配置恰好是
 * 50Hz/20ms，CCR 值即微秒数，无需改动）。
 *
 * 注意：电气范围 ≠ 机械安全范围。臂体连杆机构未必允许每个关节跑满
 * 180°——若实物上某关节接近极限会顶死/堵转，把该舵机的
 * min/max_pulse_us（或对应 min/max_angle_deg）向内收紧。
 * 上电初始角统一 90°（电气中位，脉宽 1500µs）。
 */
static const ServoConfig_t servo_config[SERVO_COUNT] =
{
    /* 底座：绕 Z 轴旋转（PB9，MG995） */
    {
        TIM_CHANNEL_4,
        500,
        2500,
        0.0f,
        180.0f,
        90.0f,
        false
    },

    /* 肩关节：第一段连杆，臂长 11cm（PB7，SG90） */
    {
        TIM_CHANNEL_2,
        500,
        2500,
        0.0f,
        180.0f,
        90.0f,
        false
    },

    /* 肘关节：第二段连杆，臂长 8.5cm（PB8，SG90）
     * 角度域用负值 [-180°, 0°]：arm_ik 的"肘上"解给第二连杆的相对角恒为负
     * （范围 [-180°, 0]）。若按 [0°, 180°] 限幅，负解会被硬夹到 0°——
     * 表现为推杆时舵机猛摆向电气极限、松杆再推又从被夹过的错误姿态重新起步。
     * 初始角 -90° 即该角度域的中位（脉宽 1500µs，与其它舵机的 90° 等价）。 */
    {
        TIM_CHANNEL_3,
        500,
        2500,
        -180.0f,
        0.0f,
        -90.0f,
        false
    },

    /* 夹爪（PB6，SG90） */
    {
        TIM_CHANNEL_1,
        500,
        2500,
        0.0f,
        180.0f,
        90.0f,
        false
    }
};

/* 软件记录的是指令位置；普通三线舵机没有实际角度反馈。 */
static float commanded_angle_deg[SERVO_COUNT];
static bool commanded_angle_valid[SERVO_COUNT];

bool servo_set_pulse(ServoId_t servo, uint16_t pulse_us)
{
    /* 检查舵机编号是否合法 */
    if ((uint32_t)servo >= (uint32_t)SERVO_COUNT)
    {
        return false;
    }

    const ServoConfig_t *config = &servo_config[servo];

    /* 脉宽下限/上限保护 */
    if (pulse_us < config->min_pulse_us)
    {
        pulse_us = config->min_pulse_us;
    }
    if (pulse_us > config->max_pulse_us)
    {
        pulse_us = config->max_pulse_us;
    }

    /* 将脉宽写入对应 TIM4 通道（CCR = 微秒值） */
    __HAL_TIM_SET_COMPARE(&htim4, config->channel, pulse_us);

    /* 兼容直接调用 servo_set_pulse() 的代码，保持位置记录一致。 */
    float ratio = (float)(pulse_us - config->min_pulse_us) /
                  (float)(config->max_pulse_us - config->min_pulse_us);
    if (config->reversed)
    {
        ratio = 1.0f - ratio;
    }
    commanded_angle_deg[servo] = config->min_angle_deg +
        ratio * (config->max_angle_deg - config->min_angle_deg);
    commanded_angle_valid[servo] = true;

    return true;
}

bool servo_set_angle(ServoId_t servo, float angle_deg)
{
    /* 检查舵机编号是否合法 */
    if ((uint32_t)servo >= (uint32_t)SERVO_COUNT)
    {
        return false;
    }

    const ServoConfig_t *config = &servo_config[servo];

    /* 拒绝 NaN；超出范围的有限值及无穷大会被下面的限幅处理。 */
    if (angle_deg != angle_deg)
    {
        return false;
    }

    /* 角度下限/上限保护 */
    if (angle_deg < config->min_angle_deg)
    {
        angle_deg = config->min_angle_deg;
    }
    if (angle_deg > config->max_angle_deg)
    {
        angle_deg = config->max_angle_deg;
    }

    /* 将角度转换为 0~1 的比例；安装方向相反则反转比例 */
    float ratio =
        (angle_deg - config->min_angle_deg) /
        (config->max_angle_deg - config->min_angle_deg);
    if (config->reversed)
    {
        ratio = 1.0f - ratio;
    }

    /* 角度比例 → 脉宽 */
    const uint16_t pulse_range_us =
        (uint16_t)(config->max_pulse_us - config->min_pulse_us);
    const float pulse_value_us =
        (float)config->min_pulse_us + ratio * (float)pulse_range_us;
    const uint16_t pulse_us = (uint16_t)pulse_value_us;

    if (!servo_set_pulse(servo, pulse_us))
    {
        return false;
    }

    /* 保留已经限幅的角度；脉宽取整产生的误差不累加到下一轨迹。 */
    commanded_angle_deg[servo] = angle_deg;
    return true;
}

bool servo_get_commanded_angle(ServoId_t servo, float *angle_deg)
{
    if ((uint32_t)servo >= (uint32_t)SERVO_COUNT ||
        angle_deg == NULL || !commanded_angle_valid[servo])
    {
        return false;
    }
    *angle_deg = commanded_angle_deg[servo];
    return true;
}

bool servo_init(void)
{
    /* 先写入四个舵机的初始角度（CCR 就位后再开 PWM，避免上电乱摆） */
    for (uint32_t i = 0; i < (uint32_t)SERVO_COUNT; i++)
    {
        if (!servo_set_angle((ServoId_t)i,
                             servo_config[i].startup_angle_deg))
        {
            return false;
        }
    }

    /* 启动 TIM4 四路 PWM（全工程此前从未启动过 TIM4） */
    for (uint32_t i = 0; i < (uint32_t)SERVO_COUNT; i++)
    {
        if (HAL_TIM_PWM_Start(&htim4, servo_config[i].channel) != HAL_OK)
        {
            return false;
        }
    }

    return true;
}
