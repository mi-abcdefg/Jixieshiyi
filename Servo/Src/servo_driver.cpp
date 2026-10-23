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
 * @brief 四个舵机的配置表（按 ServoId_t 编号顺序，**下标必须是舵机编号**）
 *
 * 实际接线（2026-10-17 通道辨识测试确认，2026/10/21 复核）：
 *   PB9/TIM4_CH4  夹爪 SG90              → SERVO_GRIPPER = 0
 *   PB7/TIM4_CH2  肩 SG90（11cm 连杆）    → SERVO_LINK_1  = 1
 *   PB8/TIM4_CH3  肘 SG90（8.5cm 连杆）   → SERVO_LINK_2  = 2
 *   PB6/TIM4_CH1  底座 MG995（绕 Z 轴）   → SERVO_BASE_Z  = 3
 * 注意 PB6 与 PB9 和"从下往上"的机构顺序正好交叉：底座在 CH1、夹爪在 CH4。
 * 判别依据是通道辨识的**观测**：当年命令底座时动的是夹爪、底座不动，说明
 * 底座的命令落在了夹爪所在的那一路（PB9）——因此底座在 PB6、夹爪在 PB9。
 * 上层一律用 ServoId_t，通道差异只体现在本配置表。
 *
 * 脉宽范围按数据手册标称值统一 500~2500µs（MG995 标称 500~2500µs；
 * SG90 要求标准 PWM：周期 20ms、高电平 0.5~2.5ms。TIM4 配置恰好是
 * 50Hz/20ms，CCR 值即微秒数，无需改动）。
 *
 * 注意：电气范围 ≠ 机械安全范围。臂体连杆机构未必允许每个关节跑满
 * 180°——若实物上某关节接近极限会顶死/堵转，把该舵机的
 * min/max_pulse_us（或对应 min/max_angle_deg）向内收紧。
 *
 * ⚠️ 历史上这里出过一次错：2026/10/17 修正通道时只换了 channel 字段，
 * 角度域/初始角仍留在原来的表格位置上，导致"命令角域"与关节错配——
 * 底座拿到 [0°,180°]（关键帧 user 90 与 180 都被夹到同一个脉宽，表现为
 * "↑ 和 × 落点相同"），夹爪拿到 [90°,270°]（开合只跑得动很小一段，
 * 表现为"○ 只能观察到开"）。2026/10/21 重建动作模块时把角度域、
 * 初始角按关节重新归位（数值本身取自 docs/机械臂动作模块重建交接.md，
 * 没做任何重新标定）。**改本表时请连同行注释一起看：编号=下标=关节。**
 */
static const ServoConfig_t servo_config[SERVO_COUNT] =
{
    /* 大臂（肩）舵机安装方向按车分套（2026/10/21 A 车实测：装反）：两车互为
     * 镜像。reversed 翻转的是**整条通道**——摇杆点动 / 动作关键帧 / 末端
     * 点动一起镜像物理方向；肩的用户角约定、篮内判定阈值、末端几何换算
     * 都按"方向正确"标定，翻转后自然对齐，上层无需任何改动。 */
#if 0    /* ===== A 车配置 ===== */
#define SHOULDER_REVERSED     false
#else    /* ===== B 车配置 ===== */
#define SHOULDER_REVERSED     true
#endif

/* 夹爪舵机安装方向按车分套（2026/10/23 B 车实测：装反）：两车互为镜像。
 * reversed 翻转的是**整条通道**——○ 两档 / 动作关键帧的开合 / 末端抖动 /
 * 左摇杆点动一起镜像物理方向；开/合两档角（gripper.h）两车共用，
 * 翻转后物理开合自动对齐，上层无需改动。 */
#if 1    /* ===== A 车配置 ===== */
#define GRIPPER_REVERSED      false
#else    /* ===== B 车配置 ===== */
#define GRIPPER_REVERSED      true
#endif

    /* [0] 夹爪（SERVO_GRIPPER，PB9/TIM4_CH4，SG90）
     * 限位为舵机全行程 [0°,180°]：曾用 [15°,165°] 边距，闭合位够不到。
     * 开/合两档命令角见 gripper.h（60/120，占位待标定），90° 为中位脉宽。
     * 安装方向按车分套（见上方 GRIPPER_REVERSED）：A 车 false / B 车 true。 */
    {
        TIM_CHANNEL_4,
        500,
        2500,
        0.0f,
        180.0f,
        90.0f,
        GRIPPER_REVERSED
    },

    /* [1] 肩关节：第一段连杆，臂长 11cm（SERVO_LINK_1，PB7/TIM4_CH2，SG90）。
     * 安装方向按车分套（见上方 SHOULDER_REVERSED）：A 车 false / B 车 true。
     * 两端各留 15° 不达极限（域 [15°,165°]，90° 仍为中位脉宽 1500µs）。 */
    {
        TIM_CHANNEL_2,
        500,
        2500,
        15.0f,
        165.0f,
        90.0f,
        SHOULDER_REVERSED
    },

    /* [2] 肘关节：第二段连杆，臂长 8.5cm（SERVO_LINK_2，PB8/TIM4_CH3，SG90）。
     * 命令角 = 相对肩连杆的带符号夹角（肘上抬方向为正增加，与 arm_ik 一致）。
     * 域 [-120°, 60°]、初始 -30°：2026/10/17 实测零位标定——中值（1500µs）时
     * 肘连杆绝对仰角 60°（肩 90° + 相对 -30°），即"距 +Z 向 -X 偏 30°"。
     * 域宽仍 180°，脉宽映射不变，物理臂在改域前后位置完全一致。
     * 实测安装方向相反，reversed 翻转（只镜像物理方向，不改变软件角度约定）。 */
    {
        TIM_CHANNEL_3,
        500,
        2500,
        -120.0f,
        60.0f,
        -30.0f,
        true
    },

    /* [3] 底座：绕 Z 轴旋转（SERVO_BASE_Z，PB6/TIM4_CH1，MG995）
     * 命令角 = 相对 +X 轴、绕 +Z 逆时针的方位角（右手系，X 为车头反方向）。
     * 域 [90°,270°]、初始 180°（1500µs = 臂朝车头，2026/10/17 实测零位）。
     * 两端 500µs / 2500µs 各对应哪一侧以关键帧验收为准：2026/10/21 实车确认
     *   90° = 500µs = 篮子侧端；270° = 2500µs = 工作侧端
     * 关键帧用户角按 cmd = 270 − user 换算（user 0=工作侧端，见 arm_keyframe.h）。 */
    {
        TIM_CHANNEL_1,
        500,
        2500,
        90.0f,
        270.0f,
        180.0f,
        false
    }
};

/* 软件记录的是指令位置；普通三线舵机没有实际角度反馈。 */
static float commanded_angle_deg[SERVO_COUNT];
static bool commanded_angle_valid[SERVO_COUNT];

/* 逐路有力状态：卸力后该路拒绝写入、命令角记录保持不变，恢复时据此重发
 * 脉宽即可回到卸力前姿态。
 * 逐路（而不是一个全局标志）是"脱离"动作需要的：大臂（肩）先给力撑直，底座与
 * 其余关节仍保持卸力，随后再逐路给力——这样恢复时不至于整条臂一起弹回去。 */
static bool torque_enabled[SERVO_COUNT] = { true, true, true, true };

bool servo_set_torque_one(ServoId_t servo, bool enabled)
{
    if ((uint32_t)servo >= (uint32_t)SERVO_COUNT)
    {
        return false;
    }
    if (torque_enabled[servo] == enabled)
    {
        return true;   // 幂等：状态已符合，避免每周期重发脉宽
    }

    torque_enabled[servo] = enabled;

    if (!enabled)
    {
        /* CCR 清零：PWM1 模式下输出恒为低，20ms 周期内无脉冲，舵机卸力。
         * 直接写 CCR 而不走 servo_set_pulse，避免污染命令角记录。 */
        __HAL_TIM_SET_COMPARE(&htim4, servo_config[servo].channel, 0);
    }
    else if (commanded_angle_valid[servo])
    {
        /* 按记录的命令角重新发脉冲（此时 set_pulse 已允许写入） */
        (void)servo_set_angle(servo, commanded_angle_deg[servo]);
    }

    return true;
}

bool servo_is_torque_one(ServoId_t servo)
{
    if ((uint32_t)servo >= (uint32_t)SERVO_COUNT)
    {
        return false;
    }
    return torque_enabled[servo];
}

void servo_set_torque_enabled(bool enabled)
{
    /* 四路状态都已符合就直接返回（恢复时不会重复重发脉宽） */
    bool all_match = true;
    for (uint32_t i = 0; i < (uint32_t)SERVO_COUNT; i++)
    {
        if (torque_enabled[i] != enabled)
        {
            all_match = false;
            break;
        }
    }
    if (all_match)
    {
        return;
    }

    for (uint32_t i = 0; i < (uint32_t)SERVO_COUNT; i++)
    {
        (void)servo_set_torque_one((ServoId_t)i, enabled);
    }
}

bool servo_is_torque_enabled(void)
{
    for (uint32_t i = 0; i < (uint32_t)SERVO_COUNT; i++)
    {
        if (!torque_enabled[i])
        {
            return false;
        }
    }
    return true;
}

bool servo_set_pulse(ServoId_t servo, uint16_t pulse_us)
{
    /* 检查舵机编号是否合法 */
    if ((uint32_t)servo >= (uint32_t)SERVO_COUNT)
    {
        return false;
    }

    /* 该路卸力时拒绝写入：点动/动作/夹爪切换都不得悄悄重新上力 */
    if (!torque_enabled[servo])
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

bool servo_get_angle_range(ServoId_t servo, float *min_deg, float *max_deg)
{
    if ((uint32_t)servo >= (uint32_t)SERVO_COUNT ||
        min_deg == NULL || max_deg == NULL)
    {
        return false;
    }
    /* 点动限幅用：命令角域以配置表为唯一数据源，避免两处各记一份后失同步 */
    *min_deg = servo_config[servo].min_angle_deg;
    *max_deg = servo_config[servo].max_angle_deg;
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
