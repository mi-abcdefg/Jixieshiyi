//
// 麦克纳姆轮运动学解算实现
// Created by MI on 2026/9/13.
//
// 计算顺序（每一步的目的都不一样，改之前先看注释）：
//   1. 混算：车体 (vx, vy, omega) → 四轮线速度；
//   2. 归一化：四轮按"绝对值最大"等比缩到满量程以内（保住方向）；
//   3. 档位缩放：整体乘 throttle（中速档 0.8）——放在归一化之后，
//      这样"满杆+饱和"的组合动作在中速档也是 80%，而不是被归一化吃掉；
//   4. 起步下限：四轮最快的仍低于起步占空比时整组等比放大（保方向、不叠加）；
//   5. 二次限幅：兜底夹到 ±满量程。
// 顺序上的关键点：3 在 4 之前——否则中速档会把起步下限也乘掉（0.8×60% = 48%，
// 又回到"推了不动"）。
//

#include "mecanum_kinematic.h"

#include <cmath>

void mecanum_get_wheel_speeds(float vx,
                              float vy,
                              float omega,
                              float throttle_scale,
                              float wheel_speeds[4])
{
    if (wheel_speeds == nullptr)
    {
        return;
    }

    // 输入含非有限值（NaN/Inf）按停车处理：上层算出 NaN 一般是未初始化或除零，
    // 传下去会变成 int 强转的未定义行为，直接当"没有指令"最安全。
    if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(omega) ||
        !std::isfinite(throttle_scale))
    {
        for (uint8_t i = 0U; i < 4U; ++i)
        {
            wheel_speeds[i] = 0.0f;
        }
        return;
    }

    // 麦克纳姆轮的运动学关系（**约定见头文件**：x 向前、y 向右、omega 顺时针为正）：
    //   FL = vx + vy + k·ω    FR = vx − vy − k·ω
    //   BL = vx − vy + k·ω    BR = vx + vy − k·ω
    // 注意 FL/BR 的 vy 同号、ω 反号，FR/BL 反过来——这是麦轮"对角同向"的必然结果，
    // 任一项符号写错都会变成斜着走或轮子互相打滑。轮序 [FL, FR, BL, BR] 与
    // motor_set_wheel_speed() 的参数顺序一致。
    const float k = MECANUM_OMEGA_SCALE * (MECANUM_CHASSIS_LENGTH_M + MECANUM_CHASSIS_WIDTH_M);

    wheel_speeds[MECANUM_WHEEL_FL] = vx + vy + k * omega;
    wheel_speeds[MECANUM_WHEEL_FR] = vx - vy - k * omega;
    wheel_speeds[MECANUM_WHEEL_BL] = vx - vy + k * omega;
    wheel_speeds[MECANUM_WHEEL_BR] = vx + vy - k * omega;

    // 速度归一化：必须按"绝对值最大"的轮速做等比缩放。
    // 只取带符号的最大值时，倒车/反向旋转时最大轮速是负数，饱和判断会失效，
    // 导致某个轮子单独被限幅、运动方向发生偏斜。
    float max_speed = 0.0f;
    for (uint8_t i = 0U; i < 4U; ++i)
    {
        const float abs_speed = std::fabs(wheel_speeds[i]);
        if (abs_speed > max_speed)
        {
            max_speed = abs_speed;
        }
    }

    if (max_speed > MECANUM_MAX_SPEED)
    {
        const float scale = MECANUM_MAX_SPEED / max_speed;
        for (uint8_t i = 0U; i < 4U; ++i)
        {
            wheel_speeds[i] *= scale;
        }
    }

    // 档位缩放（中速档 0.8）：缩整条速度曲线，但**必须在起步下限之前**，
    // 否则下限也被缩掉（48% 占空比），中速档小油门就又不转了。
    for (uint8_t i = 0U; i < 4U; ++i)
    {
        wheel_speeds[i] *= throttle_scale;
    }

    // 阻力扭矩补偿（前馈）：把"低于起步占空比就完全不转"的死区补掉。
    // 做法：看四轮里**最快的那一轮**，若它还没到起步下限，就把整组轮速
    // **等比放大**到下限；已经够快就不动。
    //   · 等比放大而不是逐轮夹：四轮的比值就是运动方向，逐轮夹会把"斜着慢走"
    //     夹成"正着走"（只要有一轮被夹，方向就歪了）；
    //   · 也不是逐轮叠加：叠加会让整条曲线整体上移，满速前就超调、固定慢速档
    //     被顶到 100% 占空比，档位梯度就没了；
    //   · 整组都是 0（摇杆回中）时不动，车不会自己爬行。
    float max_abs = 0.0f;
    for (uint8_t i = 0U; i < 4U; ++i)
    {
        const float abs_speed = std::fabs(wheel_speeds[i]);
        if (abs_speed > max_abs)
        {
            max_abs = abs_speed;
        }
    }

    const float comp_min = MECANUM_FRICTION_COMP_MPS +
                           MECANUM_FRICTION_KV_MPS *
                           (max_abs / MECANUM_MAX_SPEED);
    if (max_abs > 0.0f && max_abs < comp_min)
    {
        const float scale = comp_min / max_abs;
        for (uint8_t i = 0U; i < 4U; ++i)
        {
            wheel_speeds[i] *= scale;
        }
    }

    // A 车四轮前馈特调（等差数列，逐轮修正起步灵敏度差，宏注释有调法）：
    //   FL = −1.5d / FR = −0.5d / BL = +0.5d / BR = +1.5d，中位数 = 0。
    // 只对"有指令"的轮子生效（带符号叠加，倒车同样成立）；0 速轮不动，
    // 车不会自己爬行。
    const float trim[4] = {
        -1.5f * MECANUM_COMP_TRIM_STEP_MPS,     // FL：最灵敏，减得最多
        -0.5f * MECANUM_COMP_TRIM_STEP_MPS,     // FR
        +0.5f * MECANUM_COMP_TRIM_STEP_MPS,     // BL
        +1.5f * MECANUM_COMP_TRIM_STEP_MPS,     // BR：最迟钝，加得最多
    };
    for (uint8_t i = 0U; i < 4U; ++i)
    {
        if (wheel_speeds[i] > 0.0f)
        {
            wheel_speeds[i] += trim[i];
        }
        else if (wheel_speeds[i] < 0.0f)
        {
            wheel_speeds[i] -= trim[i];
        }
    }

    // 补偿后的二次限幅：保证任何轮速都不超满量程（下游按满量程换算占空比，
    // 电机驱动虽对上限有夹取，但在这里夹齐可避免四个轮子被截断的程度不一致）。
    for (uint8_t i = 0U; i < 4U; ++i)
    {
        if (wheel_speeds[i] > MECANUM_MAX_SPEED)
        {
            wheel_speeds[i] = MECANUM_MAX_SPEED;
        }
        else if (wheel_speeds[i] < -MECANUM_MAX_SPEED)
        {
            wheel_speeds[i] = -MECANUM_MAX_SPEED;
        }
    }
}
