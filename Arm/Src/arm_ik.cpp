//
// 机械臂平面二连杆运动学实现（纯数学，无硬件依赖）
//

#include "arm_ik.h"

#include <math.h>

/* π/180 与 180/π：几何角（度）↔ 弧度 */
#define ARM_IK_DEG2RAD (3.14159265f / 180.0f)
#define ARM_IK_RAD2DEG (180.0f / 3.14159265f)

/* 完全折起附近逆解病态，可达边界往内收一点 */
#define ARM_IK_MIN_RHO_MARGIN_M 0.002f

/* ρ 低于此值视为目标落在肩关节转轴上（无解） */
#define ARM_IK_MIN_RHO_M 0.001f

void arm_ik_get_reach(float *r_min_m, float *r_max_m)
{
    const float l1 = ARM_IK_LINK1_M;
    const float l2 = ARM_IK_LINK2_M;

    if (r_min_m != 0)
    {
        *r_min_m = fabsf(l1 - l2) + ARM_IK_MIN_RHO_MARGIN_M;
    }
    if (r_max_m != 0)
    {
        *r_max_m = (l1 + l2) - ARM_IK_REACH_MARGIN_M;
    }
}

bool arm_ik_solve(float r_m, float z_m, ArmIkBranches *out)
{
    if (out == 0)
    {
        return false;
    }

    const float l1 = ARM_IK_LINK1_M;
    const float l2 = ARM_IK_LINK2_M;

    /* 肩→目标的距离与仰角（平面极坐标） */
    const float rho = sqrtf(r_m * r_m + z_m * z_m);
    if (rho < ARM_IK_MIN_RHO_M)
    {
        return false;
    }
    const float psi = atan2f(z_m, r_m);

    /*
     * 标准二连杆解（余弦定理）：
     *   cos(a2) = (ρ² − L1² − L2²) / (2·L1·L2)，a2 ∈ [0, π]
     *   第一连杆绝对角 a1 = ψ ± φ，φ = 第二连杆相对"肩→腕连线"的偏角：
     *   cos(φ) = (ρ² + L1² − L2²) / (2·L1·ρ)
     * 两支解：肘上 a1 = ψ + φ 且 a2 取负；肘下 a1 = ψ − φ 且 a2 取正。
     * 用 atan2(sqrt(1−c²), c) 形式：ρ 贴边界导致 |c| 微越界时
     * sqrt 项按 0 处理，解自动退化为全伸/全折，天然无 NaN。
     */
    const float cos_a2 = (rho * rho - l1 * l1 - l2 * l2) / (2.0f * l1 * l2);
    const float sin_a2 = sqrtf((1.0f - cos_a2 * cos_a2) > 0.0f
                                   ? (1.0f - cos_a2 * cos_a2) : 0.0f);
    const float a2 = atan2f(sin_a2, cos_a2);

    const float cos_phi = (rho * rho + l1 * l1 - l2 * l2) / (2.0f * l1 * rho);
    const float sin_phi = sqrtf((1.0f - cos_phi * cos_phi) > 0.0f
                                    ? (1.0f - cos_phi * cos_phi) : 0.0f);
    const float phi = atan2f(sin_phi, cos_phi);

    out->link1_up_deg   = (psi + phi) * ARM_IK_RAD2DEG;
    out->link2_up_deg   = -a2 * ARM_IK_RAD2DEG;
    out->link1_down_deg = (psi - phi) * ARM_IK_RAD2DEG;
    out->link2_down_deg =  a2 * ARM_IK_RAD2DEG;
    return true;
}

void arm_ik_forward(float link1_deg, float link2_deg,
                    float *r_m, float *z_m)
{
    const float a1  = link1_deg * ARM_IK_DEG2RAD;
    const float a12 = (link1_deg + link2_deg) * ARM_IK_DEG2RAD;

    if (r_m != 0)
    {
        *r_m = ARM_IK_LINK1_M * cosf(a1) + ARM_IK_LINK2_M * cosf(a12);
    }
    if (z_m != 0)
    {
        *z_m = ARM_IK_LINK1_M * sinf(a1) + ARM_IK_LINK2_M * sinf(a12);
    }
}
