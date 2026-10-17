//
// 机械臂逆运动学实现（纯数学，无硬件依赖）
//
#include "arm_ik.h"

#include <math.h>

/* π/180：角速度类换算用，避免引入完整 libm 的常量接口差异 */
#define ARM_IK_PI 3.14159265f
#define ARM_IK_DEG2RAD (ARM_IK_PI / 180.0f)
#define ARM_IK_RAD2DEG (180.0f / ARM_IK_PI)

/* 非 |L1-L2| 时二连杆无解；留一点余量避免除零/浮点边界抖动 */
#define ARM_IK_MIN_RHO_MARGIN_M 0.002f

static float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

void arm_ik_get_reach(float *r_min_m, float *r_max_m)
{
    const float l1 = ARM_IK_LINK1_M;
    const float l2 = ARM_IK_LINK2_M;

    if (r_min_m != 0)
    {
        /* 完全伸直 |L1-L2| 附近是奇异位形，往内收一点 */
        *r_min_m = fabsf(l1 - l2) + ARM_IK_MIN_RHO_MARGIN_M;
    }
    if (r_max_m != 0)
    {
        *r_max_m = (l1 + l2) - ARM_IK_REACH_MARGIN_M;
    }
}

bool arm_ik_solve(float theta_deg, float r_m, float z_m,
                  ArmJointAngles_t *out)
{
    if (out == 0 ||
        !isfinite(theta_deg) || !isfinite(r_m) || !isfinite(z_m))
    {
        return false;
    }

    float r_min;
    float r_max;
    arm_ik_get_reach(&r_min, &r_max);

    /* 平面二连杆解的是到肩关节的距离 ρ = sqrt(r² + z²)，
     * 先把给定的 r 夹到可达范围，z 在上层限幅（这里不再二次夹取，
     * 保证 z 夹取规则只有一个所有者）。 */
    const float r_clamped = clampf(r_m, r_min, r_max);

    const float rho_sq = r_clamped * r_clamped + z_m * z_m;
    const float rho = sqrtf(rho_sq);

    /*
     * 余弦定理求第二连杆相对角 a2：
     *   cos(a2) = (ρ² - L1² - L2²) / (2·L1·L2)
     * ρ 贴近边界时结果可能略微越过 ±1，必须夹回定义域，否则 acos 产出 NaN。
     */
    const float cos_a2 = clampf(
        (rho_sq - ARM_IK_LINK1_M * ARM_IK_LINK1_M -
         ARM_IK_LINK2_M * ARM_IK_LINK2_M) /
        (2.0f * ARM_IK_LINK1_M * ARM_IK_LINK2_M),
        -1.0f, 1.0f);
    const float a2 = acosf(cos_a2);   // ∈ [0, π]

    /*
     * 第一连杆绝对角 a1 = ψ ± φ，其中 ψ 是肩→腕连线的仰角，
     * φ 是第一连杆相对该连线的偏角。肘上解取 a1 = ψ + φ（a2 取负），
     * 两处符号必须成对，否则正逆解不自洽。
     */
    const float psi = atan2f(z_m, r_clamped);
    const float cos_phi = clampf(
        (rho_sq + ARM_IK_LINK1_M * ARM_IK_LINK1_M -
         ARM_IK_LINK2_M * ARM_IK_LINK2_M) /
        (2.0f * ARM_IK_LINK1_M * rho),
        -1.0f, 1.0f);
    const float phi = acosf(cos_phi);

#if ARM_IK_ELBOW_UP
    const float a1 = psi + phi;
    const float a2_signed = -a2;
#else
    const float a1 = psi - phi;
    const float a2_signed = a2;
#endif

    out->base_z_deg = theta_deg + ARM_IK_BASE_Z_OFFSET_DEG;
    out->link1_deg = a1 * ARM_IK_RAD2DEG;
    out->link2_deg = a2_signed * ARM_IK_RAD2DEG;
    return true;
}

void arm_ik_forward(float base_z_deg, float link1_deg, float link2_deg,
                    float *theta_deg, float *r_m, float *z_m)
{
    const float a1 = link1_deg * ARM_IK_DEG2RAD;
    const float a12 = (link1_deg + link2_deg) * ARM_IK_DEG2RAD;

    if (theta_deg != 0)
    {
        *theta_deg = base_z_deg - ARM_IK_BASE_Z_OFFSET_DEG;
    }
    if (r_m != 0)
    {
        *r_m = ARM_IK_LINK1_M * cosf(a1) + ARM_IK_LINK2_M * cosf(a12);
    }
    if (z_m != 0)
    {
        *z_m = ARM_IK_LINK1_M * sinf(a1) + ARM_IK_LINK2_M * sinf(a12);
    }
}
