//
// 机械臂末端点动实现（平面柱坐标：正解 → 增量 → 工作区夹取 → 逆解 → 限速）
//
// 解算只涉及肩/肘两支舵机（平面二连杆，底座保持不动）。
// 工作区夹取在本文件是唯一所有者；arm_ik_solve 一次返回肘上/肘下两支，
// 这里取离当前关节角更近的一支，保证手臂不跨支。
//

#include "arm_endpoint.h"

#include "arm_control.h"
#include "arm_ik.h"
#include "arm_manual.h"     // ARM_MANUAL_*_MAX_DPS：关节增量限速的上限
#include "servo_driver.h"

#include <math.h>

namespace {

uint32_t s_last_ms = 0U;
bool s_has_last = false;

/* 把目标 (r, z) 拉回可达工作区：
 *   - r 不能小于"完全折起"的半径（近轴区逆解无解/病态）；
 *   - 到肩关节的距离 ρ = √(r²+z²) 不能超过 |L1|+|L2|（全伸）。
 * 只按比例缩放 ρ，方向不变——即"沿当前方向贴边停住"。 */
void clamp_to_workspace(float &r, float &z)
{
    float r_min = 0.0f;
    float r_max = 0.0f;
    arm_ik_get_reach(&r_min, &r_max);

    if (r < r_min) {
        r = r_min;      // 别越过底座轴（越过会让逆解镜像，末端跳到另一边）
    }

    const float rho = sqrtf(r * r + z * z);
    if (rho > 1e-6f && rho > r_max) {
        const float scale = r_max / rho;
        r *= scale;
        z *= scale;
    }
}

/* 单关节朝目标走一步，但每拍最多走 max_step（不追求"落在目标上"：
 * 目标是逐拍刷新的移动点，走到位反而会放大奇异区的关节速度）。 */
void step_limited(float &cur_deg, float tgt_deg, float max_step)
{
    const float delta = tgt_deg - cur_deg;
    if (delta > max_step) {
        cur_deg += max_step;
    } else if (delta < -max_step) {
        cur_deg -= max_step;
    } else {
        cur_deg = tgt_deg;
    }
}

}   // namespace

void arm_endpoint_init(void)
{
    s_last_ms = 0U;
    s_has_last = false;
}

bool arm_endpoint_jog(uint32_t now_ms, float dz_dir, float dr_dir)
{
    /* 空闲：复位时间戳链，下一次按下不会拿"很久以前"的 dt 走一大步 */
    if (dz_dir == 0.0f && dr_dir == 0.0f) {
        s_has_last = false;
        return false;
    }

    /* 第一拍只记时间（此时还没有可靠的 dt） */
    if (!s_has_last) {
        s_last_ms = now_ms;
        s_has_last = true;
        return false;
    }

    const uint32_t dt_ms = now_ms - s_last_ms;
    s_last_ms = now_ms;

    /* dt 上限保护：长时间阻塞后不做大步积分（与 arm_manual 同一策略） */
    const float dt_s = (dt_ms > ARM_MANUAL_MAX_DT_MS)
                       ? (float)ARM_MANUAL_MAX_DT_MS / 1000.0f
                       : (float)dt_ms / 1000.0f;
    if (dt_s <= 0.0f) {
        return false;
    }

    /* ---------------- 当前命令角 → 几何角 → 正解 ---------------- */
    ArmPose_t pose;
    if (!arm_get_pose(&pose)) {
        return false;
    }

    float r_m = 0.0f;
    float z_m = 0.0f;
    arm_ik_forward(pose.link1_deg - ARM_ENDPOINT_SHOULDER_OFFSET_DEG,
                   pose.link2_deg, &r_m, &z_m);

    /* ---------------- 给末端一个小位移，夹回工作区（唯一夹取点） ---------------- */
    r_m += dr_dir * ARM_ENDPOINT_R_MPS * dt_s;
    z_m += dz_dir * ARM_ENDPOINT_Z_MPS * dt_s;
    clamp_to_workspace(r_m, z_m);

    /* ---------------- 逆解：两支解取离当前关节角更近的一支 ----------------
     * 臂完全可能停在"肘下"那一支（摇杆把肘点到正角度就是这样），
     * 固定取某一支会让肘翻到另一侧，表现为"四个方向键都变成同一个动作"。
     * 比较在命令角域进行（肩要加回几何↔命令的偏移）。 */
    ArmIkBranches ik;
    if (!arm_ik_solve(r_m, z_m, &ik)) {
        return false;
    }

    const float up_link1 = arm_clamp_angle(
        SERVO_LINK_1, ik.link1_up_deg + ARM_ENDPOINT_SHOULDER_OFFSET_DEG);
    const float up_link2 = arm_clamp_angle(SERVO_LINK_2, ik.link2_up_deg);
    const float dn_link1 = arm_clamp_angle(
        SERVO_LINK_1, ik.link1_down_deg + ARM_ENDPOINT_SHOULDER_OFFSET_DEG);
    const float dn_link2 = arm_clamp_angle(SERVO_LINK_2, ik.link2_down_deg);

    const float up_dist = fabsf(up_link1 - pose.link1_deg) +
                          fabsf(up_link2 - pose.link2_deg);
    const float dn_dist = fabsf(dn_link1 - pose.link1_deg) +
                          fabsf(dn_link2 - pose.link2_deg);

    /* ---------------- 关节增量限速（防奇异区甩臂），只写肩/肘 ---------------- */
    step_limited(pose.link1_deg, (dn_dist < up_dist) ? dn_link1 : up_link1,
                 ARM_MANUAL_SHOULDER_MAX_DPS * dt_s);
    step_limited(pose.link2_deg, (dn_dist < up_dist) ? dn_link2 : up_link2,
                 ARM_MANUAL_ELBOW_MAX_DPS * dt_s);

    return arm_set_pose_masked(&pose, ARM_JOINT_SHOULDER | ARM_JOINT_ELBOW);
}
