//
// 机械臂末端抖动实现（时间驱动的振荡：正弦摆肘 + 半周期切夹爪）
//

#include "arm_shake.h"

#include "arm_control.h"
#include "gripper.h"
#include "servo_driver.h"

#include <math.h>

namespace {

/* 3.14159265f：抖动相位计算用 */
constexpr float kPi = 3.14159265f;

/* 一个周期的时长 [ms]（由频率换算，取整） */
constexpr uint32_t kPeriodMs =
    static_cast<uint32_t>(1000.0f / ARM_SHAKE_FREQ_HZ + 0.5f);
/* 总时长 = 整周期数 × 周期（保证收尾时肘恰好回中） */
constexpr uint32_t kDurationMs =
    static_cast<uint32_t>(ARM_SHAKE_CYCLES * kPeriodMs);

bool s_running = false;
uint32_t s_start_ms = 0U;
float s_elbow_center = 0.0f;        // 触发时的肘命令角（摆动中心）
GripperState_t s_gripper_state = GRIPPER_UNDEF;   // 触发前的夹爪状态
float s_gripper_angle = 0.0f;       // 触发前的夹爪命令角（状态未定义时恢复用）

/* 夹爪恢复触发前的角度/档位（卸力中写入会被拒绝，属可接受结果） */
void restore_gripper(void)
{
    if (s_gripper_state != GRIPPER_UNDEF) {
        (void)gripper_set(s_gripper_state);
    } else {
        (void)gripper_set_angle(s_gripper_angle);
    }
}

}   // namespace

void arm_shake_init(void)
{
    s_running = false;
}

bool arm_shake_start(uint32_t now_ms)
{
    if (s_running) {
        return false;
    }

    ArmPose_t pose;
    if (!arm_get_pose(&pose)) {
        return false;
    }

    s_elbow_center = pose.link2_deg;
    s_gripper_state = gripper_get_state();
    s_gripper_angle = pose.gripper_deg;
    s_start_ms = now_ms;
    s_running = true;

    /* 第一拍即进入"闭合档"：先夹住再开始摆 */
    (void)gripper_set_angle(GRIPPER_SHAKE_CLOSE_DEG);
    return true;
}

void arm_shake_update(uint32_t now_ms)
{
    if (!s_running) {
        return;
    }

    const uint32_t elapsed = now_ms - s_start_ms;

    /* 到点收尾：肘回中、夹爪恢复，回空闲 */
    if (elapsed >= kDurationMs) {
        ArmPose_t pose;
        if (arm_get_pose(&pose)) {
            pose.link2_deg = s_elbow_center;
            (void)arm_set_pose_masked(&pose, ARM_JOINT_ELBOW);
        }
        restore_gripper();
        s_running = false;
        return;
    }

    /* 肘：绕触发值正弦摆动；命令角夹到肘的量程内（servo_set_angle 内部
     * 也会限幅，这里夹是避免把大幅越界的值交给下层） */
    const float phase = 2.0f * kPi * ARM_SHAKE_FREQ_HZ *
                        static_cast<float>(elapsed) / 1000.0f;
    float elbow_cmd = s_elbow_center +
                      ARM_SHAKE_ELBOW_AMP_DEG * sinf(phase);

    ArmPose_t pose;
    if (!arm_get_pose(&pose)) {
        s_running = false;
        return;
    }
    float el_min = 0.0f;
    float el_max = 0.0f;
    if (!servo_get_angle_range(SERVO_LINK_2, &el_min, &el_max)) {
        s_running = false;
        return;
    }
    if (elbow_cmd > el_max) {
        elbow_cmd = el_max;
    } else if (elbow_cmd < el_min) {
        elbow_cmd = el_min;
    }
    pose.link2_deg = elbow_cmd;
    (void)arm_set_pose_masked(&pose, ARM_JOINT_ELBOW);

    /* 夹爪：每半周期切一次——前半周期"抖动闭合档"，后半周期张开 */
    const uint32_t half = elapsed % kPeriodMs;
    const bool hold_phase = (half < kPeriodMs / 2U);
    if (hold_phase) {
        (void)gripper_set_angle(GRIPPER_SHAKE_CLOSE_DEG);
    } else {
        (void)gripper_set(GRIPPER_OPEN);
    }
}

void arm_shake_cancel(void)
{
    if (!s_running) {
        return;
    }
    restore_gripper();
    s_running = false;
}

bool arm_shake_is_running(void)
{
    return s_running;
}
