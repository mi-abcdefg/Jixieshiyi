//
// 机械臂末端点动（柱坐标）：十字键长按 → 末端上下 / 前后平移
//
// 与 arm_manual（关节空间点动）并列的两条手动通道：
//   - 摇杆  → arm_manual  ：每个轴直接驱动一个关节（挖掘机式，所见即所得）；
//   - 十字键 → 本模块     ：直接给末端一个小位移（上下 / 径向），内部做
//                           正解 → 增量 → 逆解 → 关节增量限速 → 下发，
//                           末端沿直线走，不绕弧。
//
// 坐标：末端到"底座旋转轴 + 肩关节高度"参考点的柱坐标 (θ, r, z)，θ 保持不变
// （底座不动），夹爪不参与。z 向上为正，r 为水平伸出量。
//
// 【为什么要限速】二连杆在"全伸/全折"附近，末端小的位移会放大成很大的关节
// 角速度（实测飙到 145°/s 量级）。直接下发逆解结果会甩臂，所以每个关节每拍
// 的增量都夹在点动快速档角速度（arm_manual.h）以内——宁可末端轨迹偏离直线，
// 也不让关节超速（交接文档踩坑 11）。
//
// 【标定项】命令角 ↔ 几何角的换算：
//   肩：几何仰角 = 命令角 − ARM_ENDPOINT_SHOULDER_OFFSET_DEG（默认 90，依据
//       "肩用户角 0 = 上推极限 = 竖直" ⇒ 命令角 180 = 仰角 90°）；
//   肘：命令角本身就是"相对肩连杆的带符号夹角"，与 arm_ik 的 link2 同义，不用偏移。
//   若实车按住 ↑ 末端反而下降/偏移，说明这条换算不对：改上面那个偏移宏一行。
//

#ifndef CRTC2026_ROBOTA_ARM_ENDPOINT_H
#define CRTC2026_ROBOTA_ARM_ENDPOINT_H

#pragma once
#include <stdint.h>
#include <stdbool.h>

/* ------------------------------ 点动参数 ------------------------------ */
/* 末端平移速度 [m/s]。臂展约 19.5cm，0.04 m/s 走完全程约 5s，适合微调；
 * 嫌慢就加大（关节增量限速会兜住超速，但越大越容易贴到限速上）。 */
#define ARM_ENDPOINT_Z_MPS      0.04f   // 上下（+ = 上升）
#define ARM_ENDPOINT_R_MPS      0.04f   // 径向（+ = 伸出/向前，− = 收回/向后）

/* 肩关节命令角 → 几何仰角 的偏移（见文件头"标定项"） */
#define ARM_ENDPOINT_SHOULDER_OFFSET_DEG   90.0f

/* 单个关节每拍的增量上限直接沿用点动快速档角速度（速度数据源只有一处） */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化（复位内部时间戳；须在 servo_init() 之后调用）
 */
void arm_endpoint_init(void);

/**
 * @brief 末端点动一拍（建议与 ArmTask 同周期 20ms 调用）
 *
 * 输入是"方向"而不是"位置"：本模块每拍从当前命令角重新做一次正解，所以
 * 贴到行程边界就自然停住，不会像积分目标那样越攒越远；中途松开再按也不会跳。
 *
 * @param now_ms 当前系统时间 [ms]
 * @param dz_dir 末端上下方向 -1..1（+ = 上升）
 * @param dr_dir 末端径向方向 -1..1（+ = 远离底座）
 * @return true 本拍有下发；false 无输入 / 第一拍只记录时间 / 解算或写入失败
 * @note  dz = dr = 0 表示"空闲"：只复位内部时间戳，不下发任何东西。
 */
bool arm_endpoint_jog(uint32_t now_ms, float dz_dir, float dr_dir);

#ifdef __cplusplus
}
#endif

#endif //CRTC2026_ROBOTA_ARM_ENDPOINT_H
