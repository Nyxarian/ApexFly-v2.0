/**
 * @file vertical_est.h
 * @brief 高度/垂速状态估计（差分版 + 倾斜补偿）对外接口。
 *
 * 本模块只负责把激光测距的原始值变成"高度"和"垂速"两个可用的量，
 * **不产生任何控制量、不写电机、不参与姿态**。定高的控制律在别处（将来是 vertical_control）。
 *
 * 输出单位：高度用厘米，垂速用厘米每秒（向上为正）。
 *
 * ── 谁驱动它、谁看它 ──────────────────────────────────────────
 * 驱动：`motor_test.c` 的控制环每拍调用一次 `VerticalEst_Update()`（250Hz）。
 *       （H5 之前它挂在 tofTask 里，H4 合并 H5 时搬进来了，因为倾斜补偿需要姿态角。）
 * 查看：`tof.c` 的低优先级任务每 500ms 调一次 `VerticalEst_GetState()` 打印。
 *       控制环里**禁止打印**（一行日志约 10ms，会吃掉两拍 4ms 预算），所以打印留在那边。
 *       ⚠️ 一写一读，结构体拷贝不是原子的，打印出来的 h 与 vz 理论上可能来自相邻两拍。
 *       对诊断没有影响，但知道这件事比不知道好。
 *
 * ── 用之前必须先看 valid ──────────────────────────────────────
 * 测不到数据时本模块会**冻结**上一次的高度与垂速，并把 valid 置 false。
 * 也就是说：冻结期间 h/vz 不是"当前值"，而是"最后可信值"。
 * **任何使用 h/vz 的地方都必须先判断 valid**，否则会拿着死数当活数用。
 */

#ifndef VERTICAL_EST_H
#define VERTICAL_EST_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 垂速低通系数：vz_est += (vz_raw − vz_est) × ALPHA。
 *
 * 1/4 表示每来一条新数据朝新值靠四分之一。12Hz 下时间常数约 333ms：
 * 静止时垂速噪声从约 2cm/s（1σ）降到约 0.4cm/s，代价是约 1/3 秒的滞后。
 * 调大更灵敏（1/2 → 约 166ms、噪声约 0.7cm/s），调小更稳。
 * 这是 H3 特意选的"更稳"档；H16 换成加速度计融合后这个滤波就没用了。
 */
#define VERTICAL_EST_VZ_LPF_ALPHA 0.25f

/**
 * @brief 距上一次**用掉的样本**超过这么多秒，就判定估计不可用（valid 置 false）。
 *
 * 0.300 秒 ≈ 3.6 个采样周期（12Hz）。取值理由：
 *   - 单次丢数据（约 83ms）不该判失效，否则 valid 会每丢一拍就抖一下，
 *     上层的控制环会被这种抖动反复打断；
 *   - 连续丢三四次（约 330ms）才判失效。
 * 这个计时是**本模块自己累加的**，不依赖 ToF 任务是否还在跑，
 * 所以它同时能兜住"ToF 任务被饿死、读数一直不更新"这种情况。
 * 将来 H12 的"读数失效就交还手动"复用同一个数值。
 */
#define VERTICAL_EST_MAX_AGE_S 0.300f

/**
 * @brief 数据断了超过这么多秒才回来，就当"全新开始"，不做差分。
 *
 * 0.500 秒。两个作用：
 *   1. 数据断了一大段（比如 ToF 掉线重连）之后，不能用新读数去减旧高度——
 *      那会算出一个巨大的假垂速。实测重初始化后飙到过 +14.9cm/s，
 *      这种尖峰要是喂给控制环，就是一次真实的油门猛冲；
 *   2. 顺便让本模块**不需要外部告诉它"ToF 重新初始化了"**，
 *      少一条跨任务耦合（否则 tofTask 要 Reset、控制环在 Update，就是竞态）。
 */
#define VERTICAL_EST_RESTART_GAP_S 0.500f

/**
 * @brief 估计器的对外状态。
 */
typedef struct
{
    /* 估计高度，单位厘米。已做倾斜补偿，含约 0.8cm 的安装偏置
       （镜头在机架最下方，放在平面上读数约 1cm），没有做任何零点相减。
       注意：valid 为 false 时这个值是"最后一次可信值"，不是当前值。 */
    float height_cm;
    /* 估计垂速，单位厘米每秒，向上为正。同样：valid 为 false 时是冻结值。 */
    float vz_cms;
    /* true 表示这次估计可用。false 表示测不到或读数太旧，h/vz 已冻结。 */
    bool valid;
    /* 累计用掉的有效样本数，只用于诊断打印。 */
    uint32_t sample_count;
} VerticalEstState_t;

/**
 * @brief 初始化估计器，等价于 Reset。
 */
void VerticalEst_Init(void);

/**
 * @brief 清空全部状态，等下一个有效样本重新起步。
 *
 * 只在上电初始化、以及 IMU 重新初始化时由控制环调用。
 * **正常情况下不需要因为 ToF 掉线重连而调用**——那种情况由
 * VERTICAL_EST_RESTART_GAP_S 自动处理，见上面的说明。
 */
void VerticalEst_Reset(void);

/**
 * @brief 推进一拍。控制环每拍调用一次。
 *
 * 内部靠 ToF 的 valid_count 判断是否来了新数据，所以调用频率（12Hz 还是 250Hz）
 * 不影响任何参数——把 dt 累加起来、等真正来新数据时当作差分的时间分母。
 *
 * @param roll_deg 机体系横滚角，单位度。用于倾斜补偿。
 * @param pitch_deg 机体系俯仰角，单位度。用于倾斜补偿。
 * @param dt_s 距上一拍的时间，单位秒。正常情况下就是控制周期（约 0.004）。
 *             数值离谱时按标称值兜底，防任务被抢占后算出垃圾。
 */
void VerticalEst_Update(float roll_deg, float pitch_deg, float dt_s);

/**
 * @brief 取最近一次估计结果。只拷贝一个结构体，不做任何计算。
 * @param state 输出，不能为空。
 */
void VerticalEst_GetState(VerticalEstState_t *state);

#ifdef __cplusplus
}
#endif

#endif
