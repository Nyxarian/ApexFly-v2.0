/**
 * @file altitude_control.h
 * @brief 旧垂速环参数表。当前油门已改为直通映射，本模块不再写电机。
 *
 * hover_pwm_us 现在表示满杆油门上限，默认 1500us。最低油门固定 1000us。
 */

#ifndef ALTITUDE_CONTROL_H
#define ALTITUDE_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    /* 垂直速度比例增益，输出单位为微秒。 */
    float vz_kp;
    /* 垂直速度积分增益，输出单位为微秒。 */
    float vz_ki;
    /* 垂直速度微分增益，输出单位为微秒。 */
    float vz_kd;
    /* 满杆对应的最高油门，单位微秒。最低油门固定 1000。 */
    float hover_pwm_us;
} AltitudeControlParams_t;

typedef enum
{
    ALTITUDE_CONTROL_GROUND_IDLE = 0,
    ALTITUDE_CONTROL_TAKEOFF_RAMP,
    ALTITUDE_CONTROL_AIRBORNE,
    ALTITUDE_CONTROL_ESTIMATE_HOLD
} AltitudeControlState_t;

typedef struct
{
    /* 当前杆位生成的目标垂速，单位米每秒，向上为正。 */
    float target_vz_mps;
    /* 垂速 PID 对悬停 PWM 的修正量，单位微秒。 */
    float correction_us;
    /* 当前配置的悬停 PWM，单位微秒。 */
    float hover_pwm_us;
    /* 高度控制状态，用于黑匣子和调试输出。 */
    AltitudeControlState_t state;
} AltitudeControlDebug_t;

/** @brief 返回编译默认高度参数，只读。 */
const AltitudeControlParams_t *AltitudeControl_GetDefaultParams(void);

/** @brief 在线应用垂速 PID 和悬停 PWM，并清空控制器历史状态。 */
void AltitudeControl_ApplyParams(const AltitudeControlParams_t *params);

/** @brief 上锁、失联或重新解锁前复位高度控制状态。 */
void AltitudeControl_Reset(void);

/** @brief 读取最近一拍高度控制诊断数据。 */
void AltitudeControl_GetDebug(AltitudeControlDebug_t *debug);

/**
 * @brief 按杆位和垂直速度估计计算基础油门。
 * @param stick 遥控油门 0~1000。
 * @param z_m 离地高度，单位米。
 * @param vz_mps 垂直速度，单位米每秒，向上为正。
 * @param estimate_valid 卡尔曼健康且气压没有超时。
 * @param dt_s 控制周期，单位秒。
 * @param armed 已解锁。
 * @return 基础油门，单位微秒。
 */
uint16_t AltitudeControl_Update(uint16_t stick, float z_m, float vz_mps, bool estimate_valid, float dt_s, bool armed);

#ifdef __cplusplus
}
#endif

#endif
