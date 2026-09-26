/**
 * @file attitude_control.h
 * @brief 姿态自稳：三轴串级 PID 算出修正量，再交给混控。
 *
 * 本模块连接 cascade_pid 和 mixer，不读取 MPU6050，也不写 TIM8。
 * 控制任务提供机体系姿态角、角速度和基础油门，这里返回四路脉宽。
 *
 * 结构与老工程一致，每轴一份串级，一共 6 组独立 PID 参数：
 * 横滚角度/角速度、俯仰角度/角速度、偏航角度/角速度。
 * 运行时状态在 CascadePid_t 里，编译默认在 kDefaultPidParams。
 * 遥控在线调参走 PidTable，经 AttitudeControl_ApplyParams() 写进三轴串级。
 *
 * 每轴都走完整串级：外环角度 -> 目标角速度 -> 内环 -> 混控微秒。
 * 横滚俯仰外环吃机体系姿态角，目标角由遥控杆映射，满杆约 ±10°，再加 Trim。
 * 偏航外环吃磁航向相对松杆目标的最短角误差。
 * 偏航杆出死区时外环让路，内环直接跟杆角速度。默认表里 6 组 Kp 先全部为 0，限幅为 200。
 *
 * 送进混控的符号必须与实机测试一致，不要改 mixer.c 里的加减号：
 * roll_correction_us  = -Kp * roll_deg
 * pitch_correction_us = +Kp * pitch_deg
 *
 * 外环目标为 0 时，输出已经是 -测量。
 * 横滚：外环、内环输出直接进混控。
 * 俯仰：外环、内环输出都再取一次反，才能变成实机需要的正号。
 * 偏航：与横滚相同，内环输出直接进混控；若实机拧航向反了，再只改这里的符号。
 * 偏航外环测量是最短角误差，目标仍为 0，不要把 0～359 的航向直接送进 PID。
 */

#ifndef ATTITUDE_CONTROL_H
#define ATTITUDE_CONTROL_H

#include "imu_attitude.h"
#include "mixer.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 一组会调的 PID 数字，不含积分等运行时状态。
 *
 * 注意：这里**没有** i_limit（积分限幅）。
 * 积分限幅由 attitude_control.c 按 out_max 的固定比例推导（#26 语义拆解），
 * 不再跟输出限幅绑成同一个值，避免"改输出限幅连带改积分限幅"。
 */
typedef struct
{
    /* 比例增益。 */
    float kp;
    /* 积分增益。 */
    float ki;
    /* 微分增益。 */
    float kd;
    /* 输出上限。外环是目标角速度，单位度每秒；内环是混控修正，单位微秒。 */
    float out_max;
    /* 输出下限，通常与上限对称。 */
    float out_min;
} PidParam_t;

/**
 * @brief 飞控姿态环 6 组 PID 参数。
 *
 * 外环输出是目标角速度，单位度每秒；内环输出是混控修正，单位微秒。
 * 当前 out_max 都先填 200；积分限幅由 attitude_control.c 按 out_max 的比例推导。
 */
typedef struct
{
    /* 横滚角度外环。 */
    PidParam_t roll_angle;
    /* 横滚角速度内环。 */
    PidParam_t roll_rate;
    /* 俯仰角度外环。 */
    PidParam_t pitch_angle;
    /* 俯仰角速度内环。 */
    PidParam_t pitch_rate;
    /* 偏航角度外环。 */
    PidParam_t yaw_angle;
    /* 偏航角速度内环。 */
    PidParam_t yaw_rate;
} AttitudePidParams_t;

/**
 * @brief 用默认 6 组参数创建三轴串级 PID，并清零内部状态。
 */
void AttitudeControl_Init(void);

/**
 * @brief 返回编译期默认 6 组参数，给 RAM 表上电拷贝用。
 */
const AttitudePidParams_t *AttitudeControl_GetDefaultParams(void);

/**
 * @brief 把一组参数写进正在跑的三轴串级，并清零积分。
 * @param params 6 组增益和限幅，不能为空。
 */
void AttitudeControl_ApplyParams(const AttitudePidParams_t *params);

/**
 * @brief 清零全部 PID 积分和串级缓存。
 *
 * IMU 重新初始化、电调重新上锁后再进入控制前应调用。
 */
void AttitudeControl_Reset(void);

/**
 * @brief 用当前机体系姿态、磁航向和角速度更新一拍串级自稳，并计算四路电机脉宽。
 * @param throttle_us 基础油门，单位微秒。
 * @param body 已经映射到机体系的姿态角和角速度。
 * @param mag_heading_deg 磁航向，单位度，0～360。仅在 mag_heading_valid 时使用。
 * @param mag_heading_valid 磁力计有效且已完成硬铁校准。
 * @param yaw_stick 遥控偏航杆，-1000～1000，中位附近视为保持航向。
 * @param roll_stick 遥控横滚杆，-1000～1000，左负右正，满杆对应约 ±10°。
 * @param pitch_stick 遥控俯仰杆，-1000～1000，前负后正，满杆对应约 ±10°。
 * @param roll_trim_deg 横滚 Trim，加在目标角上，单位度。
 * @param pitch_trim_deg 俯仰 Trim，加在目标角上，单位度。
 * @param position_roll_offset_deg 定点横滚修正角，单位度；无效时传 0。
 * @param position_pitch_offset_deg 定点俯仰修正角，单位度；无效时传 0。
 * @param dt_s 与上一拍间隔，单位秒，100Hz 时应接近 0.01。
 * @param output 混控输出的四路脉宽。
 * @return 混控计算成功返回 true。
 */
bool AttitudeControl_Update(float throttle_us, const IMU_BodyAttitude_t *body, float mag_heading_deg, bool mag_heading_valid, int16_t yaw_stick, int16_t roll_stick, int16_t pitch_stick, float roll_trim_deg, float pitch_trim_deg, float position_roll_offset_deg, float position_pitch_offset_deg, float dt_s, MixerOutput_t *output);

/**
 * @brief 最近一次 Update 送进混控的三轴修正，单位微秒。给黑匣子记一拍用。
 */
void AttitudeControl_GetLastCorrections(float *roll_us, float *pitch_us, float *yaw_us);

/**
 * @brief 横滚、俯仰的角速度内环是否都已启用（任一增益非 0 即算启用）。
 *
 * 内环全 0 时外环输出会直通当混控修正量，系统有刚度无阻尼，起飞必然发散。
 * 解锁前用它做门禁，避免参数丢失时静默带病飞行。偏航内环不参与判定。
 */
bool AttitudeControl_IsInnerLoopEnabled(void);

#ifdef __cplusplus
}
#endif

#endif
