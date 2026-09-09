/**
 * @file pid.c
 * @brief 通用 PID 控制器实现。
 *
 * 输出 = Kp*误差 + Ki*积分 + Kd*测量变化率（微分对测量求导，见下）。
 * 积分和总输出都有限幅。时间间隔异常时本拍输出 0，避免任务卡住后一次积分炸掉。
 *
 * 微分项（#28）：对测量值求导，再经一阶低通。
 * 目标值（摇杆、Trim、外环输出）突变不再产生微分冲击，陀螺噪声也不再被直接放大。
 *
 * anti-windup（#27）分两层：
 * 1. 条件积分：试探性输出已经饱和、且误差还在往饱和方向推时，本拍不采纳积分；
 * 2. 交付比例收缩：积分限幅乘以 delivery_scale（上一拍混控的统一缩放系数），
 *    保证积分占"实际交付量"的比例恒定，混控缩放时账面值不会虚高。
 */

#include "pid.h"

#include <stddef.h>

/* 微分一阶低通的时间常数（#28），单位秒。
   截止频率 ≈ 1/(2π·tau) = 1/(2π×0.008) ≈ 20Hz，
   高于控制带宽（约几 Hz），低于 IMU DLPF（42Hz），在压噪和相位滞后之间取平衡。
   滤波系数由 dt 实时算出（alpha = tau/(tau+dt)），改控制频率不用改本值。 */
#define PID_DERIV_FILTER_TAU_S 0.008f

/**
 * @brief 写入增益和限幅，并清零积分、微分历史。
 */
void Pid_Init(Pid_t *pid, float kp, float ki, float kd, float i_limit, float out_max, float out_min)
{
    if (pid == NULL)
    {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->i_limit = i_limit;
    pid->out_max = out_max;
    pid->out_min = out_min;
    /* 增益写完后必须复位内部状态，避免沿用未初始化的积分。 */
    Pid_Reset(pid);
}

/**
 * @brief 在线调参时改数字并清积分，行为与 Init 相同，名字分开方便阅读。
 */
void Pid_ApplyParams(Pid_t *pid, float kp, float ki, float kd, float i_limit, float out_max, float out_min)
{
    Pid_Init(pid, kp, ki, kd, i_limit, out_max, out_min);
}

/**
 * @brief 计算一拍 PID 输出。
 */
float Pid_Update(Pid_t *pid, float setpoint, float measured, float dt)
{
    float error;
    float p_term;
    float i_term;
    float d_term;
    float alpha;
    float output;
    float i_limit_effective;
    float integral_tentative;
    float output_tentative;
    bool pushing_high;
    bool pushing_low;

    if (pid == NULL)
    {
        return 0.0f;
    }

    /* 时间间隔不合理时不积分、不微分，避免断点或任务阻塞后出现巨大跳变。
       同时让下一拍微分重新起算，避免用错的间隔求导（#28）。 */
    if ((dt < 0.0005f) || (dt > 0.10f))
    {
        pid->first_run = true;
        return 0.0f;
    }

    /* 误差 = 目标 - 测量。正误差表示还没到目标。 */
    error = setpoint - measured;
    p_term = pid->kp * error;

    /* 微分改为对**测量值**求导（#28）：
       d(error)/dt = d(目标)/dt - d(测量)/dt，丢掉 d(目标)/dt 这一项。
       这样摇杆、Trim、外环输出这些目标值突变时不会再产生微分冲击。 */
    if (pid->first_run)
    {
        /* 第一拍没有上一拍测量，微分强制为 0，避免上电瞬间打满。 */
        pid->deriv_raw = 0.0f;
        pid->deriv_filtered = 0.0f;
        pid->first_run = false;
    }
    else
    {
        pid->deriv_raw = -(measured - pid->last_measured) / dt;

        /* 一阶低通：抑制求导放大的陀螺/姿态噪声。系数与采样率无关。 */
        alpha = PID_DERIV_FILTER_TAU_S / (PID_DERIV_FILTER_TAU_S + dt);
        pid->deriv_filtered = (alpha * pid->deriv_filtered) + ((1.0f - alpha) * pid->deriv_raw);
    }
    d_term = pid->kd * pid->deriv_filtered;

    /* 本拍允许的积分上限：积分限幅 × 交付比例（anti-windup 第 2 层）。
       交付比例默认 1.0，内环由混控上一拍的 correction_scale 刷新。 */
    i_limit_effective = pid->i_limit * pid->delivery_scale;
    if (i_limit_effective < 0.0f) i_limit_effective = 0.0f;
    if (i_limit_effective > pid->i_limit) i_limit_effective = pid->i_limit;

    i_term = 0.0f;
    if (pid->ki != 0.0f)
    {
        integral_tentative = pid->integral + (error * pid->ki * dt);
        output_tentative = p_term + integral_tentative + d_term;

        /* anti-windup 第 1 层（条件积分）：试探性输出已经撞到上下限，
           且误差方向还在把输出往同一侧推，就说明这一拍积分只会加剧饱和，
           不采纳它。误差反向拉回时照常积分，积分能自然退出来。 */
        pushing_high = (output_tentative > pid->out_max) && (error > 0.0f);
        pushing_low = (output_tentative < pid->out_min) && (error < 0.0f);
        if (!pushing_high && !pushing_low)
        {
            pid->integral = integral_tentative;
        }

        /* 积分限幅，防止长时间存在小误差后输出饱和。 */
        if (pid->integral > i_limit_effective)
        {
            pid->integral = i_limit_effective;
        }
        if (pid->integral < -i_limit_effective)
        {
            pid->integral = -i_limit_effective;
        }
        i_term = pid->integral;
    }

    output = p_term + i_term + d_term;
    if (output > pid->out_max)
    {
        output = pid->out_max;
    }
    if (output < pid->out_min)
    {
        output = pid->out_min;
    }

    pid->last_measured = measured;
    return output;
}

/**
 * @brief 清零积分和微分历史，不改增益和限幅。
 */
void Pid_Reset(Pid_t *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->integral = 0.0f;
    pid->last_measured = 0.0f;
    pid->deriv_raw = 0.0f;
    pid->deriv_filtered = 0.0f;
    pid->first_run = true;
    /* 交付比例一并复位，避免沿用上锁前的旧缩放系数。 */
    pid->delivery_scale = 1.0f;
}

/**
 * @brief 设置本拍的交付比例，供 anti-windup 收缩积分限幅。
 */
void Pid_SetDeliveryScale(Pid_t *pid, float scale)
{
    if (pid == NULL)
    {
        return;
    }
    if (scale < 0.0f) scale = 0.0f;
    if (scale > 1.0f) scale = 1.0f;
    pid->delivery_scale = scale;
}
