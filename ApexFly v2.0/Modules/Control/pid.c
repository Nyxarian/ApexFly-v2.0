/**
 * @file pid.c
 * @brief 通用 PID 控制器实现。
 *
 * 输出 = Kp*误差 + Ki*积分 + Kd*误差变化率。
 * 积分和总输出都有限幅。时间间隔异常时本拍输出 0，避免任务卡住后一次积分炸掉。
 */

#include "pid.h"

#include <stddef.h>

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
    float deriv;
    float output;

    if (pid == NULL)
    {
        return 0.0f;
    }

    /* 时间间隔不合理时不积分、不微分，避免断点或任务阻塞后出现巨大跳变。 */
    if ((dt < 0.0005f) || (dt > 0.10f))
    {
        return 0.0f;
    }

    /* 误差 = 目标 - 测量。正误差表示还没到目标。 */
    error = setpoint - measured;
    p_term = pid->kp * error;

    i_term = 0.0f;
    if (pid->ki != 0.0f)
    {
        pid->integral += error * pid->ki * dt;
        /* 积分限幅，防止长时间存在小误差后输出饱和。 */
        if (pid->integral > pid->i_limit)
        {
            pid->integral = pid->i_limit;
        }
        if (pid->integral < -pid->i_limit)
        {
            pid->integral = -pid->i_limit;
        }
        i_term = pid->integral;
    }

    if (pid->first_run)
    {
        /* 第一拍没有上一拍误差，微分强制为 0，避免上电瞬间打满。 */
        deriv = 0.0f;
        pid->first_run = false;
    }
    else
    {
        deriv = (error - pid->last_error) / dt;
    }
    d_term = pid->kd * deriv;

    output = p_term + i_term + d_term;
    if (output > pid->out_max)
    {
        output = pid->out_max;
    }
    if (output < pid->out_min)
    {
        output = pid->out_min;
    }

    pid->last_error = error;
    pid->last_deriv = deriv;
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
    pid->last_error = 0.0f;
    pid->last_deriv = 0.0f;
    pid->first_run = true;
}
