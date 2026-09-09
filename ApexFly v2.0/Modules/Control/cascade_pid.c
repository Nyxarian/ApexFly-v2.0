/**
 * @file cascade_pid.c
 * @brief 串级 PID 连接实现。
 *
 * 外环 Pid_Update() 的输出写入 inner_setpoint，
 * 内环再用这个目标去跟踪角速度。
 * 本文件不读写 IMU，也不调用混控。
 */

#include "cascade_pid.h"

#include <stddef.h>

/**
 * @brief 分别初始化外环、内环 PID，并清零串级缓存。
 */
void CascadePid_Init(CascadePid_t *controller, float outer_kp, float outer_ki, float outer_kd, float outer_i_limit, float outer_out_max, float outer_out_min, float inner_kp, float inner_ki, float inner_kd, float inner_i_limit, float inner_out_max, float inner_out_min)
{
    if (controller == NULL)
    {
        return;
    }

    /* 外环：角度误差 -> 目标角速度。 */
    Pid_Init(&controller->outer, outer_kp, outer_ki, outer_kd, outer_i_limit, outer_out_max, outer_out_min);
    /* 内环：角速度误差 -> 混控修正。 */
    Pid_Init(&controller->inner, inner_kp, inner_ki, inner_kd, inner_i_limit, inner_out_max, inner_out_min);
    controller->inner_setpoint = 0.0f;
    controller->output = 0.0f;
}

/**
 * @brief 在线调参时改两环数字并清积分。
 */
void CascadePid_ApplyParams(CascadePid_t *controller, float outer_kp, float outer_ki, float outer_kd, float outer_i_limit, float outer_out_max, float outer_out_min, float inner_kp, float inner_ki, float inner_kd, float inner_i_limit, float inner_out_max, float inner_out_min)
{
    CascadePid_Init(controller, outer_kp, outer_ki, outer_kd, outer_i_limit, outer_out_max, outer_out_min, inner_kp, inner_ki, inner_kd, inner_i_limit, inner_out_max, inner_out_min);
}

/**
 * @brief 清零外环、内环状态，以及内环目标和最终输出。
 */
void CascadePid_Reset(CascadePid_t *controller)
{
    if (controller == NULL)
    {
        return;
    }

    Pid_Reset(&controller->outer);
    Pid_Reset(&controller->inner);
    controller->inner_setpoint = 0.0f;
    controller->output = 0.0f;
}

/**
 * @brief 把交付比例转发给内环 PID，供 anti-windup 收缩内环积分限幅。
 */
void CascadePid_SetInnerDeliveryScale(CascadePid_t *controller, float scale)
{
    if (controller == NULL)
    {
        return;
    }

    Pid_SetDeliveryScale(&controller->inner, scale);
}

/**
 * @brief 计算外环，并把结果存成内环下一拍要用的目标。
 */
float CascadePid_UpdateOuter(CascadePid_t *controller, float outer_setpoint, float outer_measurement, float dt_s)
{
    if (controller == NULL)
    {
        return 0.0f;
    }

    controller->inner_setpoint = Pid_Update(&controller->outer, outer_setpoint, outer_measurement, dt_s);
    return controller->inner_setpoint;
}

/**
 * @brief 用最近一次外环目标计算内环，得到这轴最终输出。
 */
float CascadePid_UpdateInner(CascadePid_t *controller, float inner_measurement, float dt_s)
{
    if (controller == NULL)
    {
        return 0.0f;
    }

    controller->output = Pid_Update(&controller->inner, controller->inner_setpoint, inner_measurement, dt_s);
    return controller->output;
}
