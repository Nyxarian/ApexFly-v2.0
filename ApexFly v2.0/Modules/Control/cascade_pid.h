/**
 * @file cascade_pid.h
 * @brief 串级 PID：外环输出作为内环目标。
 *
 * 本身没有新的 PID 公式，只是把两个 Pid_t 接在一起。
 * 飞控里一轴两环：外环管角度，内环管角速度。
 * 横滚、俯仰、偏航各持有一份，一共 6 组独立 PID，状态不能共用。
 */

#ifndef CASCADE_PID_H
#define CASCADE_PID_H

#include "pid.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 一轴串级控制器：外环 + 内环。
 */
typedef struct
{
    /* 外环。姿态控制中把角度误差换成目标角速度，单位通常是度每秒。 */
    Pid_t outer;
    /* 内环。把角速度误差换成混控修正，单位通常是微秒。 */
    Pid_t inner;
    /* 最近一次外环输出，也是内环的目标值。 */
    float inner_setpoint;
    /* 最近一次内环输出，也是这轴送给混控的修正量。 */
    float output;
} CascadePid_t;

/**
 * @brief 写入外环、内环增益和限幅，并清零运行状态。
 * @param controller 这一轴的串级对象。
 * @param outer_kp 外环比例增益。
 * @param outer_ki 外环积分增益。
 * @param outer_kd 外环微分增益。
 * @param outer_i_limit 外环积分限幅。
 * @param outer_out_max 外环输出上限，通常是最大目标角速度。
 * @param outer_out_min 外环输出下限。
 * @param inner_kp 内环比例增益。
 * @param inner_ki 内环积分增益。
 * @param inner_kd 内环微分增益。
 * @param inner_i_limit 内环积分限幅。
 * @param inner_out_max 内环输出上限，通常是最大混控修正微秒。
 * @param inner_out_min 内环输出下限。
 */
void CascadePid_Init(CascadePid_t *controller, float outer_kp, float outer_ki, float outer_kd, float outer_i_limit, float outer_out_max, float outer_out_min, float inner_kp, float inner_ki, float inner_kd, float inner_i_limit, float inner_out_max, float inner_out_min);

/**
 * @brief 在线调参：改外环、内环增益和限幅，并清零积分。
 */
void CascadePid_ApplyParams(CascadePid_t *controller, float outer_kp, float outer_ki, float outer_kd, float outer_i_limit, float outer_out_max, float outer_out_min, float inner_kp, float inner_ki, float inner_kd, float inner_i_limit, float inner_out_max, float inner_out_min);

/**
 * @brief 清零外环、内环积分和串级缓存。
 */
void CascadePid_Reset(CascadePid_t *controller);

/**
 * @brief 更新外环，并把输出存成内环目标。
 * @param outer_setpoint 外环目标，例如目标横滚角，单位度。
 * @param outer_measurement 外环测量，例如当前横滚角，单位度。
 * @param dt_s 外环周期，单位秒。
 * @return 内环目标，例如目标角速度，单位度每秒。
 */
float CascadePid_UpdateOuter(CascadePid_t *controller, float outer_setpoint, float outer_measurement, float dt_s);

/**
 * @brief 用最近一次外环目标更新内环。
 * @param inner_measurement 内环测量，例如机体系陀螺角速度，单位度每秒。
 * @param dt_s 内环周期，单位秒。
 * @return 这轴最终控制量，飞控里通常是混控修正，单位微秒。
 */
float CascadePid_UpdateInner(CascadePid_t *controller, float inner_measurement, float dt_s);

#ifdef __cplusplus
}
#endif

#endif
