/**
 * @file pid.h
 * @brief 通用 PID 控制器。
 *
 * 本模块只做误差计算，不读取 IMU，也不调用混控。
 * 控制任务先调用 Pid_Update() 得到修正量，再把结果交给 Mixer_Calculate()。
 *
 * 第一版建议：
 * 比例项用角度误差，微分项可先用误差差分；
 * 积分项先把 ki 设为 0，等自稳符号确认后再打开。
 *
 * 微分项（#28）：对测量值求导 + 一阶低通，避免目标值突变产生微分冲击，
 * 同时抑制求导放大的陀螺/姿态噪声。详见 pid.c。
 */

#ifndef PID_H
#define PID_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 一组独立的 PID 状态。
 *
 * 横滚、俯仰、偏航应各持有一份，不要共用积分和上一拍误差。
 */
typedef struct
{
    /* 比例增益。输出与当前误差成正比。 */
    float kp;
    /* 积分增益。第一版可设为 0，避免静差积分把油门顶满。 */
    float ki;
    /* 微分增益。对**测量值**求导（#28），抑制机体快速变化，不受目标值突变影响。 */
    float kd;
    /* 积分项限幅，防止积分顶满后长时间刹不住。 */
    float i_limit;
    /* 输出上限，飞控里通常对应混控修正的最大微秒。 */
    float out_max;
    /* 输出下限，通常与上限对称。 */
    float out_min;
    /* 当前积分累加值。 */
    float integral;
    /* 上一拍测量值，供"对测量微分"使用（#28）。 */
    float last_measured;
    /* 本拍滤波前的原始微分值，便于观察滤波效果，不参与计算。 */
    float deriv_raw;
    /* 本拍滤波后的微分值，实际参与 D 项计算。 */
    float deriv_filtered;
    /* true 表示还没有完整的上一拍测量，第一拍微分强制为 0。 */
    bool first_run;
    /* 上一拍的实际交付比例，0～1，默认 1.0。
       混控会把三轴修正统一缩放，内环输出（混控修正）因此可能只交付一部分；
       积分限幅会乘以本比例，保证积分占"实际交付量"的比例恒定，不会账面虚高。
       由调用者每拍通过 Pid_SetDeliveryScale() 刷新。 */
    float delivery_scale;
} Pid_t;

/**
 * @brief 写入增益和限幅，并清零内部状态。
 * @param pid 要初始化的 PID 对象。
 * @param kp 比例增益。
 * @param ki 积分增益，可先填 0。
 * @param kd 微分增益。
 * @param i_limit 积分限幅的绝对值上限。
 * @param out_max 输出上限。
 * @param out_min 输出下限。
 */
void Pid_Init(Pid_t *pid, float kp, float ki, float kd, float i_limit, float out_max, float out_min);

/**
 * @brief 只改增益和限幅，并清零积分。给在线调参用，不要每拍调用。
 */
void Pid_ApplyParams(Pid_t *pid, float kp, float ki, float kd, float i_limit, float out_max, float out_min);

/**
 * @brief 按目标值和测量值更新一次 PID。
 * @param pid PID 对象。
 * @param setpoint 目标值，例如目标横滚角，单位与测量值相同。
 * @param measured 测量值，例如当前横滚角。
 * @param dt 与上一拍的时间间隔，单位秒。100Hz 时应接近 0.01。
 * @return 已经限幅后的输出。飞控里通常当作混控修正量，单位微秒。
 */
float Pid_Update(Pid_t *pid, float setpoint, float measured, float dt);

/**
 * @brief 清零积分和微分历史。
 *
 * 上锁、断控或重新校准时调用，避免旧积分在下次解锁时突然顶出去。
 * 同时把交付比例复位成 1.0，避免沿用上锁前的旧值。
 */
void Pid_Reset(Pid_t *pid);

/**
 * @brief 设置本拍的交付比例，供 anti-windup 收缩积分限幅。
 *
 * 飞控里只有内环（输出是混控修正）需要这个值，取上一拍 Mixer_Calculate()
 * 返回的 correction_scale。其余场合保持默认 1.0 即可。
 * @param pid PID 对象。
 * @param scale 交付比例，会被限制到 0～1。
 */
void Pid_SetDeliveryScale(Pid_t *pid, float scale);

#ifdef __cplusplus
}
#endif

#endif
