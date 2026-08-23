/**
 * @file attitude_control.c
 * @brief 姿态自稳实现：三轴串级 PID，再混控。
 *
 * 横滚、俯仰、偏航各持有一套 CascadePid_t，状态互不影响。
 * 6 组编译默认放在 kDefaultPidParams。遥控在线调参改 RAM 表后再 ApplyParams。
 *
 * 每轴闭环：
 * 外环：目标角 0 vs 当前角 -> 目标角速度
 * 内环：目标角速度 vs 机体系陀螺 -> 混控修正微秒
 * 内环 Kp/Ki/Kd 全为 0 时，外环输出直接当作混控修正，方便先调角度环。
 *
 * 实机已确认的混控入口符号：
 * roll_correction  = -Kp * roll_deg
 * pitch_correction = +Kp * pitch_deg
 *
 * 外环目标为 0 时输出是 -测量，所以：
 * 横滚、偏航：串级输出直接进混控；
 * 俯仰：串级输出再取反后进混控。
 */

#include "attitude_control.h"

#include "cascade_pid.h"
#include "motor.h"
#include <stddef.h>

/*
 * 调参改这里。6 组对象独立，不要改回共用一个 Pid_t。
 * 当前全部 Kp/Ki/Kd 为 0，积分限幅和输出限幅为 200。
 */
static const AttitudePidParams_t kDefaultPidParams = {
    .roll_angle = {.kp = 0.0f, .ki = 0.0f, .kd = 0.0f, .i_limit = 200.0f, .out_max = 200.0f, .out_min = -200.0f},
    .roll_rate = {.kp = 0.0f, .ki = 0.0f, .kd = 0.0f, .i_limit = 200.0f, .out_max = 200.0f, .out_min = -200.0f},
    .pitch_angle = {.kp = 0.0f, .ki = 0.0f, .kd = 0.0f, .i_limit = 200.0f, .out_max = 200.0f, .out_min = -200.0f},
    .pitch_rate = {.kp = 0.0f, .ki = 0.0f, .kd = 0.0f, .i_limit = 200.0f, .out_max = 200.0f, .out_min = -200.0f},
    .yaw_angle = {.kp = 0.0f, .ki = 0.0f, .kd = 0.0f, .i_limit = 200.0f, .out_max = 200.0f, .out_min = -200.0f},
    .yaw_rate = {.kp = 0.0f, .ki = 0.0f, .kd = 0.0f, .i_limit = 200.0f, .out_max = 200.0f, .out_min = -200.0f},
};

static CascadePid_t roll_cascade;
static CascadePid_t pitch_cascade;
static CascadePid_t yaw_cascade;
static float last_roll_correction_us;
static float last_pitch_correction_us;
static float last_yaw_correction_us;

/**
 * @brief 用外环、内环各一组参数初始化一轴串级。
 */
static void AttitudeControl_InitAxis(CascadePid_t *axis, const PidParam_t *outer, const PidParam_t *inner)
{
    CascadePid_Init(axis, outer->kp, outer->ki, outer->kd, outer->i_limit, outer->out_max, outer->out_min, inner->kp, inner->ki, inner->kd, inner->i_limit, inner->out_max, inner->out_min);
}

/**
 * @brief 内环增益全为 0 时视为未启用，外环输出直通到混控。
 */
static bool AttitudeControl_InnerEnabled(const CascadePid_t *axis)
{
    return (axis->inner.kp != 0.0f) || (axis->inner.ki != 0.0f) || (axis->inner.kd != 0.0f);
}

/**
 * @brief 更新一轴串级，并用 mixer_sign 对齐已测混控符号。横滚、偏航为 +1，俯仰为 -1。
 */
static float AttitudeControl_UpdateAxis(CascadePid_t *axis, float angle_measured_deg, float rate_measured_dps, float dt_s, float mixer_sign)
{
    float outer_out;
    float inner_out;

    outer_out = CascadePid_UpdateOuter(axis, 0.0f, angle_measured_deg, dt_s);
    if (AttitudeControl_InnerEnabled(axis))
    {
        inner_out = CascadePid_UpdateInner(axis, rate_measured_dps, dt_s);
    }
    else
    {
        /* 内环未开：保持和之前开环角度测试相同的通路。 */
        inner_out = outer_out;
    }

    return mixer_sign * inner_out;
}

/**
 * @brief 把默认 6 组参数灌进三套串级 PID。
 */
void AttitudeControl_Init(void)
{
    AttitudeControl_ApplyParams(&kDefaultPidParams);
}

const AttitudePidParams_t *AttitudeControl_GetDefaultParams(void)
{
    return &kDefaultPidParams;
}

void AttitudeControl_ApplyParams(const AttitudePidParams_t *params)
{
    if (params == NULL) return;
    AttitudeControl_InitAxis(&roll_cascade, &params->roll_angle, &params->roll_rate);
    AttitudeControl_InitAxis(&pitch_cascade, &params->pitch_angle, &params->pitch_rate);
    AttitudeControl_InitAxis(&yaw_cascade, &params->yaw_angle, &params->yaw_rate);
}

/**
 * @brief 三轴串级一起复位，避免旧积分在下次解锁时顶出去。
 */
void AttitudeControl_Reset(void)
{
    CascadePid_Reset(&roll_cascade);
    CascadePid_Reset(&pitch_cascade);
    CascadePid_Reset(&yaw_cascade);
}

/**
 * @brief 三轴外环加内环，按已测符号送进混控。
 */
bool AttitudeControl_Update(float throttle_us, const IMU_BodyAttitude_t *body, float dt_s, MixerOutput_t *output)
{
    MixerInput_t mixer_input;

    if ((body == NULL) || (output == NULL))
    {
        return false;
    }

    mixer_input.throttle_us = throttle_us;
    mixer_input.roll_correction_us = AttitudeControl_UpdateAxis(&roll_cascade, body->roll_deg, body->roll_rate_dps, dt_s, 1.0f);
    mixer_input.pitch_correction_us = AttitudeControl_UpdateAxis(&pitch_cascade, body->pitch_deg, body->pitch_rate_dps, dt_s, -1.0f);
    mixer_input.yaw_correction_us = AttitudeControl_UpdateAxis(&yaw_cascade, body->yaw_deg, body->yaw_rate_dps, dt_s, 1.0f);
    last_roll_correction_us = mixer_input.roll_correction_us;
    last_pitch_correction_us = mixer_input.pitch_correction_us;
    last_yaw_correction_us = mixer_input.yaw_correction_us;

    return Mixer_Calculate(&mixer_input, MOTOR_PWM_MIN_US, MOTOR_PWM_MAX_US, output);
}

void AttitudeControl_GetLastCorrections(float *roll_us, float *pitch_us, float *yaw_us)
{
    if (roll_us != NULL) *roll_us = last_roll_correction_us;
    if (pitch_us != NULL) *pitch_us = last_pitch_correction_us;
    if (yaw_us != NULL) *yaw_us = last_yaw_correction_us;
}
