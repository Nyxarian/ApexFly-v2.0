/**
 * @file altitude_control.c
 * @brief 旧垂速环实现。当前电机任务已改为直通油门，本文件不再写 PWM。
 *
 * hover_pwm_us 由 PID 表复用为满杆油门上限，默认 1500us。
 */

#include "altitude_control.h"

#include "motor.h"
#include "pid.h"
#include <stddef.h>

#define ALT_STICK_DEAD_LOW 400U
#define ALT_STICK_DEAD_HIGH 600U
#define ALT_CLIMB_MPS 0.30f
#define ALT_DESCEND_MPS 0.20f
#define ALT_HOLD_KP 0.50f
#define ALT_HOLD_VZ_MAX 0.20f
#define ALT_VZ_I_LIMIT_US 80.0f
#define ALT_PID_OUT_LIMIT_US 180.0f
#define ALT_OUT_MIN_US 1050.0f
#define ALT_OUT_MAX_US 1750.0f
#define ALT_TAKEOFF_SLEW_UP_US_PER_S 100.0f
#define ALT_TAKEOFF_SLEW_DOWN_US_PER_S 80.0f
#define ALT_AIR_SLEW_UP_US_PER_S 350.0f
#define ALT_AIR_SLEW_DOWN_US_PER_S 220.0f
#define ALT_AIRBORNE_HEIGHT_M 0.15f
#define ALT_AIRBORNE_VZ_MPS 0.12f
#define ALT_TAKEOFF_REQUEST_CONFIRM_S 0.15f

static const AltitudeControlParams_t altitude_default_params = {100.0f, 0.0f, 0.0f, 1500.0f};
static AltitudeControlParams_t altitude_params = {100.0f, 0.0f, 0.0f, 1500.0f};
static Pid_t vz_pid;
static float z_hold_m;
static float output_us;
static float takeoff_request_s;
static float airborne_confirm_s;
static AltitudeControlState_t state_before_estimate_hold;
static bool holding;
static bool inited;
static AltitudeControlDebug_t altitude_debug;

static float AltitudeControl_Clampf(float value, float minimum, float maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static float AltitudeControl_Slew(float current, float target, float up_rate, float down_rate, float dt_s)
{
    float step;

    if (target > current)
    {
        step = up_rate * dt_s;
        return (target - current > step) ? (current + step) : target;
    }
    step = down_rate * dt_s;
    return (current - target > step) ? (current - step) : target;
}

static void AltitudeControl_InitPid(void)
{
    Pid_Init(&vz_pid, altitude_params.vz_kp, altitude_params.vz_ki, altitude_params.vz_kd, ALT_VZ_I_LIMIT_US, ALT_PID_OUT_LIMIT_US, -ALT_PID_OUT_LIMIT_US);
}

const AltitudeControlParams_t *AltitudeControl_GetDefaultParams(void)
{
    return &altitude_default_params;
}

void AltitudeControl_ApplyParams(const AltitudeControlParams_t *params)
{
    if (params == NULL) return;
    altitude_params = *params;
    AltitudeControl_InitPid();
}

void AltitudeControl_Reset(void)
{
    AltitudeControl_InitPid();
    z_hold_m = 0.0f;
    output_us = MOTOR_PWM_MIN_US;
    takeoff_request_s = 0.0f;
    airborne_confirm_s = 0.0f;
    state_before_estimate_hold = ALTITUDE_CONTROL_GROUND_IDLE;
    holding = false;
    altitude_debug.target_vz_mps = 0.0f;
    altitude_debug.correction_us = 0.0f;
    altitude_debug.hover_pwm_us = altitude_params.hover_pwm_us;
    altitude_debug.state = ALTITUDE_CONTROL_GROUND_IDLE;
    inited = true;
}

void AltitudeControl_GetDebug(AltitudeControlDebug_t *debug)
{
    if (debug != NULL) *debug = altitude_debug;
}

uint16_t AltitudeControl_Update(uint16_t stick, float z_m, float vz_mps, bool estimate_valid, float dt_s, bool armed)
{
    uint16_t stick_used;
    float vz_sp;
    float hold_vz;
    float target_us;
    float up_rate;
    float down_rate;

    if (!inited) AltitudeControl_Reset();
    altitude_debug.hover_pwm_us = altitude_params.hover_pwm_us;
    if (!armed)
    {
        AltitudeControl_Reset();
        return MOTOR_PWM_MIN_US;
    }

    stick_used = (stick > 1000U) ? 1000U : stick;

    /* 地面未启动时绝不送悬停油门；连续 150ms 健康且杆在上升区才接受起飞请求。 */
    if (altitude_debug.state == ALTITUDE_CONTROL_GROUND_IDLE)
    {
        altitude_debug.target_vz_mps = 0.0f;
        altitude_debug.correction_us = 0.0f;
        if (!estimate_valid || (stick_used <= ALT_STICK_DEAD_HIGH))
        {
            takeoff_request_s = 0.0f;
            return MOTOR_PWM_MIN_US;
        }
        takeoff_request_s += dt_s;
        if (takeoff_request_s < ALT_TAKEOFF_REQUEST_CONFIRM_S) return MOTOR_PWM_MIN_US;
        takeoff_request_s = 0.0f;
        altitude_debug.state = ALTITUDE_CONTROL_TAKEOFF_RAMP;
        Pid_Reset(&vz_pid);
    }

    /* 飞行中高度估计丢失时冻结最近安全油门，不突变为直通映射。 */
    if (!estimate_valid)
    {
        holding = false;
        Pid_Reset(&vz_pid);
        altitude_debug.target_vz_mps = 0.0f;
        altitude_debug.correction_us = 0.0f;
        if (altitude_debug.state != ALTITUDE_CONTROL_ESTIMATE_HOLD) state_before_estimate_hold = altitude_debug.state;
        altitude_debug.state = ALTITUDE_CONTROL_ESTIMATE_HOLD;
        return (uint16_t)(output_us + 0.5f);
    }
    if (altitude_debug.state == ALTITUDE_CONTROL_ESTIMATE_HOLD)
    {
        /* 恢复后回到失效前状态，不用单个高度样本猜测是否离地。 */
        altitude_debug.state = state_before_estimate_hold;
        airborne_confirm_s = 0.0f;
        Pid_Reset(&vz_pid);
    }

    if (stick_used < ALT_STICK_DEAD_LOW)
    {
        holding = false;
        vz_sp = -ALT_DESCEND_MPS * (float)(ALT_STICK_DEAD_LOW - stick_used) / (float)ALT_STICK_DEAD_LOW;
    }
    else if (stick_used <= ALT_STICK_DEAD_HIGH)
    {
        if (!holding)
        {
            z_hold_m = z_m;
            holding = true;
        }
        hold_vz = ALT_HOLD_KP * (z_hold_m - z_m);
        vz_sp = AltitudeControl_Clampf(hold_vz, -ALT_HOLD_VZ_MAX, ALT_HOLD_VZ_MAX);
    }
    else
    {
        holding = false;
        vz_sp = ALT_CLIMB_MPS * (float)(stick_used - ALT_STICK_DEAD_HIGH) / (float)(1000U - ALT_STICK_DEAD_HIGH);
    }

    altitude_debug.target_vz_mps = vz_sp;
    /* 斜坡阶段禁用 I/D 历史，避免地面无响应时积分或杆区切换微分冲击。 */
    if (altitude_debug.state == ALTITUDE_CONTROL_TAKEOFF_RAMP) Pid_Reset(&vz_pid);
    altitude_debug.correction_us = Pid_Update(&vz_pid, vz_sp, vz_mps, dt_s);

    /* 尚未确认离地时杆退出上升区，就撤回本次起飞请求并缓慢回到 1000us。 */
    if ((altitude_debug.state == ALTITUDE_CONTROL_TAKEOFF_RAMP) && (stick_used <= ALT_STICK_DEAD_HIGH))
    {
        Pid_Reset(&vz_pid);
        altitude_debug.correction_us = 0.0f;
        output_us = AltitudeControl_Slew(output_us, MOTOR_PWM_MIN_US, ALT_TAKEOFF_SLEW_UP_US_PER_S, ALT_TAKEOFF_SLEW_DOWN_US_PER_S, dt_s);
        if (output_us <= (float)MOTOR_PWM_MIN_US + 0.5f) altitude_debug.state = ALTITUDE_CONTROL_GROUND_IDLE;
        return (uint16_t)(output_us + 0.5f);
    }

    target_us = AltitudeControl_Clampf(altitude_params.hover_pwm_us + altitude_debug.correction_us, ALT_OUT_MIN_US, ALT_OUT_MAX_US);

    if (altitude_debug.state == ALTITUDE_CONTROL_TAKEOFF_RAMP)
    {
        /* 连续 0.3 秒同时满足高度和上升速度才确认离地，过滤地面气压单点噪声。 */
        if (((z_m > ALT_AIRBORNE_HEIGHT_M) && (vz_mps > ALT_AIRBORNE_VZ_MPS)) || (z_m > 0.30f)) airborne_confirm_s += dt_s;
        else airborne_confirm_s = 0.0f;
        if (airborne_confirm_s >= 0.30f)
        {
            altitude_debug.state = ALTITUDE_CONTROL_AIRBORNE;
            Pid_Reset(&vz_pid);
        }
    }
    if (altitude_debug.state == ALTITUDE_CONTROL_AIRBORNE)
    {
        up_rate = ALT_AIR_SLEW_UP_US_PER_S;
        down_rate = ALT_AIR_SLEW_DOWN_US_PER_S;
    }
    else
    {
        up_rate = ALT_TAKEOFF_SLEW_UP_US_PER_S;
        down_rate = ALT_TAKEOFF_SLEW_DOWN_US_PER_S;
    }
    output_us = AltitudeControl_Slew(output_us, target_us, up_rate, down_rate, dt_s);
    output_us = AltitudeControl_Clampf(output_us, MOTOR_PWM_MIN_US, ALT_OUT_MAX_US);
    return (uint16_t)(output_us + 0.5f);
}
