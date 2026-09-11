/**
 * @file attitude_control.c
 * @brief 姿态自稳实现：三轴串级 PID，再混控。
 *
 * 横滚、俯仰、偏航各持有一套 CascadePid_t，状态互不影响。
 * 6 组编译默认放在 kDefaultPidParams。遥控在线调参改 RAM 表后再 ApplyParams。
 *
 * 每轴闭环：
 * 外环：目标角 vs 当前角 -> 目标角速度
 * 内环：目标角速度 vs 机体系陀螺 -> 混控修正微秒
 * 横滚俯仰目标角由遥控杆映射，满杆约 ±10°，再加 Trim；中位回平。
 * 偏航外环吃磁航向最短角误差；杆出死区时外环让路，内环跟杆。
 * 内环 Kp/Ki/Kd 全为 0 时，外环输出直接当作混控修正，方便先调角度环。
 *
 * anti-windup（#27）：内环积分限幅按上一拍混控的 correction_scale 收缩，输出饱和时
 * 停止积分；外环输出是目标角速度，不经过混控缩放，交付比例恒为 1.0。
 *
 * 目标信号斜率限制（#29）：横滚/俯仰目标角、偏航目标角速度都先过 AttitudeControl_SlewLimit()，
 * 杆量、Trim、链路跳变不会让目标值一步到位，打杆冲击有界可预测。
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

#define ATTITUDE_YAW_STICK_DEADZONE 80
#define ATTITUDE_STICK_DEADZONE 80
#define ATTITUDE_YAW_TILT_FREEZE_DEG 25.0f
#define ATTITUDE_STICK_ABS_MAX 1000.0f
#define ATTITUDE_STICK_MAX_ANGLE_DEG 10.0f

/* 俯仰摇杆方向开关，只作用于"摇杆 -> 目标俯仰角"这一支：
   +1 = 推杆向前使机头下俯（前飞），-1 = 反相。
   不影响姿态反馈方向、mixer_sign、混控符号（那些是实机验证过的）。
   出现"推杆向前却向后飞"时，优先在遥控端把俯仰通道反向；
   若遥控端改不了，把这里改成 -1 即可，只动这一行。 */
#define ATTITUDE_PITCH_STICK_SIGN (-1.0f)

/* 目标信号斜率限制（#29）。
   杆量、Trim、链路跳变都会让目标值一步到位，外环误差跟着跳，
   内环 P 项就会瞬间变大（快速打杆时可达限幅的一半以上）。
   下面三个上限把"目标值每拍能变多少"封住，让打杆冲击有界、可预测。 */
/* 横滚/俯仰目标角最大变化率：满杆 10°，200°/s → 全行程最快 50ms 走完。 */
#define ATTITUDE_TARGET_SLEW_DPS 200.0f
/* 偏航满杆对应的目标角速度。与偏航外环输出限幅解耦：外环限幅是"航向保持环
   能要求多快"，这里是"杆能要求多快"，语义不同，不共用一个参数。 */
#define ATTITUDE_YAW_STICK_MAX_RATE_DPS 200.0f
/* 偏航目标角速度最大变化率：0→满速 250ms 走完，避免打杆瞬间把偏航内环打满。 */
#define ATTITUDE_YAW_RATE_SLEW_DPS 800.0f

/* 积分限幅 = 输出限幅 × 本比例（#26 限幅语义拆解）。
   外环输出是目标角速度（度每秒），内环输出是混控修正（微秒）。
   取 15% / 25%：积分项最多只占输出的一小部分，Ki 打开后也不会独占整个输出造成 windup。
   遥控只能改输出限幅（MAX_RATE_DPS / MAX_CORRECTION_US），积分限幅按本比例自动跟随，
   不再出现"改一个动两个、且两者永远相等"的耦合。 */
#define ATTITUDE_OUTER_I_LIMIT_RATIO 0.15f
#define ATTITUDE_INNER_I_LIMIT_RATIO 0.25f

/*
 * 调参改这里。6 组对象独立，不要改回共用一个 Pid_t。
 * 当前全部 Kp/Ki/Kd 为 0，输出限幅为 200；积分限幅不在这里，按上面的比例推导。
 */
static const AttitudePidParams_t kDefaultPidParams = {
    .roll_angle = {.kp = 0.0f, .ki = 0.0f, .kd = 0.0f, .out_max = 200.0f, .out_min = -200.0f},
    .roll_rate = {.kp = 0.0f, .ki = 0.0f, .kd = 0.0f, .out_max = 200.0f, .out_min = -200.0f},
    .pitch_angle = {.kp = 0.0f, .ki = 0.0f, .kd = 0.0f, .out_max = 200.0f, .out_min = -200.0f},
    .pitch_rate = {.kp = 0.0f, .ki = 0.0f, .kd = 0.0f, .out_max = 200.0f, .out_min = -200.0f},
    .yaw_angle = {.kp = 0.0f, .ki = 0.0f, .kd = 0.0f, .out_max = 200.0f, .out_min = -200.0f},
    .yaw_rate = {.kp = 0.0f, .ki = 0.0f, .kd = 0.0f, .out_max = 200.0f, .out_min = -200.0f},
};

static CascadePid_t roll_cascade;
static CascadePid_t pitch_cascade;
static CascadePid_t yaw_cascade;
static float last_roll_correction_us;
static float last_pitch_correction_us;
static float last_yaw_correction_us;
static float yaw_hold_heading_deg;
static bool yaw_hold_valid;
static bool yaw_stick_was_deflected;
/* 上一拍混控的统一缩放系数，作为三个内环积分限幅的收缩系数（#27 anti-windup）。
   混控把三轴修正统一缩放后，内环实际交付量小于请求量，积分限幅要跟着收缩。 */
static float last_correction_scale = 1.0f;
/* 斜率限制后的目标值（#29），每拍朝原始目标移动，单次最多动限幅量。 */
static float roll_setpoint_limited_deg;
static float pitch_setpoint_limited_deg;
static float yaw_rate_setpoint_limited_dps;

/**
 * @brief 用外环、内环各一组参数初始化一轴串级。
 *
 * 积分限幅不取参数表里的值，而是按输出限幅的固定比例推导（#26）：
 * 这样遥控改 MAX_RATE_DPS / MAX_CORRECTION_US 时，积分限幅按比例跟随，
 * 既不会跟输出限幅相等（windup），也不会出现积分限幅大于输出限幅的非法组合。
 */
static void AttitudeControl_InitAxis(CascadePid_t *axis, const PidParam_t *outer, const PidParam_t *inner)
{
    CascadePid_Init(axis,
                    outer->kp, outer->ki, outer->kd,
                    outer->out_max * ATTITUDE_OUTER_I_LIMIT_RATIO, outer->out_max, outer->out_min,
                    inner->kp, inner->ki, inner->kd,
                    inner->out_max * ATTITUDE_INNER_I_LIMIT_RATIO, inner->out_max, inner->out_min);
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
static float AttitudeControl_Absf(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float AttitudeControl_Wrap180(float heading_deg)
{
    while (heading_deg > 180.0f) heading_deg -= 360.0f;
    while (heading_deg < -180.0f) heading_deg += 360.0f;
    return heading_deg;
}

static float AttitudeControl_Wrap360(float heading_deg)
{
    while (heading_deg < 0.0f) heading_deg += 360.0f;
    while (heading_deg >= 360.0f) heading_deg -= 360.0f;
    return heading_deg;
}

/**
 * @brief 斜率限制（#29）：把 current 朝 target 移动，单次最多移动 max_delta。
 *
 * 和"一阶滤波"不同，这里的速率上界是硬约束，可以直接验证。
 */
static float AttitudeControl_SlewLimit(float current, float target, float max_delta)
{
    float delta;

    delta = target - current;
    if (delta > max_delta) delta = max_delta;
    if (delta < -max_delta) delta = -max_delta;
    return current + delta;
}

static float AttitudeControl_StickToAngleDeg(int16_t stick)
{
    int16_t stick_used;

    if (stick > ATTITUDE_STICK_DEADZONE) stick_used = (int16_t)(stick - ATTITUDE_STICK_DEADZONE);
    else if (stick < -ATTITUDE_STICK_DEADZONE) stick_used = (int16_t)(stick + ATTITUDE_STICK_DEADZONE);
    else return 0.0f;
    return ((float)stick_used / (ATTITUDE_STICK_ABS_MAX - (float)ATTITUDE_STICK_DEADZONE)) * ATTITUDE_STICK_MAX_ANGLE_DEG;
}

static float AttitudeControl_UpdateAxis(CascadePid_t *axis, float angle_setpoint_deg, float angle_measured_deg, float rate_measured_dps, float dt_s, float mixer_sign)
{
    float outer_out;
    float inner_out;

    outer_out = CascadePid_UpdateOuter(axis, angle_setpoint_deg, angle_measured_deg, dt_s);
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

static float AttitudeControl_UpdateYaw(float mag_heading_deg, bool mag_heading_valid, int16_t yaw_stick, float yaw_rate_dps, float roll_deg, float pitch_deg, float dt_s)
{
    bool stick_deflected;
    bool heading_ok;
    float heading_error_deg;
    float target_rate_dps;

    stick_deflected = (yaw_stick > ATTITUDE_YAW_STICK_DEADZONE) || (yaw_stick < -ATTITUDE_YAW_STICK_DEADZONE);
    heading_ok = mag_heading_valid && (AttitudeControl_Absf(roll_deg) < ATTITUDE_YAW_TILT_FREEZE_DEG) && (AttitudeControl_Absf(pitch_deg) < ATTITUDE_YAW_TILT_FREEZE_DEG);

    if (stick_deflected)
    {
        /* 打方向时外环让路，回中后再用当时航向当新目标。 */
        Pid_Reset(&yaw_cascade.outer);
        yaw_hold_valid = false;
        yaw_stick_was_deflected = true;
        target_rate_dps = ((float)yaw_stick / ATTITUDE_STICK_ABS_MAX) * ATTITUDE_YAW_STICK_MAX_RATE_DPS;
    }
    else if (heading_ok)
    {
        mag_heading_deg = AttitudeControl_Wrap360(mag_heading_deg);
        if (!yaw_hold_valid || yaw_stick_was_deflected) yaw_hold_heading_deg = mag_heading_deg;
        yaw_hold_valid = true;
        yaw_stick_was_deflected = false;
        /* PID 误差 = 0 - 测量。测量放「当前相对目标的偏差」，机头顺时针偏过则测量为正。 */
        heading_error_deg = AttitudeControl_Wrap180(mag_heading_deg - yaw_hold_heading_deg);
        target_rate_dps = CascadePid_UpdateOuter(&yaw_cascade, 0.0f, heading_error_deg, dt_s);
    }
    else
    {
        /* 磁航向不可用或倾角太大：冻外环，内环只阻尼角速度。 */
        Pid_Reset(&yaw_cascade.outer);
        target_rate_dps = 0.0f;
    }

    /* 三个来源（打杆 / 航向外环 / 冻结归零）的目标角速度统一做斜率限制（#29），
       避免打杆或回中瞬间速率目标一步到位，把偏航内环 P 项直接打满。 */
    yaw_rate_setpoint_limited_dps = AttitudeControl_SlewLimit(yaw_rate_setpoint_limited_dps, target_rate_dps, ATTITUDE_YAW_RATE_SLEW_DPS * dt_s);
    yaw_cascade.inner_setpoint = yaw_rate_setpoint_limited_dps;

    if (AttitudeControl_InnerEnabled(&yaw_cascade))
    {
        return CascadePid_UpdateInner(&yaw_cascade, yaw_rate_dps, dt_s);
    }
    return yaw_rate_setpoint_limited_dps;
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
    yaw_hold_heading_deg = 0.0f;
    yaw_hold_valid = false;
    yaw_stick_was_deflected = false;
    /* 交付比例一并复位，避免解锁瞬间沿用上锁前的旧缩放系数。 */
    last_correction_scale = 1.0f;
    /* 斜率限制后的目标值一并复位（#29），解锁后从 0 平滑爬到当前杆位。 */
    roll_setpoint_limited_deg = 0.0f;
    pitch_setpoint_limited_deg = 0.0f;
    yaw_rate_setpoint_limited_dps = 0.0f;
}

/**
 * @brief 三轴外环加内环，按已测符号送进混控。
 */
bool AttitudeControl_Update(float throttle_us, const IMU_BodyAttitude_t *body, float mag_heading_deg, bool mag_heading_valid, int16_t yaw_stick, int16_t roll_stick, int16_t pitch_stick, float roll_trim_deg, float pitch_trim_deg, float dt_s, MixerOutput_t *output)
{
    MixerInput_t mixer_input;
    float roll_setpoint_raw_deg;
    float pitch_setpoint_raw_deg;

    if ((body == NULL) || (output == NULL))
    {
        return false;
    }

    /* anti-windup（#27）：把上一拍混控的交付比例喂给三个内环，
       内环积分限幅会按它收缩；外环输出是目标角速度，不经过混控缩放，保持 1.0。 */
    CascadePid_SetInnerDeliveryScale(&roll_cascade, last_correction_scale);
    CascadePid_SetInnerDeliveryScale(&pitch_cascade, last_correction_scale);
    CascadePid_SetInnerDeliveryScale(&yaw_cascade, last_correction_scale);

    /* 杆量映射成目标角后再加 Trim；Trim 只偏置中位，不把满杆角再放大。
       映射结果先过斜率限制（#29），杆量/Trim 跳变不会让目标角一步到位。 */
    roll_setpoint_raw_deg = AttitudeControl_StickToAngleDeg(roll_stick) + roll_trim_deg;
    /* 俯仰摇杆按 ATTITUDE_PITCH_STICK_SIGN 取向后，再加 Trim。
       Trim 加在取反之后，免得它的符号跟着一起翻。 */
    pitch_setpoint_raw_deg = (ATTITUDE_PITCH_STICK_SIGN * AttitudeControl_StickToAngleDeg(pitch_stick)) + pitch_trim_deg;
    roll_setpoint_limited_deg = AttitudeControl_SlewLimit(roll_setpoint_limited_deg, roll_setpoint_raw_deg, ATTITUDE_TARGET_SLEW_DPS * dt_s);
    pitch_setpoint_limited_deg = AttitudeControl_SlewLimit(pitch_setpoint_limited_deg, pitch_setpoint_raw_deg, ATTITUDE_TARGET_SLEW_DPS * dt_s);
    mixer_input.throttle_us = throttle_us;
    mixer_input.roll_correction_us = AttitudeControl_UpdateAxis(&roll_cascade, roll_setpoint_limited_deg, body->roll_deg, body->roll_rate_dps, dt_s, 1.0f);
    mixer_input.pitch_correction_us = AttitudeControl_UpdateAxis(&pitch_cascade, pitch_setpoint_limited_deg, body->pitch_deg, body->pitch_rate_dps, dt_s, -1.0f);
    mixer_input.yaw_correction_us = AttitudeControl_UpdateYaw(mag_heading_deg, mag_heading_valid, yaw_stick, body->yaw_rate_dps, body->roll_deg, body->pitch_deg, dt_s);
    last_roll_correction_us = mixer_input.roll_correction_us;
    last_pitch_correction_us = mixer_input.pitch_correction_us;
    last_yaw_correction_us = mixer_input.yaw_correction_us;

    if (!Mixer_Calculate(&mixer_input, MIXER_PWM_EFFECTIVE_MIN_US, MOTOR_PWM_MAX_US, output))
    {
        return false;
    }

    /* 记录本拍实际交付比例，供下一拍三个内环的 anti-windup 收缩积分限幅。 */
    last_correction_scale = output->correction_scale;

    return true;
}

void AttitudeControl_GetLastCorrections(float *roll_us, float *pitch_us, float *yaw_us)
{
    if (roll_us != NULL) *roll_us = last_roll_correction_us;
    if (pitch_us != NULL) *pitch_us = last_pitch_correction_us;
    if (yaw_us != NULL) *yaw_us = last_yaw_correction_us;
}

/**
 * @brief 横滚、俯仰内环是否都已启用，用于解锁前门禁。
 */
bool AttitudeControl_IsInnerLoopEnabled(void)
{
    return AttitudeControl_InnerEnabled(&roll_cascade) && AttitudeControl_InnerEnabled(&pitch_cascade);
}
