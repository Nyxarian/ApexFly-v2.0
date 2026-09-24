/**
 * @file position_control.c
 * @brief 光流定点 OF6：58° 坐标旋转、位置 P 环、速度 P 环、输出低通。
 *
 * 机体系约定：X 朝前、Y 朝右；roll 正为右倾，pitch 正为抬头。
 * 因而飞机向前偏时应抬头后拉，向右偏时应左倾回拉。
 */

#include "position_control.h"

#include <stddef.h>

/* 光流相对机体系的初始 yaw 安装角。OF6 台架四方向复核后才允许改。 */
#define POSITION_CONTROL_COS_YAW 0.5299f
#define POSITION_CONTROL_SIN_YAW 0.8480f

/* 位置环每两帧更新一次：25Hz 光流下约 12.5Hz，避开约 80ms 的测量滞后。 */
#define POSITION_CONTROL_OUTER_DIVIDER 2U
/* 外环：position[count] -> velocity setpoint[count/s]。 */
#define POSITION_CONTROL_POSITION_KP_S_INV 0.50f
#define POSITION_CONTROL_VELOCITY_SP_LIMIT_CPS 200.0f
/* 内环：velocity error[count/s] -> desired attitude[deg]。 */
#define POSITION_CONTROL_VELOCITY_KP_DEG_PER_CPS 0.010f
#define POSITION_CONTROL_ANGLE_LIMIT_DEG 3.0f
/* 一阶低通：fc=2.5Hz，tau=1/(2*pi*fc)。不调用 libm。 */
#define POSITION_CONTROL_FILTER_TAU_S 0.063662f
#define POSITION_CONTROL_RECENTER_FRAMES 5U /* 25Hz × 200ms */
/* OF10 接入后请求来自控制帧；上电默认关闭，避免遥控器尚未明确请求时进入定点。 */
#define POSITION_CONTROL_DIAGNOSTIC_REQUEST_DEFAULT false

static PositionControlSnapshot_t position_snapshot;
static uint8_t position_outer_divider;
static bool position_request;
static bool position_link_ok;
static bool position_sticks_centered;
static bool position_control_permission;
static bool position_flow_available;
static bool position_latched;
static uint8_t position_recenter_frames;
static float position_anchor_x;
static float position_anchor_y;

static float PositionControl_Clamp(float value, float lower, float upper)
{
    if (value > upper) return upper;
    if (value < lower) return lower;
    return value;
}

static void PositionControl_ResetOutput(void)
{
    position_outer_divider = 0U;
    position_snapshot.valid = false;
    position_snapshot.velocity_forward_sp_cps = 0.0f;
    position_snapshot.velocity_right_sp_cps = 0.0f;
    position_snapshot.raw_roll_deg = 0.0f;
    position_snapshot.raw_pitch_deg = 0.0f;
    position_snapshot.roll_deg = 0.0f;
    position_snapshot.pitch_deg = 0.0f;
}

static void PositionControl_LatchOff(void)
{
    position_latched = true;
    position_recenter_frames = 0U;
    position_snapshot.state = (uint8_t)POSITION_CONTROL_STATE_LATCHED;
    PositionControl_ResetOutput();
}

static uint8_t PositionControl_GetGateMask(void)
{
    uint8_t mask = 0U;

    if (position_request) mask |= POSITION_CONTROL_GATE_REQUEST;
    if (position_link_ok) mask |= POSITION_CONTROL_GATE_LINK;
    if (position_flow_available) mask |= POSITION_CONTROL_GATE_FLOW;
    if (position_sticks_centered) mask |= POSITION_CONTROL_GATE_STICKS;
    if (position_control_permission) mask |= POSITION_CONTROL_GATE_PERMISSION;
    return mask;
}

static bool PositionControl_AllGatesOpen(uint8_t mask)
{
    return mask == (POSITION_CONTROL_GATE_REQUEST |
                    POSITION_CONTROL_GATE_LINK |
                    POSITION_CONTROL_GATE_FLOW |
                    POSITION_CONTROL_GATE_STICKS |
                    POSITION_CONTROL_GATE_PERMISSION);
}

void PositionControl_Init(void)
{
    position_request = POSITION_CONTROL_DIAGNOSTIC_REQUEST_DEFAULT;
    position_link_ok = true;
    position_sticks_centered = true;
    position_control_permission = true;
    position_flow_available = false;
    position_latched = false;
    position_recenter_frames = 0U;
    position_anchor_x = 0.0f;
    position_anchor_y = 0.0f;

    position_snapshot.valid = false;
    position_snapshot.request = position_request;
    position_snapshot.gate_mask = 0U;
    position_snapshot.state = (uint8_t)POSITION_CONTROL_STATE_WAITING;
    position_snapshot.velocity_update_count = 0U;
    position_snapshot.position_update_count = 0U;
    position_snapshot.position_forward_counts = 0.0f;
    position_snapshot.position_right_counts = 0.0f;
    position_snapshot.velocity_forward_cps = 0.0f;
    position_snapshot.velocity_right_cps = 0.0f;
    position_snapshot.anchor_x_counts = 0.0f;
    position_snapshot.anchor_y_counts = 0.0f;
    PositionControl_ResetOutput();
}

void PositionControl_SetRequest(bool requested)
{
    position_request = requested;
    position_snapshot.request = requested;
    position_snapshot.gate_mask = PositionControl_GetGateMask();
    if (!requested)
    {
        position_latched = false;
        position_anchor_x = 0.0f;
        position_anchor_y = 0.0f;
        position_recenter_frames = 0U;
        position_snapshot.state = (uint8_t)POSITION_CONTROL_STATE_DISABLED;
        PositionControl_ResetOutput();
    }
}

void PositionControl_SetGateContext(bool link_ok, bool sticks_centered, bool control_permission)
{
    position_link_ok = link_ok;
    position_sticks_centered = sticks_centered;
    position_control_permission = control_permission;
    position_snapshot.gate_mask = PositionControl_GetGateMask();

    if ((position_snapshot.state == (uint8_t)POSITION_CONTROL_STATE_ACTIVE ||
         position_snapshot.state == (uint8_t)POSITION_CONTROL_STATE_PILOT_MOVE) &&
        (!link_ok || !control_permission))
    {
        PositionControl_LatchOff();
    }
    else if ((position_snapshot.state == (uint8_t)POSITION_CONTROL_STATE_ACTIVE) && !sticks_centered)
    {
        position_snapshot.state = (uint8_t)POSITION_CONTROL_STATE_PILOT_MOVE;
        position_recenter_frames = 0U;
        PositionControl_ResetOutput();
    }
}

void PositionControl_Update(float sum_x, float sum_y, float vx_cps, float vy_cps, float dt_s)
{
    float rotated_position_forward;
    float rotated_position_right;
    float rotated_velocity_forward;
    float rotated_velocity_right;
    uint8_t gate_mask;
    float alpha;
    float velocity_forward_error;
    float velocity_right_error;

    if (dt_s <= 0.0f) return;

    /* 模块坐标 -> 机体系。位置与速度必须使用同一旋转，不能各写一套符号。 */
    rotated_position_forward = (POSITION_CONTROL_COS_YAW * sum_x) + (POSITION_CONTROL_SIN_YAW * sum_y);
    rotated_position_right = (-POSITION_CONTROL_SIN_YAW * sum_x) + (POSITION_CONTROL_COS_YAW * sum_y);
    rotated_velocity_forward = (POSITION_CONTROL_COS_YAW * vx_cps) + (POSITION_CONTROL_SIN_YAW * vy_cps);
    rotated_velocity_right = (-POSITION_CONTROL_SIN_YAW * vx_cps) + (POSITION_CONTROL_COS_YAW * vy_cps);

    gate_mask = PositionControl_GetGateMask();
    position_snapshot.request = position_request;
    position_snapshot.gate_mask = gate_mask;

    if (position_latched)
    {
        position_snapshot.state = (uint8_t)POSITION_CONTROL_STATE_LATCHED;
        PositionControl_ResetOutput();
        return;
    }

    if (!position_request)
    {
        position_snapshot.state = (uint8_t)POSITION_CONTROL_STATE_DISABLED;
        PositionControl_ResetOutput();
        return;
    }

    if (!position_link_ok || !position_flow_available || !position_control_permission)
    {
        position_snapshot.state = (uint8_t)POSITION_CONTROL_STATE_WAITING;
        PositionControl_ResetOutput();
        return;
    }

    if (!position_sticks_centered)
    {
        if (position_snapshot.state != (uint8_t)POSITION_CONTROL_STATE_PILOT_MOVE)
        {
            position_snapshot.state = (uint8_t)POSITION_CONTROL_STATE_PILOT_MOVE;
            PositionControl_ResetOutput();
        }
        position_snapshot.position_forward_counts = rotated_position_forward - position_anchor_x;
        position_snapshot.position_right_counts = rotated_position_right - position_anchor_y;
        position_snapshot.velocity_forward_cps = rotated_velocity_forward;
        position_snapshot.velocity_right_cps = rotated_velocity_right;
        position_snapshot.velocity_update_count++;
        return;
    }

    if (position_snapshot.state == (uint8_t)POSITION_CONTROL_STATE_PILOT_MOVE)
    {
        position_recenter_frames++;
        if (position_recenter_frames < POSITION_CONTROL_RECENTER_FRAMES)
        {
            position_snapshot.position_forward_counts = rotated_position_forward - position_anchor_x;
            position_snapshot.position_right_counts = rotated_position_right - position_anchor_y;
            position_snapshot.velocity_forward_cps = rotated_velocity_forward;
            position_snapshot.velocity_right_cps = rotated_velocity_right;
            position_snapshot.velocity_update_count++;
            PositionControl_ResetOutput();
            return;
        }

        /* 手动飞行结束：把当前机体位置作为新的定点原点。 */
        position_anchor_x = rotated_position_forward;
        position_anchor_y = rotated_position_right;
        position_snapshot.anchor_x_counts = position_anchor_x;
        position_snapshot.anchor_y_counts = position_anchor_y;
        position_snapshot.state = (uint8_t)POSITION_CONTROL_STATE_ACTIVE;
        PositionControl_ResetOutput();
    }

    if (!PositionControl_AllGatesOpen(gate_mask))
    {
        position_snapshot.state = (uint8_t)POSITION_CONTROL_STATE_WAITING;
        PositionControl_ResetOutput();
        return;
    }

    if (position_snapshot.state != (uint8_t)POSITION_CONTROL_STATE_ACTIVE)
    {
        /* 无扰切入：当前位置就是新的定点原点，旧累计量不带入。 */
        position_anchor_x = rotated_position_forward;
        position_anchor_y = rotated_position_right;
        position_snapshot.anchor_x_counts = position_anchor_x;
        position_snapshot.anchor_y_counts = position_anchor_y;
        position_snapshot.state = (uint8_t)POSITION_CONTROL_STATE_ACTIVE;
        PositionControl_ResetOutput();
    }

    position_snapshot.position_forward_counts = rotated_position_forward - position_anchor_x;
    position_snapshot.position_right_counts = rotated_position_right - position_anchor_y;
    position_snapshot.velocity_forward_cps = rotated_velocity_forward;
    position_snapshot.velocity_right_cps = rotated_velocity_right;
    position_snapshot.velocity_update_count++;

    position_outer_divider++;
    if (position_outer_divider >= POSITION_CONTROL_OUTER_DIVIDER)
    {
        position_outer_divider = 0U;
        /* 目标恒为原点：偏前为正时给负速度目标，要求它向后回来。 */
        position_snapshot.velocity_forward_sp_cps = PositionControl_Clamp(
            -POSITION_CONTROL_POSITION_KP_S_INV * position_snapshot.position_forward_counts,
            -POSITION_CONTROL_VELOCITY_SP_LIMIT_CPS,
            POSITION_CONTROL_VELOCITY_SP_LIMIT_CPS);
        position_snapshot.velocity_right_sp_cps = PositionControl_Clamp(
            -POSITION_CONTROL_POSITION_KP_S_INV * position_snapshot.position_right_counts,
            -POSITION_CONTROL_VELOCITY_SP_LIMIT_CPS,
            POSITION_CONTROL_VELOCITY_SP_LIMIT_CPS);
        position_snapshot.position_update_count++;
    }

    velocity_forward_error = position_snapshot.velocity_forward_sp_cps - position_snapshot.velocity_forward_cps;
    velocity_right_error = position_snapshot.velocity_right_sp_cps - position_snapshot.velocity_right_cps;

    /* 前向加速度由 pitch 负（低头）产生；右向加速度由 roll 正（右倾）产生。 */
    position_snapshot.raw_roll_deg = PositionControl_Clamp(
        POSITION_CONTROL_VELOCITY_KP_DEG_PER_CPS * velocity_right_error,
        -POSITION_CONTROL_ANGLE_LIMIT_DEG,
        POSITION_CONTROL_ANGLE_LIMIT_DEG);
    position_snapshot.raw_pitch_deg = PositionControl_Clamp(
        -POSITION_CONTROL_VELOCITY_KP_DEG_PER_CPS * velocity_forward_error,
        -POSITION_CONTROL_ANGLE_LIMIT_DEG,
        POSITION_CONTROL_ANGLE_LIMIT_DEG);

    alpha = dt_s / (POSITION_CONTROL_FILTER_TAU_S + dt_s);
    alpha = PositionControl_Clamp(alpha, 0.0f, 1.0f);
    position_snapshot.roll_deg += alpha * (position_snapshot.raw_roll_deg - position_snapshot.roll_deg);
    position_snapshot.pitch_deg += alpha * (position_snapshot.raw_pitch_deg - position_snapshot.pitch_deg);
    position_snapshot.valid = true;
}

void PositionControl_SetFlowAvailable(bool available)
{
    position_flow_available = available;
    position_snapshot.gate_mask = PositionControl_GetGateMask();
    if (!available && (position_snapshot.state == (uint8_t)POSITION_CONTROL_STATE_ACTIVE))
    {
        PositionControl_LatchOff();
    }
}

void PositionControl_GetSnapshot(PositionControlSnapshot_t *snapshot)
{
    if (snapshot == NULL) return;

    /* 写入只发生在 flowTask，读取只发生在同级的 tofTask；两者不会同时运行。
       不在这个纯计算模块引入 FreeRTOS 临界区，宿主机仿真也能直接编译。 */
    *snapshot = position_snapshot;
}
