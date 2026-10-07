/* OF6 位置/速度两级环宿主机仿真：真实编译 position_control.c。
 *
 * gcc -std=c99 -Wall -Wextra -Werror -Wno-unused-parameter -DPOSITION_CONTROL_SIM \
 *   -I flow_stub -I "../../ApexFly v2.0/Modules/Control" \
 *   sim_position_control.c "../../ApexFly v2.0/Modules/Control/position_control.c" \
 *   -o sim_position_control.exe
 */

#include "position_control.h"

#include <stdio.h>
#include <stdlib.h>

#define COS_YAW 0.5878f
#define SIN_YAW 0.8090f
#define DT_S 0.040f

static int checks;
static int fails;

static float absf_local(float value)
{
    return (value < 0.0f) ? -value : value;
}

static void check(const char *name, int ok)
{
    checks++;
    if (!ok) fails++;
    printf("[%s] %s\n", ok ? "OK" : "失败", name);
}

static PositionControlSnapshot_t snap(void)
{
    PositionControlSnapshot_t snapshot;

    PositionControl_GetSnapshot(&snapshot);
    return snapshot;
}

/* 把机体系前/右量反旋回模块 XY，给产品代码验证正向旋转。 */
static void body_to_flow(float forward, float right, float *x, float *y)
{
    *x = (COS_YAW * forward) - (SIN_YAW * right);
    *y = (SIN_YAW * forward) + (COS_YAW * right);
}

static void update_body(float position_forward, float position_right,
                        float velocity_forward, float velocity_right)
{
    float sum_x;
    float sum_y;
    float vx;
    float vy;

    body_to_flow(position_forward, position_right, &sum_x, &sum_y);
    body_to_flow(velocity_forward, velocity_right, &vx, &vy);
    PositionControl_Update(sum_x, sum_y, vx, vy, DT_S);
}

static void init_ready(void)
{
    PositionControl_Init();
    PositionControl_SetSafetyContext(true, true, true, false);
    PositionControl_SetRequest(true);
    PositionControl_SetFlowAvailable(true);
}

static void test_forward_return(void)
{
    PositionControlSnapshot_t s;

    init_ready();
    update_body(0.0f, 0.0f, 0.0f, 0.0f); /* 使能时捕获当前位置为新原点。 */
    update_body(100.0f, 0.0f, 0.0f, 0.0f);
    s = snap();

    check("1 坐标旋转：向前 100 count -> forward=100、right=0",
          (absf_local(s.position_forward_counts - 100.0f) < 0.1f) && (absf_local(s.position_right_counts) < 0.1f));
    check("1 频率：两帧速度环、一次位置环", (s.velocity_update_count == 2U) && (s.position_update_count == 1U));
    check("1 向前偏 -> pitch 为正（抬头后拉）", (s.raw_pitch_deg > 0.49f) && (s.raw_pitch_deg < 0.51f));
    check("1 roll 基本为零", absf_local(s.raw_roll_deg) < 0.01f);
    check("1 低通输出介于 0 与原始输出之间", (s.pitch_deg > 0.0f) && (s.pitch_deg < s.raw_pitch_deg));
}

static void test_right_return(void)
{
    PositionControlSnapshot_t s;

    init_ready();
    update_body(0.0f, 0.0f, 0.0f, 0.0f);
    update_body(0.0f, 100.0f, 0.0f, 0.0f);
    s = snap();

    check("2 坐标旋转：向右 100 count -> right=100、forward=0",
          (absf_local(s.position_right_counts - 100.0f) < 0.1f) && (absf_local(s.position_forward_counts) < 0.1f));
    check("2 向右偏 -> roll 为负（左倾回拉）", (s.raw_roll_deg < -0.49f) && (s.raw_roll_deg > -0.51f));
    check("2 pitch 基本为零", absf_local(s.raw_pitch_deg) < 0.01f);
}

static void test_velocity_and_limit(void)
{
    PositionControlSnapshot_t s;

    init_ready();
    update_body(0.0f, 0.0f, 0.0f, 0.0f);
    update_body(1000.0f, 0.0f, 1000.0f, 0.0f);
    s = snap();

    check("3 外环速度目标钳在 -200 count/s", (s.velocity_forward_sp_cps < -199.9f) && (s.velocity_forward_sp_cps > -200.1f));
    check("3 向前速度会加强后拉，角度钳在 +8deg", (s.raw_pitch_deg > 7.99f) && (s.raw_pitch_deg < 8.01f));
}

static void test_invalid_reset(void)
{
    PositionControlSnapshot_t s;

    PositionControl_Init();
    PositionControl_SetSafetyContext(true, true, true, false);
    PositionControl_SetRequest(true);
    PositionControl_SetFlowAvailable(false);
    s = snap();
    check("4 光流失效 -> ok=0", !s.valid);
    check("4 光流失效 -> 目标与滤波输出全归零",
          (s.velocity_forward_sp_cps == 0.0f) && (s.velocity_right_sp_cps == 0.0f) &&
          (s.raw_roll_deg == 0.0f) && (s.raw_pitch_deg == 0.0f) &&
          (s.roll_deg == 0.0f) && (s.pitch_deg == 0.0f));

    PositionControl_SetFlowAvailable(true);
    update_body(0.0f, 0.0f, 0.0f, 0.0f);
    update_body(0.0f, 0.0f, 0.0f, 0.0f);
    s = snap();
    check("4 合格帧恢复后重新开始计算", s.valid && (s.position_update_count == 1U));
}

static void test_gate_latch_and_reanchor(void)
{
    PositionControlSnapshot_t s;

    init_ready();
    update_body(0.0f, 0.0f, 0.0f, 0.0f);
    update_body(100.0f, 0.0f, 0.0f, 0.0f);
    PositionControl_SetPilotContext(true, false);
    update_body(200.0f, 0.0f, 0.0f, 0.0f);
    s = snap();
    check("5 摇杆离开中心 -> PILOT_MOVE 且输出归零",
          (s.state == (uint8_t)POSITION_CONTROL_STATE_PILOT_MOVE) && !s.valid && (s.pitch_deg == 0.0f));

    PositionControl_SetPilotContext(true, true);
    update_body(200.0f, 0.0f, 0.0f, 0.0f);
    update_body(200.0f, 0.0f, 0.0f, 0.0f);
    update_body(200.0f, 0.0f, 0.0f, 0.0f);
    update_body(200.0f, 0.0f, 0.0f, 0.0f);
    update_body(200.0f, 0.0f, 0.0f, 0.0f);
    s = snap();
    check("5 摇杆回中 200ms -> 在当前位置重新锚定", s.valid && (s.state == (uint8_t)POSITION_CONTROL_STATE_ACTIVE) && (absf_local(s.position_forward_counts) < 0.1f));

    PositionControl_SetFlowAvailable(false);
    s = snap();
    check("5 光流失效 -> FAULT_LATCHED", s.state == (uint8_t)POSITION_CONTROL_STATE_LATCHED);
}

static void test_safety_gate(void)
{
    PositionControlSnapshot_t s;

    init_ready();
    update_body(0.0f, 0.0f, 0.0f, 0.0f);
    PositionControl_SetSafetyContext(true, false, true, false);
    update_body(0.0f, 0.0f, 0.0f, 0.0f);
    s = snap();
    check("6 姿态超限门禁 -> FAULT_LATCHED", (s.state == (uint8_t)POSITION_CONTROL_STATE_LATCHED) &&
          (s.fault_reason == (uint8_t)POSITION_CONTROL_FAULT_ATTITUDE));
    check("6 姿态故障 -> 定点输出归零", !s.valid && (s.roll_deg == 0.0f) && (s.pitch_deg == 0.0f));

    PositionControl_SetSafetyContext(true, true, true, false);
    update_body(0.0f, 0.0f, 0.0f, 0.0f);
    s = snap();
    check("6 安全条件恢复后仍保持锁存", s.state == (uint8_t)POSITION_CONTROL_STATE_LATCHED);

    PositionControl_SetRequest(false);
    PositionControl_SetRequest(true);
    update_body(0.0f, 0.0f, 0.0f, 0.0f);
    s = snap();
    check("6 关开请求后重新进入 ACTIVE", (s.state == (uint8_t)POSITION_CONTROL_STATE_ACTIVE) &&
          (s.fault_reason == (uint8_t)POSITION_CONTROL_FAULT_NONE));
}

static void test_safety_waiting(void)
{
    PositionControlSnapshot_t s;

    PositionControl_Init();
    PositionControl_SetSafetyContext(true, true, false, false);
    PositionControl_SetRequest(true);
    PositionControl_SetFlowAvailable(true);
    update_body(0.0f, 0.0f, 0.0f, 0.0f);
    s = snap();
    check("7 启动时高度门禁未通过 -> WAITING", (s.state == (uint8_t)POSITION_CONTROL_STATE_WAITING) &&
          (s.fault_reason == (uint8_t)POSITION_CONTROL_FAULT_HEIGHT));

    PositionControl_SetSafetyContext(true, true, true, false);
    update_body(0.0f, 0.0f, 0.0f, 0.0f);
    s = snap();
    check("7 高度门禁恢复后允许 ACTIVE", s.state == (uint8_t)POSITION_CONTROL_STATE_ACTIVE);
}

static int output_is_zero(const PositionControlSnapshot_t *s)
{
    return !s->valid && (s->velocity_forward_sp_cps == 0.0f) &&
           (s->velocity_right_sp_cps == 0.0f) && (s->raw_roll_deg == 0.0f) &&
           (s->raw_pitch_deg == 0.0f) && (s->roll_deg == 0.0f) && (s->pitch_deg == 0.0f);
}

static void test_pilot_disarm_cycles(void)
{
    PositionControlSnapshot_t s;
    int cycle;

    init_ready();
    update_body(0.0f, 0.0f, 0.0f, 0.0f);
    for (cycle = 0; cycle < 2; cycle++)
    {
        float next_forward = 600.0f + 300.0f * (float)cycle;
        float next_right = -200.0f - 100.0f * (float)cycle;

        update_body(next_forward - 100.0f, next_right, 30.0f, -20.0f);
        s = snap();
        check("8 上锁前定点正在输出", s.valid &&
              ((absf_local(s.roll_deg) > 0.01f) || (absf_local(s.pitch_deg) > 0.01f)));

        PositionControl_SetSafetyContext(false, true, true, true);
        s = snap();
        check("8 手动上锁许可发布后立即禁止旧输出", output_is_zero(&s));
        PositionControl_SetMotionSuppressed(true);
        PositionControl_SetFlowAvailable(true); /* 无新光流帧，只有 Flow 轮询。 */
        s = snap();
        check("8 旋转冻结/无新帧时手动上锁也退 WAITING",
              (s.state == POSITION_CONTROL_STATE_WAITING) && (s.fault_reason == POSITION_CONTROL_FAULT_NONE));
        check("8 等待态保留请求，许可门禁关闭",
              s.request && (s.gate_mask == 0x0FU) && output_is_zero(&s));

        PositionControl_SetMotionSuppressed(false);
        update_body(next_forward, next_right, 0.0f, 0.0f);
        update_body(next_forward, next_right, 0.0f, 0.0f);
        s = snap();
        check("8 调参期间持续保持 WAITING，无 fr=5",
              (s.state == POSITION_CONTROL_STATE_WAITING) && (s.fault_reason == POSITION_CONTROL_FAULT_NONE));

        PositionControl_SetSafetyContext(true, true, true, false);
        PositionControl_SetFlowAvailable(true);
        update_body(next_forward, next_right, 0.0f, 0.0f);
        s = snap();
        check("8 不关开定点请求，重新解锁即可 ACTIVE",
              s.valid && (s.state == POSITION_CONTROL_STATE_ACTIVE) && (s.fault_reason == POSITION_CONTROL_FAULT_NONE));
        check("8 重新解锁捕获新原点，旧位置和滤波输出不带入",
              (absf_local(s.position_forward_counts) < 0.1f) &&
              (absf_local(s.position_right_counts) < 0.1f) &&
              (s.raw_roll_deg == 0.0f) && (s.raw_pitch_deg == 0.0f) &&
              (s.roll_deg == 0.0f) && (s.pitch_deg == 0.0f));
    }
}

static void test_fault_latch_survives_pilot_disarm(void)
{
    PositionControlSnapshot_t s;
    uint8_t reason;

    for (reason = POSITION_CONTROL_FAULT_FLOW; reason <= POSITION_CONTROL_FAULT_PERMISSION; reason++)
    {
        init_ready();
        update_body(0.0f, 0.0f, 0.0f, 0.0f);
        if (reason == POSITION_CONTROL_FAULT_FLOW) PositionControl_SetFlowAvailable(false);
        else if (reason == POSITION_CONTROL_FAULT_LINK) PositionControl_SetPilotContext(false, true);
        else PositionControl_SetSafetyContext(reason != POSITION_CONTROL_FAULT_PERMISSION,
                                              reason != POSITION_CONTROL_FAULT_ATTITUDE,
                                              reason != POSITION_CONTROL_FAULT_HEIGHT, false);
        update_body(0.0f, 0.0f, 0.0f, 0.0f);
        s = snap();
        check("9 真实故障仍按原因锁存",
              (s.state == POSITION_CONTROL_STATE_LATCHED) && (s.fault_reason == reason));

        PositionControl_SetPilotContext(true, true);
        PositionControl_SetSafetyContext(false, true, true, true);
        PositionControl_SetFlowAvailable(false);
        update_body(300.0f, 0.0f, 0.0f, 0.0f);
        s = snap();
        check("9 后续手动上锁及光流失效不清除或覆盖已有故障",
              (s.state == POSITION_CONTROL_STATE_LATCHED) && (s.fault_reason == reason) && output_is_zero(&s));

        PositionControl_SetSafetyContext(true, true, true, false);
        PositionControl_SetFlowAvailable(true);
        update_body(300.0f, 0.0f, 0.0f, 0.0f);
        s = snap();
        check("9 真实故障恢复再解锁也不会自动重启",
              (s.state == POSITION_CONTROL_STATE_LATCHED) && (s.fault_reason == reason));
    }
}

static void test_pilot_disarm_waiting_gates(void)
{
    PositionControlSnapshot_t s;

    init_ready();
    update_body(0.0f, 0.0f, 0.0f, 0.0f);
    PositionControl_SetPilotContext(true, false);
    update_body(100.0f, 0.0f, 0.0f, 0.0f);
    PositionControl_SetSafetyContext(false, true, true, true);
    update_body(100.0f, 0.0f, 0.0f, 0.0f);
    s = snap();
    check("10 手动飞行态上锁同样退 WAITING",
          (s.state == POSITION_CONTROL_STATE_WAITING) && (s.fault_reason == POSITION_CONTROL_FAULT_NONE) && output_is_zero(&s));

    PositionControl_SetFlowAvailable(false);
    s = snap();
    check("10 正常上锁且无光流时仍等待，不误锁存",
          (s.state == POSITION_CONTROL_STATE_WAITING) && (s.fault_reason == POSITION_CONTROL_FAULT_NONE));
    PositionControl_SetPilotContext(true, true);
    PositionControl_SetSafetyContext(true, true, true, false);
    update_body(200.0f, 0.0f, 0.0f, 0.0f);
    s = snap();
    check("10 再解锁不能绕过光流门禁", !s.valid && (s.state == POSITION_CONTROL_STATE_WAITING));
    PositionControl_SetFlowAvailable(true);
    update_body(200.0f, 0.0f, 0.0f, 0.0f);
    s = snap();
    check("10 全部门禁恢复后重新锚定并工作", s.valid &&
          (s.state == POSITION_CONTROL_STATE_ACTIVE) && (absf_local(s.position_forward_counts) < 0.1f));

    PositionControl_SetSafetyContext(false, true, true, true);
    PositionControl_SetRequest(false);
    PositionControl_SetFlowAvailable(true);
    update_body(0.0f, 0.0f, 0.0f, 0.0f);
    s = snap();
    check("10 定点请求关闭时维持 DISABLED", !s.request && (s.state == POSITION_CONTROL_STATE_DISABLED));
}

int main(void)
{
    printf("OF6/OF7/OF8 位置控制仿真：旋转 / 两级环 / 门禁 / 锚定 / 限幅 / 低通 / 失效\n");
    test_forward_return();
    test_right_return();
    test_velocity_and_limit();
    test_invalid_reset();
    test_gate_latch_and_reanchor();
    test_safety_gate();
    test_safety_waiting();
    test_pilot_disarm_cycles();
    test_fault_latch_survives_pilot_disarm();
    test_pilot_disarm_waiting_gates();
    printf("\n===== 检查 %d 项，失败 %d 项 =====\n", checks, fails);
    return (fails == 0) ? 0 : 1;
}
