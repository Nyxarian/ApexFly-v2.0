/* H9（垂速内环 P）的台架仿真：把验收矩阵在**烧录之前**先跑一遍。
   真实参与编译的只有 vertical_control.c 和 pid.c，其余（估计器、参数表）用桩顶掉。

   编译（宿主机 gcc；本文件不碰任何硬件）：
     gcc -std=c99 -Wall -Wextra -Wno-unused-parameter `
       -I sim_stub -I "ApexFly v2.0/Modules/Control" -I "ApexFly v2.0/Modules/Motor" `
       -I "ApexFly v2.0/Modules/Communication" `
       sim_vc_h9.c "ApexFly v2.0/Modules/Control/vertical_control.c" `
       "ApexFly v2.0/Modules/Control/pid.c" -o sim_vc_h9.exe

   说明：motor.h 会 include "main.h"（HAL 聚合头），宿主机上不需要它的任何内容，
        所以用 sim_stub/main.h 顶掉。sim_stub 绝不能加进 Keil 的包含路径。

   ⚠️ 仿真**不能**替代真机验证。它能证明的是：
      ① 控制律的**符号**对（vz 与 corr 反号）—— 这是本环节唯一会真正导致失控的错误；
      ② 限幅、动态限幅、停手闭嘴、参数在线变更的行为正确。
      它证明不了：电机真的转、以及**增益是否合适**。
      增益在台架上也验不了（手是"无限权限"的扰动源，飞机不会真的跑飞），
      只能首飞用 ±50µs 的小限幅去试。真机验收步骤见
      `优化一下/高度接入施工顺序.md` 的「H9 验收步骤」。
*/
#include "vertical_control.h"
#include "vertical_est.h"
#include "pid_table.h"
#include <stdio.h>

/* ── 假的估计器：替换掉真的 vertical_est ── */
static VerticalEstState_t fake_est;

void VerticalEst_GetState(VerticalEstState_t *state) { *state = fake_est; }
void VerticalEst_Update(float roll_deg, float pitch_deg, float dt_s) { (void)roll_deg; (void)pitch_deg; (void)dt_s; }
void VerticalEst_Init(void) {}
void VerticalEst_Reset(void) {}

/* ── 假的参数表：替换掉真的 pid_table（它要 FreeRTOS 临界区 + 协议头）── */
static float stub_vz_kp = 20.0f;
static float stub_vz_ki = 0.0f;
static float stub_vz_kd = 0.0f;
static uint16_t stub_max_throttle_us = 1500U;

void PidTable_GetAltitudeParams(float *vz_kp, float *vz_ki, float *vz_kd)
{
    if (vz_kp != NULL) *vz_kp = stub_vz_kp;
    if (vz_ki != NULL) *vz_ki = stub_vz_ki;
    if (vz_kd != NULL) *vz_kd = stub_vz_kd;
}

/* 与真的 PidTable_GetMaxThrottleUs 一致：先按 1200~2000 夹取再四舍五入。 */
uint16_t PidTable_GetMaxThrottleUs(void)
{
    float v = (float)stub_max_throttle_us;
    if (v < 1200.0f) v = 1200.0f;
    if (v > 2000.0f) v = 2000.0f;
    return (uint16_t)(v + 0.5f);
}

/* ── 从 motor_test.c 抄过来的常量（改那边就必须改这里）── */
#define MT_PWM_MIN_US           1000U
#define MT_SLEW_UP_US_PER_S     800.0f
#define MT_SLEW_DOWN_US_PER_S   300.0f
#define MT_STICK_MAX            1000U

static const float DT = 0.004f; /* 250Hz */
static int fails = 0;

/* ── 与 motor_test.c 的 MotorTest_MapStickToThrottleUs 等价（满杆固定 1500）── */
static uint16_t manual_map(uint16_t stick)
{
    uint32_t span = 1500U - MT_PWM_MIN_US;
    if (stick > MT_STICK_MAX) stick = MT_STICK_MAX;
    return (uint16_t)(MT_PWM_MIN_US + ((span * (uint32_t)stick) / MT_STICK_MAX));
}

/* ── 与 motor_test.c 的 MotorTest_ThrottleRampLimit 等价 ── */
static float ramp_limit(float cur, float target, float dt)
{
    float step;
    if (dt <= 0.0f) return cur;
    step = (target >= cur) ? (MT_SLEW_UP_US_PER_S * dt) : (MT_SLEW_DOWN_US_PER_S * dt);
    if (target > (cur + step)) return cur + step;
    if (target < (cur - step)) return cur - step;
    return target;
}

/* ── 复刻 motor_test.c 的油门来源分叉 ── */
static uint16_t throttle_target_us(uint16_t request_stick)
{
    VerticalControlSnapshot_t s;
    VerticalControl_GetSnapshot(&s);
    if (s.mode == (uint8_t)VERTICAL_MODE_ALT_HOLD)
    {
        return VerticalControl_GetThrottleUs();
    }
    return manual_map(request_stick);
}

/* ── 复刻控制环：Update -> （上锁分支 | 分叉 -> 限速）── */
static float slew_us = (float)MT_PWM_MIN_US;
static uint16_t last_throttle_us = MT_PWM_MIN_US;

static uint16_t tick(uint8_t req, uint16_t stick, int link_ok, int armed)
{
    float target;

    VerticalControl_Update(req, stick, link_ok != 0, armed != 0, 0.0f, 0.0f, DT);

    if (!armed)
    {
        slew_us = (float)MT_PWM_MIN_US;
        last_throttle_us = MT_PWM_MIN_US;
        return last_throttle_us;
    }

    target = (float)throttle_target_us(stick);
    slew_us = ramp_limit(slew_us, target, DT);
    last_throttle_us = (uint16_t)(slew_us + 0.5f);
    return last_throttle_us;
}

/* 与 tick 相同，但允许指定姿态角（用于验门禁②的姿态判据） */
static uint16_t tick_att(uint8_t req, uint16_t stick, int link_ok, int armed, float roll_deg)
{
    float target;
    VerticalControl_Update(req, stick, link_ok != 0, armed != 0, roll_deg, 0.0f, DT);
    if (!armed)
    {
        slew_us = (float)MT_PWM_MIN_US;
        last_throttle_us = MT_PWM_MIN_US;
        return last_throttle_us;
    }
    target = (float)throttle_target_us(stick);
    slew_us = ramp_limit(slew_us, target, DT);
    last_throttle_us = (uint16_t)(slew_us + 0.5f);
    return last_throttle_us;
}

static uint16_t run(int n, uint8_t req, uint16_t stick, int link_ok, int armed)
{
    int i;
    uint16_t v = last_throttle_us;
    for (i = 0; i < n; i++) v = tick(req, stick, link_ok, armed);
    return v;
}

static VerticalControlSnapshot_t snap(void)
{
    VerticalControlSnapshot_t s;
    VerticalControl_GetSnapshot(&s);
    return s;
}

static void expect(const char *what, long got, long want)
{
    if (got == want) printf("  [OK]   %-52s = %ld\n", what, got);
    else { printf("  [FAIL] %-52s = %ld  期望 %ld\n", what, got, want); fails++; }
}

static void expect_between(const char *what, long got, long lo, long hi)
{
    if ((got >= lo) && (got <= hi)) printf("  [OK]   %-52s = %ld  (在 %ld~%ld)\n", what, got, lo, hi);
    else { printf("  [FAIL] %-52s = %ld  期望 %ld~%ld\n", what, got, lo, hi); fails++; }
}

static void title(const char *t) { printf("\n== %s ==\n", t); }

static void reset_all(float h)
{
    VerticalControl_Init();
    fake_est.height_cm = h;
    fake_est.vz_cms = 0.0f;
    fake_est.valid = true;
    fake_est.sample_count = 1;
    slew_us = (float)MT_PWM_MIN_US;
    last_throttle_us = MT_PWM_MIN_US;
    stub_vz_kp = 20.0f;
    stub_vz_ki = 0.0f;
    stub_vz_kd = 0.0f;
    stub_max_throttle_us = 1500U;
}

/* 上锁 + 杆最低进定高，再解锁（杆仍最低）→ 停在"停手" */
static void enter_alt_hold_on_ground(float h)
{
    reset_all(h);
    tick(1, 0, 1, 0);
    run(250, 1, 0, 1, 1);
}

/* 从停手推进到"控制启用"（st=1）：推满杆等 h_sp 涨过 4cm */
static void enter_active(void)
{
    run(400, 1, 1000, 1, 1);   /* 1.6s > 1.33s */
}

int main(void)
{
    VerticalControlSnapshot_t s;
    long corr_milli;

    /* ══════════════════════════════════════════════════════════════
       0 回归：普通模式一个字节都没变
       ══════════════════════════════════════════════════════════════ */
    title("0 回归：普通模式油门映射不变（与 H8 完全一致）");
    reset_all(0.8f);
    expect("杆 0", run(400, 0, 0, 1, 1), 1000);
    expect("杆 250", run(400, 0, 250, 1, 1), 1125);
    expect("杆 500", run(400, 0, 500, 1, 1), 1250);
    expect("杆 690", run(400, 0, 690, 1, 1), 1345);
    expect("杆 1000", run(400, 0, 1000, 1, 1), 1500);
    expect("普通模式下 corr 恒 0", (long)(snap().vz_correction_us * 1000.0f), 0);
    expect("普通模式下兜底直接问 GetThrottleUs 也给 1000", VerticalControl_GetThrottleUs(), 1000);

    /* ══════════════════════════════════════════════════════════════
       1 停手 / 上锁：内环必须闭嘴，且积分复位
       ══════════════════════════════════════════════════════════════ */
    title("1 停手（地面）与上锁：corr 恒 0、thr 恒 1000");
    enter_alt_hold_on_ground(0.8f);
    s = snap();
    expect("st = 4（停手）", s.state, VERTICAL_STATE_GROUNDED);
    expect("thr", last_throttle_us, 1000);
    expect("corr", (long)(s.vz_correction_us * 1000.0f), 0);
    /* 就算此刻垂速很大（有人晃飞机），停手也不许动 */
    fake_est.vz_cms = 60.0f;
    run(50, 1, 0, 1, 1);
    s = snap();
    expect("停手中 vz=+60cm/s 时 corr 仍为 0", (long)(s.vz_correction_us * 1000.0f), 0);
    expect("停手中 thr 仍为 1000", last_throttle_us, 1000);
    fake_est.vz_cms = 0.0f;
    tick(1, 0, 1, 0);   /* 上锁 */
    s = snap();
    expect("上锁后 corr = 0", (long)(s.vz_correction_us * 1000.0f), 0);

    /* ══════════════════════════════════════════════════════════════
       2 ⭐⭐ 符号验证：本环节的核心
       ══════════════════════════════════════════════════════════════ */
    title("2 **符号：vz 与 corr 必须反号**（写反 = 正反馈 = 一直往上飞）");
    enter_alt_hold_on_ground(0.8f);
    enter_active();
    s = snap();
    expect("控制启用：st = 1", s.state, VERTICAL_STATE_ALT_HOLD);
    expect("门禁② 全通", s.control_gates, VERTICAL_GATE_CT_ALL);

    /* vz = 0 → corr = 0，thr 正好是基准 1250 */
    fake_est.vz_cms = 0.0f;
    run(60, 1, 1000, 1, 1);
    expect("vz=0 时 corr", (long)(snap().vz_correction_us * 1000.0f), 0);
    expect("vz=0 时 thr = 基准 1250", last_throttle_us, 1250);

    /* ⭐ 向上抬（vz > 0）→ 必须**减**油门（corr < 0） */
    fake_est.vz_cms = 50.0f;
    tick(1, 1000, 1, 1);
    corr_milli = (long)(snap().vz_correction_us * 1000.0f);
    printf("  (vz=+50cm/s → corr = %ld/1000 µs)\n", corr_milli);
    expect("⭐ 向上抬时 corr 为负（减油门）", (corr_milli < 0) ? 1 : 0, 1);
    expect_between("⭐ 向上抬时 corr 顶到 -30", corr_milli, -30000, -29000);
    run(60, 1, 1000, 1, 1);
    expect("⭐ 向上抬稳定后 thr = 1220（基准 − 30）", last_throttle_us, 1220);

    /* ⭐ 向下压（vz < 0）→ 必须**加**油门（corr > 0） */
    fake_est.vz_cms = -50.0f;
    tick(1, 1000, 1, 1);
    corr_milli = (long)(snap().vz_correction_us * 1000.0f);
    printf("  (vz=-50cm/s → corr = %ld/1000 µs)\n", corr_milli);
    expect("⭐ 向下压时 corr 为正（加油门）", (corr_milli > 0) ? 1 : 0, 1);
    expect_between("⭐ 向下压时 corr 顶到 +30", corr_milli, 29000, 30000);
    run(60, 1, 1000, 1, 1);
    expect("⭐ 向下压稳定后 thr = 1280（基准 + 30）", last_throttle_us, 1280);

    /* 小信号：不顶限幅，且严格按 Kp 线性 */
    fake_est.vz_cms = 0.5f;   /* Kp=20 → corr = -10µs */
    tick(1, 1000, 1, 1);
    expect_between("vz=+0.5cm/s（Kp=20）→ corr 约 -10µs", (long)(snap().vz_correction_us * 1000.0f), -10500, -9500);

    /* ══════════════════════════════════════════════════════════════
       3 限幅不会越界
       ══════════════════════════════════════════════════════════════ */
    title("3 限幅：任意垂速下 thr 都锁在 1220~1280");
    {
        int i;
        int over = 0, under = 0;
        uint16_t v;
        for (i = 0; i < 500; i++)
        {
            fake_est.vz_cms = (float)(((i % 40) - 20) * 25);  /* ±500 来回扫 */
            v = tick(1, 1000, 1, 1);
            if (v > 1280U) over++;
            if (v < 1220U) under++;
        }
        expect("thr > 1280 的拍数", over, 0);
        expect("thr < 1220 的拍数", under, 0);
        expect("扫描结束时 thr 仍在范围内", (last_throttle_us >= 1220U && last_throttle_us <= 1280U) ? 1 : 0, 1);
    }

    /* ══════════════════════════════════════════════════════════════
       4 动态限幅
       ══════════════════════════════════════════════════════════════ */
    title("4 动态限幅：向上余量随满杆油门上限变化");
    /* ⚠️ 不能只看 5 拍：thr 是要经过速率限制器的（下降只有 300µs/s），
       从上一段的 1280 降到 1250 要 30/300 = 0.1s = 25 拍。
       这是 H8 仿真踩过的同一个坑：把"限速滞后"误判成"逻辑错"。 */
    fake_est.vz_cms = -50.0f;   /* 一直要往上加 */
    stub_max_throttle_us = 1350U;   /* 1350-1250-100 = 0 → 不许往上加 */
    run(40, 1, 1000, 1, 1);
    corr_milli = (long)(snap().vz_correction_us * 1000.0f);
    expect("满杆 1350 时向上的余量算出来是 0 → corr 不能为正", (corr_milli <= 0) ? 1 : 0, 1);
    expect("此时 thr 不会超过基准（限速器已走完）", (last_throttle_us <= 1250U) ? 1 : 0, 1);

    stub_max_throttle_us = 1500U;
    run(5, 1, 1000, 1, 1);
    corr_milli = (long)(snap().vz_correction_us * 1000.0f);
    expect("满杆改回 1500 后 corr 恢复为正", (corr_milli > 0) ? 1 : 0, 1);

    /* 满杆上限极端低（1200）→ 向上限幅永远是 0，向下仍受 1100 边界与硬限幅约束 */
    stub_max_throttle_us = 1200U;
    run(5, 1, 1000, 1, 1);
    expect("满杆 1200 时 corr 也不能为正", (snap().vz_correction_us <= 0.0f) ? 1 : 0, 1);
    fake_est.vz_cms = 50.0f;
    run(5, 1, 1000, 1, 1);
    expect("满杆 1200 时向下仍顶到 -30（下限只由 1100 边界与硬限幅决定）",
           (long)(snap().vz_correction_us * 1000.0f), -30000);
    expect("⚠️ 此时 thr 的下限 1220 仍远高于混控下限 1100", (last_throttle_us >= 1220U) ? 1 : 0, 1);
    stub_max_throttle_us = 1500U;

    /* ══════════════════════════════════════════════════════════════
       5 门禁失效 → 内环闭嘴（这既是 H9 的验收，也是待办 5 的已知缺口）
       ══════════════════════════════════════════════════════════════ */
    title("5 门禁②失效时 corr 立即归零（含待办 5 那个已知缺口的行为）");
    enter_alt_hold_on_ground(100.0f);   /* 已经在空中 */
    tick(1, 0, 1, 0);
    enter_active();
    fake_est.vz_cms = -50.0f;
    run(30, 1, 1000, 1, 1);
    expect("正常飞行中 corr 为正", (snap().vz_correction_us > 0.0f) ? 1 : 0, 1);

    fake_est.valid = false;
    tick(1, 1000, 1, 1);
    s = snap();
    expect("读数失效那一拍：corr = 0", (long)(s.vz_correction_us * 1000.0f), 0);
    expect("读数失效那一拍：st = 5（IDLE）", s.state, VERTICAL_STATE_IDLE);
    /* ⚠️ 这就是待办 5：空中的含义是"油门压到 1000"，也就是空中停车。H12 之前不许飞。 */
    /* ⚠️ 从 ~1280 降回 1000 要走 280µs，下降限速 300µs/s ⇒ 约 234 拍（0.93s）。
       这里必须等够，否则测到的是限速器的中间值，不是"最终会不会降到 1000"。 */
    run(300, 1, 1000, 1, 1);
    expect("⚠️ 已知缺口：空中读数失效 → thr 降到 1000（= 空中停车）", last_throttle_us, 1000);
    fake_est.valid = true;

    /* 大倾角 */
    reset_all(100.0f);
    tick(1, 0, 1, 0);
    enter_active();
    fake_est.vz_cms = -50.0f;
    run(30, 1, 1000, 1, 1);
    tick_att(1, 1000, 1, 1, 40.0f);   /* 歪 40 度 */
    s = snap();
    expect("歪 40 度：corr = 0", (long)(s.vz_correction_us * 1000.0f), 0);
    expect("歪 40 度：st = 5", s.state, VERTICAL_STATE_IDLE);

    /* ══════════════════════════════════════════════════════════════
       6 停手往返：内环状态干净
       ══════════════════════════════════════════════════════════════ */
    title("6 停手往返：corr 进停手即为 0，重新启用后重新起算");
    enter_alt_hold_on_ground(0.8f);
    enter_active();
    fake_est.vz_cms = -50.0f;
    run(30, 1, 1000, 1, 1);
    expect("启用中 corr 非 0", (snap().vz_correction_us != 0.0f) ? 1 : 0, 1);
    run(400, 1, 0, 1, 1);           /* 杆拉到底，回停手 */
    s = snap();
    expect("回停手：st = 4", s.state, VERTICAL_STATE_GROUNDED);
    expect("回停手：corr = 0", (long)(s.vz_correction_us * 1000.0f), 0);
    expect("回停手：thr = 1000", last_throttle_us, 1000);

    /* ══════════════════════════════════════════════════════════════
       7 参数在线变更
       ══════════════════════════════════════════════════════════════ */
    title("7 参数在线变更生效，且变更会清零积分");
    enter_alt_hold_on_ground(0.8f);
    enter_active();
    fake_est.vz_cms = 1.0f;
    tick(1, 1000, 1, 1);
    expect("Kp=20 时 vz=+1 → corr = -20µs", (long)(snap().vz_correction_us * 1000.0f), -20000);
    stub_vz_kp = 5.0f;
    tick(1, 1000, 1, 1);
    expect("Kp 改成 5 → corr = -5µs（证明参数是活的）", (long)(snap().vz_correction_us * 1000.0f), -5000);
    stub_vz_kp = 0.0f;
    tick(1, 1000, 1, 1);
    expect("Kp=0（参数丢失）→ corr = 0，飞机不会乱动", (long)(snap().vz_correction_us * 1000.0f), 0);
    stub_vz_kp = 20.0f;

    /* Ki ≠ 0：积分累积与积分限幅（H10 才会用到，这里先证明代码支持得住） */
    stub_vz_ki = 0.5f;
    fake_est.vz_cms = 10.0f;    /* 持续误差 -10 */
    run(400, 1, 1000, 1, 1);    /* 1.6s：积分 = -10*0.5*1.6 = -8µs，P 项 -200 → 早就顶了 -30 */
    corr_milli = (long)(snap().vz_correction_us * 1000.0f);
    expect("Ki≠0 时输出仍被限幅在 -30000 以上", (corr_milli >= -30000) ? 1 : 0, 1);
    /* 积分限幅 = out_max(30) * 0.25 = 7.5；反号之后 P 项为正，看总输出是否被积分拖住 */
    fake_est.vz_cms = -10.0f;
    run(400, 1, 1000, 1, 1);
    corr_milli = (long)(snap().vz_correction_us * 1000.0f);
    expect("Ki≠0 时输出仍被限幅在 +30000 以内", (corr_milli <= 30000) ? 1 : 0, 1);
    stub_vz_ki = 0.0f;

    /* ══════════════════════════════════════════════════════════════
       8 不变式：定高模式下 thr 永远落在 [1000, 1280]
       ══════════════════════════════════════════════════════════════ */
    title("8 不变式：定高模式下 thr 永不超过 1280、永不低于 1000");
    {
        uint16_t sticks[] = {0U, 100U, 350U, 500U, 650U, 690U, 800U, 1000U};
        float heights[] = {0.8f, 4.0f, 10.0f, 50.0f, 100.0f, 119.0f};
        float vzs[] = {-80.0f, -20.0f, 0.0f, 20.0f, 80.0f};
        int a, b, c;
        int hard = 0;         /* 真正的越界：>1280 或 <1000 */
        int transient = 0;    /* (1000,1220) 之间的**限速器过渡值**，不是越界 */
        int settled_bad = 0;  /* 每个组合最后一拍必须落到 {1000} ∪ [1220,1280] */
        int combos = 0;
        int sample_n = 0;
        uint16_t vmin = 65535U, vmax = 0U;
        uint16_t sample_transient[5];

        for (a = 0; a < (int)(sizeof(heights) / sizeof(heights[0])); a++)
        {
            for (b = 0; b < (int)(sizeof(sticks) / sizeof(sticks[0])); b++)
            {
                for (c = 0; c < (int)(sizeof(vzs) / sizeof(vzs[0])); c++)
                {
                    int i;
                    uint16_t v = 1000U;
                    combos++;
                    enter_alt_hold_on_ground(heights[a]);
                    run(400, 1, sticks[b], 1, 1);
                    for (i = 0; i < 60; i++)
                    {
                        fake_est.vz_cms = vzs[c];
                        v = tick(1, sticks[b], 1, 1);
                        if (v < vmin) vmin = v;
                        if (v > vmax) vmax = v;
                        if ((v > 1280U) || (v < 1000U))
                        {
                            hard++;
                        }
                        else if ((v != 1000U) && (v < 1220U))
                        {
                            transient++;
                            if (sample_n < 5) sample_transient[sample_n++] = v;
                        }
                    }
                    if ((v != 1000U) && ((v < 1220U) || (v > 1280U))) settled_bad++;
                }
            }
        }
        printf("  (遍历 %d 种组合；thr 范围 %u ~ %u)\n", combos, (unsigned)vmin, (unsigned)vmax);
        if (transient > 0)
        {
            printf("  (过渡值 %d 个，例：", transient);
            for (a = 0; a < sample_n; a++) printf("%u ", (unsigned)sample_transient[a]);
            printf("—— 这是限速器在 1000↔1220 之间的中间值，不是越界)\n");
        }
        expect("真正的越界拍数（>1280 或 <1000）", hard, 0);
        expect("每个组合**稳定后**仍越界的组合数", settled_bad, 0);
        expect("全局最小值 = 1000（停手）", vmin, 1000);
        expect("全局最大值 = 1280（基准 + 限幅）", vmax, 1280);
    }

    /* ══════════════════════════════════════════════════════════════
       9 回归：H6/H8 那几条不许被 H9 破坏
       ══════════════════════════════════════════════════════════════ */
    title("9 回归：停手不许抖 / 上锁立即 1000 / 模式切换");
    enter_alt_hold_on_ground(4.0f);
    {
        int i, flips = 0, prev;
        VerticalControl_GetSnapshot(&s);
        prev = s.grounded ? 1 : 0;
        for (i = 0; i < 2500; i++)
        {
            int now;
            tick(1, 0, 1, 1);
            VerticalControl_GetSnapshot(&s);
            now = s.grounded ? 1 : 0;
            if (now != prev) flips++;
            prev = now;
        }
        expect("杆最低 10 秒内停手翻转次数", flips, 0);
    }
    tick(1, 0, 1, 0);
    expect("上锁那一拍 thr = 1000", last_throttle_us, 1000);

    reset_all(0.8f);
    run(600, 0, 1000, 1, 1);
    expect("普通模式杆 1000 → 1500（回归）", last_throttle_us, 1500);
    tick(1, 0, 1, 0);
    expect("上锁 → 1000", last_throttle_us, 1000);
    tick(1, 0, 1, 1);
    expect("解锁进定高停手 → 1000", last_throttle_us, 1000);
    enter_active();
    fake_est.vz_cms = 0.0f;
    run(60, 1, 1000, 1, 1);
    expect("推进到控制启用 → 1250", last_throttle_us, 1250);

    printf("\n===== %s（失败 %d 项）=====\n", fails == 0 ? "全部通过" : "存在失败", fails);
    return fails == 0 ? 0 : 1;
}
