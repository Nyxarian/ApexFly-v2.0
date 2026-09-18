/* H8（油门来源接管）的台架仿真：把验收矩阵在**烧录之前**先跑一遍。
   只碰 vertical_control 的状态机与新加的 VerticalControl_GetThrottleUs()，
   再加一个与 motor_test.c **逐行对应**的油门来源分叉 + 速率限制器 + 上锁分支。

   编译（宿主机 gcc；本文件不碰任何硬件）：
     gcc -std=c99 -Wall -Wextra -I sim_stub -I "ApexFly v2.0/Modules/Control" `
         -I "ApexFly v2.0/Modules/Motor" sim_vc_h8.c `
         "ApexFly v2.0/Modules/Control/vertical_control.c" -o sim_vc_h8.exe
   说明：motor.h 会 include "main.h"（HAL 的聚合头），宿主机上不需要它的任何内容，
        所以用 sim_stub/main.h 顶掉。sim_stub 绝不能加进 Keil 的包含路径。

   ⚠️ 仿真**不能**替代真机验证：它证明的是"门禁 → 状态 → 油门取值"这条逻辑链正确，
      证明不了电机真的不转。真机验收步骤见
      `优化一下/高度接入施工顺序.md` 的「H8 验收步骤」。

   本脚本第一版跑出 10 个失败，逐条查下来**全部是脚本自己的模型错**（漏了 motor_test.c 的
   `if (!armed)` 分支）或**我自己写的期望值错**（把"停手退出需要 h_sp 掉回 4cm 以下"记错了，
   实际退出只需要 500ms 去抖；把"限速器会立即到位"记错了，实际要 0.13s/0.33s）。
   产品代码在这 10 条上**没有**发现问题。两个被纠正的期望值已经写回
   `优化一下/高度接入施工顺序.md` 的验收表（那里原来的写法会让真机测试误判成失败）。
*/
#include "vertical_control.h"
#include "vertical_est.h"
#include <stdio.h>

/* ── 假的估计器：替换掉真的 vertical_est ── */
static VerticalEstState_t fake_est;

void VerticalEst_GetState(VerticalEstState_t *state) { *state = fake_est; }
void VerticalEst_Update(float roll_deg, float pitch_deg, float dt_s) { (void)roll_deg; (void)pitch_deg; (void)dt_s; }
void VerticalEst_Init(void) {}
void VerticalEst_Reset(void) {}

/* ── 从 motor_test.c 抄过来的常量（改那边就必须改这里）── */
#define MT_PWM_MIN_US           1000U
#define MT_PWM_MAX_MAP_US       1500U  /* PidTable 里 altitude 组的 max_throttle 默认值 */
#define MT_SLEW_UP_US_PER_S     800.0f
#define MT_SLEW_DOWN_US_PER_S   300.0f
#define MT_ATTITUDE_ENABLE_US   1100U  /* = MIXER_PWM_EFFECTIVE_MIN_US，判据含等号 */
#define MT_STICK_MAX            1000U

static const float DT = 0.004f; /* 250Hz */
static int fails = 0;

/* ── 与 motor_test.c 的 MotorTest_MapStickToThrottleUs 等价（max 固定 1500）── */
static uint16_t manual_map(uint16_t stick)
{
    uint32_t span = (uint32_t)MT_PWM_MAX_MAP_US - (uint32_t)MT_PWM_MIN_US;
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

/* ── 复刻 motor_test.c 里改完之后的那个分叉 ── */
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
        /* motor_test.c 的 if (!armed) 分支：直接给最小值，并复位限速器。 */
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

/* 跑 n 拍，返回"超过 limit 的拍数"，并把峰值写到 *peak。
   同时把"第一拍达到 mark 的拍号"写到 *mark_at（没达到写 -1）。 */
static int run_watch(int n, uint8_t req, uint16_t stick, int link_ok, int armed,
                     uint16_t limit, uint16_t mark, uint16_t *peak, int *mark_at)
{
    int i;
    int over = 0;
    uint16_t mx = 0U;
    if (mark_at != NULL) *mark_at = -1;
    for (i = 0; i < n; i++)
    {
        uint16_t v = tick(req, stick, link_ok, armed);
        if (v > mx) mx = v;
        if (v > limit) over++;
        if ((mark_at != NULL) && (*mark_at < 0) && (v >= mark)) *mark_at = i;
    }
    if (peak != NULL) *peak = mx;
    return over;
}

/* 跑 n 拍，返回 grounded 翻转次数 */
static int run_count_flips(int n, uint8_t req, uint16_t stick, int link_ok, int armed)
{
    int i, flips = 0, prev;
    VerticalControlSnapshot_t s;

    VerticalControl_GetSnapshot(&s);
    prev = s.grounded ? 1 : 0;
    for (i = 0; i < n; i++)
    {
        int now;
        tick(req, stick, link_ok, armed);
        VerticalControl_GetSnapshot(&s);
        now = s.grounded ? 1 : 0;
        if (now != prev) flips++;
        prev = now;
    }
    return flips;
}

/* 跑 n 拍，返回"第一次出现指定 state 的拍号"（没出现写 -1） */
static int run_until_state(int n, uint8_t req, uint16_t stick, int link_ok, int armed, uint8_t want)
{
    int i;
    VerticalControlSnapshot_t s;
    for (i = 0; i < n; i++)
    {
        tick(req, stick, link_ok, armed);
        VerticalControl_GetSnapshot(&s);
        if (s.state == want) return i;
    }
    return -1;
}

static VerticalControlSnapshot_t snap(void)
{
    VerticalControlSnapshot_t s;
    VerticalControl_GetSnapshot(&s);
    return s;
}

static void expect(const char *what, long got, long want)
{
    if (got == want) printf("  [OK]   %-48s = %ld\n", what, got);
    else { printf("  [FAIL] %-48s = %ld  期望 %ld\n", what, got, want); fails++; }
}

static void expect_between(const char *what, long got, long lo, long hi)
{
    if ((got >= lo) && (got <= hi)) printf("  [OK]   %-48s = %ld  (在 %ld~%ld)\n", what, got, lo, hi);
    else { printf("  [FAIL] %-48s = %ld  期望 %ld~%ld\n", what, got, lo, hi); fails++; }
}

static void title(const char *t) { printf("\n== %s ==\n", t); }

/* 复位到"上电、飞机放在平面上"的状态 */
static void reset_ground(float h)
{
    VerticalControl_Init();
    fake_est.height_cm = h;
    fake_est.vz_cms = 0.0f;
    fake_est.valid = true;
    fake_est.sample_count = 1;
    slew_us = (float)MT_PWM_MIN_US;
    last_throttle_us = MT_PWM_MIN_US;
}

/* 上锁 + 杆最低 -> 进定高并停手（模拟"地面选模式"） */
static void enter_alt_hold_on_ground(float h)
{
    reset_ground(h);
    tick(1, 0, 1, 0);      /* 请求定高，上锁、杆最低 -> 门禁① 全通 */
    run(250, 1, 0, 1, 1);  /* 解锁，保持杆最低 -> 已在停手 */
}

int main(void)
{
    VerticalControlSnapshot_t s;
    uint16_t peak;
    int mark_at;
    int over;
    long h_sp_before;

    /* ══════════════════════════════════════════════════════════════
       验收 ①：普通模式油门映射必须一个字节都没变
       ══════════════════════════════════════════════════════════════ */
    title("1 普通模式：油门杆 0->1000 就是 1000->1500us（映射与改动前一致）");
    reset_ground(0.8f);
    expect("杆 0", run(400, 0, 0, 1, 1), 1000);
    expect("杆 250", run(400, 0, 250, 1, 1), 1125);
    expect("杆 500", run(400, 0, 500, 1, 1), 1250);
    expect("杆 750", run(400, 0, 750, 1, 1), 1375);
    expect("杆 1000", run(600, 0, 1000, 1, 1), 1500);
    expect("state = 0（MANUAL）", snap().state, VERTICAL_STATE_MANUAL);
    expect("上锁 -> 立即回 1000", run(1, 0, 1000, 1, 0), 1000);
    /* 全程峰值必须正好是 1500：多一个 us 都说明定高那条路被误走。 */
    reset_ground(0.8f);
    over = run_watch(2500, 0, 1000, 1, 1, 1500U, 0U, &peak, NULL);
    expect("普通模式 10 秒内的油门峰值", peak, 1500);
    expect("普通模式超过 1500 的拍数", over, 0);

    /* ══════════════════════════════════════════════════════════════
       验收 ②：地面上、定高模式、杆最低 -> 停手、1000
       ══════════════════════════════════════════════════════════════ */
    title("2 定高 + 上锁 + 杆最低 -> st=4 且 thr=1000 不动");
    reset_ground(0.8f);
    expect("切进定高那拍", tick(1, 0, 1, 0), 1000);
    s = snap();
    expect("mode", s.mode, 1);
    expect("state = 4（停手）", s.state, VERTICAL_STATE_GROUNDED);
    expect("解锁后仍是 1000", run(250, 1, 0, 1, 1), 1000);

    /* ══════════════════════════════════════════════════════════════
       验收 ③：推杆到底 —— 前 1.33s 恒 1000（电机不转），之后才升到 1100
       ══════════════════════════════════════════════════════════════ */
    title("3 **推杆到底：前 1.33s thr 恒 1000（电机不转）**，之后按设计爬到 1100");
    /* 对照：改动前（H6）在定高模式下推同量杆会给多少。 */
    reset_ground(0.8f);
    expect("对照：普通模式杆 690 给多少", run(600, 0, 690, 1, 1), 1345);
    {
        uint16_t v_before_cross = 0U;
        int i;
        over = 0;
        peak = 0U;
        mark_at = -1;
        enter_alt_hold_on_ground(0.8f);
        for (i = 0; i < 600; i++)
        {
            uint16_t v = tick(1, 1000, 1, 1);
            if (v > peak) peak = v;
            if (v > MT_ATTITUDE_ENABLE_US) over++;
            if ((mark_at < 0) && (v >= MT_ATTITUDE_ENABLE_US)) mark_at = i;
            if (i == 332) v_before_cross = v;   /* 退出停手的前一拍 */
        }
        expect("第 333 拍（退出停手前）thr", v_before_cross, 1000);
        expect_between("thr 第一次 >=1100 的拍号（期望约 365）", mark_at, 362, 370);
        expect("全程油门峰值", peak, 1100);
        expect("超过 1100 的拍数", over, 0);
    }
    s = snap();
    expect("st = 1", s.state, VERTICAL_STATE_ALT_HOLD);
    expect("门禁② 全通", s.control_gates, VERTICAL_GATE_CT_ALL);

    title("4 杆 690（刚过保持段）：8 秒内 thr 仍是 1000（油门不跟杆走）");
    enter_alt_hold_on_ground(0.8f);
    /* rate = +3×(690−650)/350 = +0.343 cm/s，h_sp 涨过 4cm 要 11.7 s，
       所以 8 秒（2000 拍）内一定还在停手。 */
    over = run_watch(2000, 1, 690, 1, 1, MT_ATTITUDE_ENABLE_US, 0U, &peak, NULL);
    expect("8 秒内油门峰值", peak, 1000);
    expect("超过 1100 的拍数", over, 0);
    expect("st 仍是 4", snap().state, VERTICAL_STATE_GROUNDED);

    /* ══════════════════════════════════════════════════════════════
       验收 ④⑤⑥：停手 -> 控制 -> 停手 的完整来回
       ══════════════════════════════════════════════════════════════ */
    title("5 推杆到底 -> 第 333 拍退出停手；thr 再用约 32 拍爬到 1100");
    enter_alt_hold_on_ground(0.8f);
    expect_between("退出停手发生在第几拍（250Hz，期望 333）",
                   run_until_state(600, 1, 1000, 1, 1, VERTICAL_STATE_ALT_HOLD), 332, 335);
    {
        int i;
        int reached_at = -1;
        for (i = 0; i < 200; i++)
        {
            uint16_t v = tick(1, 1000, 1, 1);
            if ((reached_at < 0) && (v >= 1100U)) reached_at = i;
        }
        /* 100us / 800us每秒 = 0.125s = 31.25 拍；上一步已经跑了 1 拍左右。 */
        expect_between("thr 从 1000 爬到 1100 用了多少拍（期望约 31）", reached_at, 29, 33);
    }

    title("6 杆回中段 -> h_sp 冻结、thr 停在 1100");
    s = snap();
    h_sp_before = (long)(s.h_sp_cm * 10.0f + 0.5f);
    expect("中段油门", run(500, 1, 500, 1, 1), 1100);
    s = snap();
    expect("rate = 0", (long)(s.stick_rate_cms * 100.0f), 0);
    expect("h_sp 冻结", (long)(s.h_sp_cm * 10.0f + 0.5f), h_sp_before);

    title("7 杆拉到底 -> 约 0.5s（125 拍）后回 st=4；thr 再用约 84 拍降回 1000");
    /* ⚠️ 这里**不需要**等 h_sp 掉回 4cm 以下：停手进入判据用的是 h_est + rate，
       不是 h_sp。（第一版脚本把这条记错了，期望值写成了约 190 拍。） */
    expect_between("重新进停手发生在第几拍（期望 124）",
                   run_until_state(600, 1, 0, 1, 1, VERTICAL_STATE_GROUNDED), 123, 126);
    {
        int i;
        int low_at = -1;
        for (i = 0; i < 300; i++)
        {
            uint16_t v = tick(1, 0, 1, 1);
            if ((low_at < 0) && (v <= 1000U)) low_at = i;
        }
        /* 100us / 300us每秒 = 0.333s = 83.3 拍。 */
        expect_between("thr 从 1100 降回 1000 用了多少拍（期望约 83）", low_at, 81, 86);
    }
    expect("st = 4", snap().state, VERTICAL_STATE_GROUNDED);

    /* ══════════════════════════════════════════════════════════════
       验收 ⑦⑧：上锁 / 举到 1.3m
       ══════════════════════════════════════════════════════════════ */
    title("8 上锁 -> thr 立即 1000（上锁分支直接赋值，不走限速器）");
    run(1, 1, 0, 1, 0);
    expect("上锁那一拍", last_throttle_us, 1000);

    title("9 **举到 1.3m 以上**：v=0、sw 掉两位、st 仍是 4、thr 仍是 1000");
    enter_alt_hold_on_ground(0.8f);
    expect("举高前 st", snap().state, VERTICAL_STATE_GROUNDED);
    fake_est.valid = false;          /* 超量程 -> 估计器判定不可信 */
    fake_est.height_cm = 130.0f;
    over = run_watch(500, 1, 0, 1, 1, 1000U, 0U, &peak, NULL); /* 只统计 >1000 的拍数 */
    expect("举高 2 秒内 thr 超过 1000 的拍数", over, 0);
    expect("举高 2 秒内油门峰值", peak, 1000);
    s = snap();
    expect("sw 掉 EST_VALID", (s.switch_gates & VERTICAL_GATE_SW_EST_VALID) ? 1 : 0, 0);
    expect("sw 掉 HEIGHT_OK", (s.switch_gates & VERTICAL_GATE_SW_HEIGHT_OK) ? 1 : 0, 0);
    expect("st 仍是 4（停手不因高度退出，这是设计）", s.state, VERTICAL_STATE_GROUNDED);
    expect("thr 仍是 1000", last_throttle_us, 1000);
    fake_est.valid = true;

    title("10 飞行中读数失效 -> 目标立刻回 1000，实际油门约 0.33s 跟上");
    reset_ground(100.0f);            /* 已经在空中 */
    tick(1, 0, 1, 0);                /* 切进定高（高度 100 < 120，门禁① 能过） */
    run(400, 1, 1000, 1, 1);
    expect("正常飞行中 thr", last_throttle_us, 1100);
    fake_est.valid = false;
    VerticalControl_Update(1, 1000, 1, 1, 0.0f, 0.0f, DT);
    expect("失效那一拍：目标值立即是 1000", VerticalControl_GetThrottleUs(), 1000);
    expect("失效那一拍：state = 5（IDLE）", snap().state, VERTICAL_STATE_IDLE);
    {
        int i;
        int low_at = -1;
        for (i = 0; i < 300; i++)
        {
            uint16_t v = tick(1, 1000, 1, 1);
            if ((low_at < 0) && (v <= 1000U)) low_at = i;
        }
        expect_between("实际油门降回 1000 用了多少拍（期望约 83）", low_at, 81, 86);
    }
    fake_est.valid = true;

    title("11 大倾角 -> 门禁② 掉 ATT_OK -> 目标同样压回 1000");
    reset_ground(100.0f);
    tick(1, 0, 1, 0);
    run(400, 1, 1000, 1, 1);
    expect("正常飞行中 thr", last_throttle_us, 1100);
    VerticalControl_Update(1, 1000, 1, 1, 40.0f, 0.0f, DT);
    expect("歪 40 度那一拍：目标就是 1000", VerticalControl_GetThrottleUs(), 1000);
    expect("state = 5", snap().state, VERTICAL_STATE_IDLE);

    /* ══════════════════════════════════════════════════════════════
       验收 ⑨⑩：模式来回切
       ══════════════════════════════════════════════════════════════ */
    title("12 切回普通模式 -> 摇杆重新驱动油门");
    enter_alt_hold_on_ground(0.8f);
    expect("定高停手时 1000", last_throttle_us, 1000);
    tick(0, 0, 1, 0);                /* 上锁 + 杆最低 -> 切回普通 */
    s = snap();
    expect("mode 回 0", s.mode, 0);
    expect("state = 0", s.state, VERTICAL_STATE_MANUAL);
    expect("普通模式杆 1000 -> 1500", run(600, 0, 1000, 1, 1), 1500);
    expect("普通模式杆 0 -> 1000", run(600, 0, 0, 1, 1), 1000);

    title("13 模式切换过程中油门连续无台阶");
    reset_ground(0.8f);
    run(600, 0, 1000, 1, 1);         /* 普通模式杆拉满 -> 1500 */
    expect("切换前", last_throttle_us, 1500);
    expect("上锁那一拍就回 1000", tick(1, 0, 1, 0), 1000);
    expect("解锁进定高停手", tick(1, 0, 1, 1), 1000);
    over = run_watch(2500, 1, 1000, 1, 1, MT_ATTITUDE_ENABLE_US, 0U, &peak, NULL);
    expect("之后 10 秒内油门峰值（只能到 1100）", peak, 1100);
    expect("超过 1100 的拍数", over, 0);

    /* ══════════════════════════════════════════════════════════════
       不变式：定高模式下油门只可能 ≤ 1100，永远不会更高
       ══════════════════════════════════════════════════════════════ */
    title("14 不变式：定高模式下任意杆量、任意高度的油门都 ≤ 1100");
    {
        uint16_t max_seen = 0U;
        int worst_over = 0;
        uint16_t sticks[] = {0U, 100U, 350U, 500U, 650U, 690U, 800U, 1000U};
        float heights[] = {0.8f, 4.0f, 10.0f, 50.0f, 100.0f, 119.0f};
        int a, b;
        for (a = 0; a < (int)(sizeof(heights) / sizeof(heights[0])); a++)
        {
            for (b = 0; b < (int)(sizeof(sticks) / sizeof(sticks[0])); b++)
            {
                enter_alt_hold_on_ground(heights[a]);
                over = run_watch(2500, 1, sticks[b], 1, 1, 1100U, 0U, &peak, NULL);
                if (peak > max_seen) max_seen = peak;
                worst_over += over;
            }
        }
        expect("48 种组合遍历中的全局峰值", max_seen, 1100);
        expect("超过 1100 的总拍数", worst_over, 0);
    }

    title("15 回归：H6 那条'停手不许抖'仍然成立");
    enter_alt_hold_on_ground(4.0f);  /* 垫高台架的地面读数 */
    expect("杆最低 10 秒内停手翻转次数", run_count_flips(2500, 1, 0, 1, 1), 0);
    expect("杆中段 10 秒内停手翻转次数", run_count_flips(2500, 1, 500, 1, 1), 0);

    printf("\n===== %s（失败 %d 项）=====\n", fails == 0 ? "全部通过" : "存在失败", fails);
    return fails == 0 ? 0 : 1;
}
