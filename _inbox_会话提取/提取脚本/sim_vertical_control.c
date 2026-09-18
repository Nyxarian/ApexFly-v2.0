/* 在 PC 上跑 vertical_control 的状态机，把 H6 的验收矩阵在烧录之前先跑一遍。
   编译（宿主机 gcc 即可，本文件不碰任何硬件）：
     gcc -std=c99 -I "ApexFly v2.0/Modules/Control" sim_vertical_control.c "ApexFly v2.0/Modules/Control/vertical_control.c" -o sim_vc.exe
   用法：sim_vc.exe */
#include "vertical_control.h"
#include "vertical_est.h"
#include <stdio.h>

/* ── 假的估计器：替换掉真的 vertical_est，测试里直接改 fake_est 就能模拟高度 ── */
static VerticalEstState_t fake_est;

void VerticalEst_GetState(VerticalEstState_t *state) { *state = fake_est; }
void VerticalEst_Update(float roll_deg, float pitch_deg, float dt_s) { (void)roll_deg; (void)pitch_deg; (void)dt_s; }
void VerticalEst_Init(void) {}
void VerticalEst_Reset(void) {}

static const float DT = 0.004f; /* 250Hz */
static int fails = 0;

static void step(int n, uint8_t req, uint16_t stick, int link_ok, int armed)
{
    int i;
    for (i = 0; i < n; i++) VerticalControl_Update(req, stick, link_ok != 0, armed != 0, 0.0f, 0.0f, DT);
}

static VerticalControlSnapshot_t snap(void)
{
    VerticalControlSnapshot_t s;
    VerticalControl_GetSnapshot(&s);
    return s;
}

static void expect(const char *what, long got, long want)
{
    if (got == want) printf("  [OK]   %-44s = %ld\n", what, got);
    else { printf("  [FAIL] %-44s = %ld  期望 %ld\n", what, got, want); fails++; }
}

static void title(const char *t) { printf("\n== %s ==\n", t); }

/* 浮点积分会有小漂移（dt=0.004 累加），需要跨场景对齐的值用带容差的比较。 */
static void expect_near(const char *what, long got, long want, long tol)
{
    if ((got >= (want - tol)) && (got <= (want + tol))) printf("  [OK]   %-44s = %ld (~%ld)\n", what, got, want);
    else { printf("  [FAIL] %-44s = %ld  期望 %ld±%ld\n", what, got, want, tol); fails++; }
}

int main(void)
{
    VerticalControlSnapshot_t s;
    long h_sp_after_climb;
    int flips;
    int prev_grounded;
    int i;

    VerticalControl_Init();
    fake_est.height_cm = 0.8f; /* 飞机放在平面上，镜头安装偏置约 8mm */
    fake_est.vz_cms = 0.0f;
    fake_est.valid = true;
    fake_est.sample_count = 1;

    title("1 失联时不得切模式（armed 也是 false，不能当成'在地面'）");
    step(250, 1, 0, 0, 0);
    s = snap();
    expect("mode", s.mode, 0);
    expect("门禁① 缺 ON_GROUND 位", (s.switch_gates & VERTICAL_GATE_SW_ON_GROUND) ? 1 : 0, 0);

    title("2 链路正常 + 上锁 + 杆最低 -> 立刻进定高并停手");
    step(1, 1, 0, 1, 0);
    s = snap();
    expect("mode", s.mode, 1);
    expect("门禁① 全通", s.switch_gates, VERTICAL_GATE_SW_ALL);
    expect("停手（进入时直接置满计时，无 500ms 窗口）", s.grounded ? 1 : 0, 1);
    expect("state = 4（停手）", s.state, VERTICAL_STATE_GROUNDED);
    expect("h_sp 归零（不对齐到地面读数）", (long)(s.h_sp_cm * 10.0f + 0.5f), 0);

    title("3 杆推离最低位后门禁① 不再通过，但已进的模式不会被踢出来");
    step(250, 1, 300, 1, 0);
    s = snap();
    expect("mode 仍为 1", s.mode, 1);
    expect("门禁① 掉 STICK_LOW", (s.switch_gates & VERTICAL_GATE_SW_STICK_LOW) ? 1 : 0, 0);

    title("4 解锁后仍是停手（飞机确实在地面上）");
    step(250, 1, 0, 1, 1);
    s = snap();
    expect("grounded", s.grounded ? 1 : 0, 1);
    expect("state", s.state, VERTICAL_STATE_GROUNDED);

    title("5 推杆满上升 -> h_sp 涨过 4cm -> 退出停手，控制器接上");
    /* ⚠️ 起点是 0 不是 0.8：停手期间杆一直停在下降段，h_sp 早就被钳到下限了。
       所以从 0 涨到 4cm 要 4/3 = 1.33s（这就是留档里那个"推杆后约 1.3s 才动"）。 */
    step(400, 1, 1000, 1, 1); /* 400 拍 = 1.6s */
    s = snap();
    expect("grounded 已退出", s.grounded ? 1 : 0, 0);
    expect("state = 1（定高在干活）", s.state, VERTICAL_STATE_ALT_HOLD);
    expect("门禁② 全通", s.control_gates, VERTICAL_GATE_CT_ALL);
    printf("  (h_sp = %.2f cm)\n", (double)s.h_sp_cm);

    title("6 杆回中段 -> h_sp 冻结（松手就稳住）");
    h_sp_after_climb = (long)(s.h_sp_cm * 10.0f + 0.5f);
    step(500, 1, 500, 1, 1);
    s = snap();
    expect("rate = 0", (long)(s.stick_rate_cms * 100.0f), 0);
    expect("h_sp 与 5 结束时的值一致", (long)(s.h_sp_cm * 10.0f + 0.5f), h_sp_after_climb);
    printf("  (h_sp = %.2f cm)\n", (double)s.h_sp_cm);

    title("7 杆拉到底 -> h_sp 掉回 <=4cm，且 h<=2cm 持续 500ms -> 重新停手");
    step(200, 1, 0, 1, 1); /* rate=-2，从 4.49 降到 4 约 25 拍，再 500ms 去抖 */
    s = snap();
    expect("grounded 重新进入", s.grounded ? 1 : 0, 1);
    expect("state 回到 4", s.state, VERTICAL_STATE_GROUNDED);

    title("8 推力偏弱起飞：一直在地面(0.8cm)、杆保持上升 -> 绝不能被重新按回停手");
    step(1500, 1, 1000, 1, 1); /* 6 秒，足够过去抖窗口 */
    s = snap();
    expect("grounded 保持退出", s.grounded ? 1 : 0, 0);
    expect("state 保持 1", s.state, VERTICAL_STATE_ALT_HOLD);
    /* 期望值 = 6 秒 × 3cm/s = 18cm（这也是"推力偏弱时爬升指令没被吞掉"的证据）。
       起点是 0：场景 7 进停手时 h_sp 被归零，杆又一直在下降段，所以它一直是 0。 */
    expect_near("h_sp 一路涨（爬升指令没被吞掉）", (long)(s.h_sp_cm * 10.0f + 0.5f), 180, 5);

    title("9 未上锁时请求切回普通 -> 被门禁拒绝，遥测带 0x80");
    step(250, 0, 1000, 1, 1);
    s = snap();
    expect("mode 仍为 1", s.mode, 1);
    expect("telemetry_mode 带拒绝位", s.telemetry_mode, (long)(0x80U | 1U));

    title("10 上锁 + 杆最低 + 请求普通 -> 切回普通");
    step(250, 0, 0, 1, 0);
    s = snap();
    expect("mode", s.mode, 0);
    expect("state = 0", s.state, VERTICAL_STATE_MANUAL);
    expect("telemetry_mode 无拒绝位", s.telemetry_mode, 0);

    title("11 未知请求值（2/3）只拒绝，不猜");
    step(250, 2, 0, 1, 0);
    s = snap();
    expect("mode 仍为 0", s.mode, 0);

    title("12 大倾角 -> 门禁② 掉 ATT_OK，状态退成 5（已进模式但未使能）");
    step(1, 1, 0, 1, 0);   /* 先切进定高 */
    step(1, 1, 1000, 1, 1);
    /* 400 拍 = 1.6s：h_sp 起点是 0（停手归零），涨过 4cm 要 1.33s。 */
    step(400, 1, 1000, 1, 1); /* 先正常退出停手 */
    VerticalControl_Update(1, 1000, 1, 1, 40.0f, 0.0f, DT); /* 歪 40 度 */
    s = snap();
    expect("门禁② 掉 ATT_OK", (s.control_gates & VERTICAL_GATE_CT_ATT_OK) ? 1 : 0, 0);
    expect("state = 5（IDLE）", s.state, VERTICAL_STATE_IDLE);

    title("13 读数失效 -> 门禁① 掉 EST_VALID，且不再进停手");
    fake_est.valid = false;
    step(300, 0, 0, 1, 0);
    s = snap();
    expect("门禁① 掉 EST_VALID", (s.switch_gates & VERTICAL_GATE_SW_EST_VALID) ? 1 : 0, 0);
    expect("mode 保持 1，不得被切走", s.mode, 1);
    fake_est.valid = true;

    title("14 高度超过 120cm -> 门禁① 掉 HEIGHT_OK");
    fake_est.height_cm = 130.0f;
    step(300, 0, 0, 1, 0);
    s = snap();
    expect("门禁① 掉 HEIGHT_OK", (s.switch_gates & VERTICAL_GATE_SW_HEIGHT_OK) ? 1 : 0, 0);
    expect("mode 仍为 1", s.mode, 1);

    title("15 h_sp 钳位：长时间满杆上升不超过 120cm，长时间下拉不低于 0");
    fake_est.height_cm = 0.8f;
    step(60000, 1, 1000, 1, 1); /* 240 秒 */
    s = snap();
    expect("h_sp 上限", (long)(s.h_sp_cm * 10.0f + 0.5f), 1200);
    step(120000, 1, 0, 1, 1);   /* 480 秒 */
    s = snap();
    expect("h_sp 下限", (long)(s.h_sp_cm * 10.0f + 0.5f), 0);

    title("16 正常起飞 -> 悬停 -> 落地：落地必须能重新进停手（保护已被离地清掉）");
    VerticalControl_Init();
    fake_est.height_cm = 0.8f;
    fake_est.valid = true;
    step(10, 1, 0, 1, 0);      /* 上锁 + 杆最低 -> 进定高 + 停手 */
    step(1, 1, 1000, 1, 1);    /* 解锁并推杆 */
    fake_est.height_cm = 100.0f;   /* 真的飞起来了 */
    step(500, 1, 1000, 1, 1);      /* 2 秒，h_est > 10cm -> 起飞保护清除 */
    step(100, 1, 500, 1, 1);       /* 杆回中段悬停 */
    s = snap();
    expect("悬停中：不在停手", s.grounded ? 1 : 0, 0);
    expect("悬停中：state = 1", s.state, VERTICAL_STATE_ALT_HOLD);
    fake_est.height_cm = 0.8f;     /* 落地 */
    step(250, 1, 500, 1, 1);       /* 杆保持中位，1 秒 */
    s = snap();
    expect("落地后重新进停手", s.grounded ? 1 : 0, 1);
    expect("落地后 state = 4", s.state, VERTICAL_STATE_GROUNDED);
    expect("h_sp 归零", (long)(s.h_sp_cm * 10.0f + 0.5f), 0);

    title("17 垫高台架（地面读数 4cm、门槛 6cm）：停手必须进得去，而且不许抖");
    /* 复刻 2026-09-14 的实测现场：飞机被垫高，地面读数从约 1cm 变成约 4cm，
       当时门槛还是 2.0，结果停手一次都没进去（st 一直 1、gt 一直 0ms）。 */
    VerticalControl_Init();
    fake_est.height_cm = 4.0f;
    fake_est.valid = true;
    step(10, 1, 0, 1, 0); /* 上锁 + 杆最低 */
    s = snap();
    expect("mode", s.mode, 1);
    expect("停手进得去（4cm ≤ 6cm）", s.grounded ? 1 : 0, 1);
    expect("state = 4", s.state, VERTICAL_STATE_GROUNDED);
    expect("h_sp 归零", (long)(s.h_sp_cm * 10.0f + 0.5f), 0);

    /* 保持 10 秒，中途状态一次都不许翻转（这是"进入即退出"抖动的回归测试）。 */
    prev_grounded = 1;
    flips = 0;
    for (i = 0; i < 2500; i++)
    {
        VerticalControl_Update(1, 0, true, true, 0.0f, 0.0f, DT);
        s = snap();
        if ((s.grounded ? 1 : 0) != prev_grounded) flips++;
        prev_grounded = s.grounded ? 1 : 0;
    }
    expect("10 秒内停手被翻出来的次数", flips, 0);

    /* 地面读数在实测的 3.5~4.7cm 区间里来回抖，也不许跳出去。 */
    flips = 0;
    for (i = 0; i < 2500; i++)
    {
        fake_est.height_cm = ((i % 2) == 0) ? 3.5f : 4.7f;
        VerticalControl_Update(1, 0, true, true, 0.0f, 0.0f, DT);
        s = snap();
        if ((s.grounded ? 1 : 0) != prev_grounded) flips++;
        prev_grounded = s.grounded ? 1 : 0;
    }
    expect("读数在 3.5~4.7cm 之间抖时的翻转次数", flips, 0);

    /* 推杆起飞 -> 退出停手；收杆到下降段 -> 重新进停手。 */
    fake_est.height_cm = 4.0f;
    step(400, 1, 1000, 1, 1);
    s = snap();
    expect("推杆后退出停手", s.grounded ? 1 : 0, 0);
    expect("state = 1", s.state, VERTICAL_STATE_ALT_HOLD);
    step(400, 1, 0, 1, 1);
    s = snap();
    expect("收杆后重新进停手", s.grounded ? 1 : 0, 1);

    printf("\n===== %s（失败 %d 项）=====\n", fails == 0 ? "全部通过" : "存在失败", fails);
    return fails == 0 ? 0 : 1;
}
