/* H14（停手改成"推杆即解手"）的台架仿真：把验收矩阵在**烧录之前**先跑一遍。

   背景：2026-09-15 之前，停手的**唯一出口**是"杆积分出来的 h_sp 攒过 4cm"，
   而 h_sp 只在杆 >650（上升段）时才涨 ⇒ 杆在下降段（0~350）和中间段（350~650）
   推得再多，油门恒 1000µs、**电机一点都不转**，想解手必须推到上升段再保持约 0.8 秒。
   改成"**已解锁 + 杆从低位推过 200（边沿）**"之后：推过 200 就解手，
   油门交给控制器（1000 → 0.31s 到 1250 → 花 8~13 秒自己把悬停油门学出来）。
   ⭐ 本次**不做**"起飞段手动油门"（那是另一个方案），全程都是定高控制。

   真实参与编译的只有 vertical_control.c 和 pid.c，其余（估计器、参数表）用桩顶掉。

   编译（宿主机 gcc；本文件不碰任何硬件）：
     gcc -std=c99 -Wall -Wextra -Wno-unused-parameter ^
       -I sim_stub -I "ApexFly v2.0/Modules/Control" -I "ApexFly v2.0/Modules/Motor" ^
       -I "ApexFly v2.0/Modules/Communication" ^
       sim_vc_h14.c "ApexFly v2.0/Modules/Control/vertical_control.c" ^
       "ApexFly v2.0/Modules/Control/pid.c" -o sim_vc_h14.exe

   ⭐ 本仿真**开着真实外环**（h_kp=0.3、vz_kp=2 = 首飞参数），因为要向用户保证的一条行为
      正依赖它："杆停在下降段/中间段时，油门从 1250 慢慢漂到地板 1150，然后停住"。
      （H10/H12 那两个仿真反而必须 h_kp=0 才能与 H13 之前逐位等价 —— 是相反的选择，
        两者都对，别把它们改成一样。）

   ⚠️ 本文件里所有断言都是"把验收表写成数字"，其中三条是最要紧的：
      · 「7」杆停在 200~650 六十秒，`st` 必须**恒为 1** —— 这是"解手后不被判回停手"的证明；
        它同时守着本次最容易写错的改动（起飞保护的清除条件从"变化率<0"改成"杆≤100"）。
      · 「9」落地自动进停手 —— 守着"进入条件不许加杆量判据"这条决定。
      · 「10」上锁边界 —— 守着"`!armed` 要把起飞保护清掉"，否则解锁瞬间油门直接 1250。

   ⚠️ 仿真**不能**代替真机验证。它能证明：边沿、滞回、不抖、落地自动进停手、上锁边界、
      解手后加油门的下限、H12 回归。它证明不了：电机真的转（那要拆桨台架）。 */
#include "vertical_control.h"
#include "vertical_est.h"
#include "pid_table.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

/* ── 假的估计器：替换掉真的 vertical_est ── */
static VerticalEstState_t fake_est;

void VerticalEst_GetState(VerticalEstState_t *state) { *state = fake_est; }
void VerticalEst_Update(float roll_deg, float pitch_deg, float dt_s) { (void)roll_deg; (void)pitch_deg; (void)dt_s; }
void VerticalEst_Init(void) {}
void VerticalEst_Reset(void) {}

/* ── 假的参数表：替换掉真的 pid_table（它要 FreeRTOS 临界区 + 协议头）── */
static float stub_vz_kp = 2.0f;    /* 首飞值（H13 扫表定的，不是设计值 20） */
static float stub_vz_ki = 5.0f;
static float stub_vz_kd = 0.0f;

void PidTable_GetAltitudeParams(float *vz_kp, float *vz_ki, float *vz_kd)
{
    if (vz_kp != NULL) *vz_kp = stub_vz_kp;
    if (vz_ki != NULL) *vz_ki = stub_vz_ki;
    if (vz_kd != NULL) *vz_kd = stub_vz_kd;
}

/* 外环（H13）：本仿真用**真实值**，理由见文件头。 */
static float stub_h_kp = 0.3f;
static float stub_h_ki = 0.0f;
static float stub_h_kd = 0.0f;

void PidTable_GetHeightParams(float *h_kp, float *h_ki, float *h_kd)
{
    if (h_kp != NULL) *h_kp = stub_h_kp;
    if (h_ki != NULL) *h_ki = stub_h_ki;
    if (h_kd != NULL) *h_kd = stub_h_kd;
}

/* ── 从 vertical_control.h / motor.h 抄过来的常量（改那边就必须改这里）── */
#define VC_THR_MIN      1150.0f  /* VERTICAL_CONTROL_THROTTLE_MIN_US */
#define VC_THR_MAX      1650.0f  /* VERTICAL_CONTROL_THROTTLE_MAX_US */
#define VC_STICK_LOW    100U     /* VERTICAL_GROUND_STICK_LOW */
#define VC_EXIT_STICK   200U     /* VERTICAL_GROUND_EXIT_STICK */
#define VC_ENTER_H      4.0f     /* VERTICAL_GROUND_ENTER_H_CM */
#define VC_HOLD_MS      500U     /* VERTICAL_GROUND_ENTER_HOLD_MS */
#define VC_GUARD_H      10.0f    /* VERTICAL_TAKEOFF_GUARD_CLEAR_H_CM */
#define VC_START_US     1250.0f  /* VERTICAL_THROTTLE_START_US */
#define MT_PWM_MIN      1000U    /* MOTOR_PWM_MIN_US */

static const float DT = 0.004f;   /* 250Hz */
static const float H_GROUND = 3.0f;

static int checks = 0;
static int fails = 0;

static void check(const char *name, bool ok, const char *detail)
{
    checks++;
    if (!ok) fails++;
    printf("[%s] %s%s%s\n", ok ? "OK" : "失败", name, (detail != NULL) ? "  " : "", (detail != NULL) ? detail : "");
}

static VerticalControlSnapshot_t snap(void)
{
    VerticalControlSnapshot_t s;

    VerticalControl_GetSnapshot(&s);
    return s;
}

/* 驱动一拍。req 一般传 1（定高），只有测试模式切换时传 0。 */
static void tickReq(uint8_t req, uint16_t stick, bool armed, float roll, float pitch, float h, float vz, bool valid)
{
    fake_est.valid = valid;
    fake_est.height_cm = h;
    fake_est.vz_cms = vz;
    VerticalControl_Update(req, stick, true, armed, roll, pitch, DT);
}

static void tick(uint16_t stick, bool armed, float h, float vz, bool valid)
{
    tickReq(1U, stick, armed, 0.0f, 0.0f, h, vz, valid);
}

/* ⭐ 进定高 + 判在停手（上锁、杆最低、地面读数 3cm）。
   一拍就够：门禁①靠"上锁 + 杆 ≤50"全通 ⇒ 切进定高；而 ④ 段里有一条
   "切换时若读数在停手门槛内，直接把停手计时置满"，所以不用等 500ms 去抖。 */
static void setupGrounded(void)
{
    VerticalControl_Init();
    tick(0U, false, H_GROUND, 0.0f, true);
}

/* ⭐ 再解手到"控制启用"（st=1）。
   ⚠️ 解手靠**边沿**：上一拍杆是 0（低位），这一拍推 1000 ⇒ 成立。
      改回"只判电平"的实现（或把 setupGrounded 的第一拍杆改成 500）本函数就会卡住，
      后面所有断言会一起失败 —— 这是刻意的，它守着边沿条件。 */
static void setupFlying(float h_fly, float vz_fly)
{
    uint32_t k;

    setupGrounded();
    for (k = 0U; k < 400U; k++)
    {
        tick(1000U, true, h_fly, vz_fly, true);
        if (snap().state == (uint8_t)VERTICAL_STATE_ALT_HOLD) return;
    }
}

int main(void)
{
    VerticalControlSnapshot_t s;
    float hSpAtRelease;
    uint32_t k;
    bool armed;
    uint32_t thr;

    printf("========== 1 普通模式 / 停机状态回归 ==========\n");
    VerticalControl_Init();
    tickReq(0U, 0U, false, 0.0f, 0.0f, H_GROUND, 0.0f, true);
    s = snap();
    check("普通模式 st=0", s.state == (uint8_t)VERTICAL_STATE_MANUAL, NULL);
    check("普通模式 thr=1000", VerticalControl_GetThrottleUs() == MT_PWM_MIN, NULL);

    printf("\n========== 2 停手：上锁 + 杆最低 ==========\n");
    setupGrounded();
    s = snap();
    check("上锁 + 杆最低 + 读数正常 ⇒ st=4", s.state == (uint8_t)VERTICAL_STATE_GROUNDED, NULL);
    check("停手 thr=1000", VerticalControl_GetThrottleUs() == MT_PWM_MIN, NULL);
    /* 上锁状态下推杆：**不许解手**（出口要求 armed） */
    for (k = 0U; k < 300U; k++) tick(1000U, false, H_GROUND, 0.0f, true);
    s = snap();
    check("⭐ 上锁状态下把杆推到底 ⇒ 仍 st=4、thr=1000（不许解手）",
          (s.state == (uint8_t)VERTICAL_STATE_GROUNDED) && (VerticalControl_GetThrottleUs() == MT_PWM_MIN), NULL);

    printf("\n========== 3 ⭐ 解手边沿的三个边界（199 / 200 / 201）==========\n");
    /* 判据是 `stick > VERTICAL_GROUND_EXIT_STICK`，含不含等号是这类 bug 的老地方。
     200 必须**不解手**、201 必须解手。 */
    setupGrounded();
    tick(199U, true, H_GROUND, 0.0f, true);
    s = snap();
    check("杆 199 ⇒ 不解手（st=4）", s.state == (uint8_t)VERTICAL_STATE_GROUNDED, NULL);

    setupGrounded();
    tick(200U, true, H_GROUND, 0.0f, true);
    s = snap();
    check("杆 200（正好在门槛上）⇒ 不解手 —— 抓 `>` 被写成 `>=`", s.state == (uint8_t)VERTICAL_STATE_GROUNDED, NULL);

    setupGrounded();
    tick(201U, true, H_GROUND, 0.0f, true);
    s = snap();
    check("杆 201 ⇒ 解手、st=1", s.state == (uint8_t)VERTICAL_STATE_ALT_HOLD, NULL);

    printf("\n========== 4 ⭐ 解手要求「上一拍杆在低位」（边沿，不是电平）==========\n");
    /* 要验三件事，按危险程度排序：
       ① ⭐⭐ **锁着时先把杆推到底、再解锁** —— 只看电平的话会在解锁那一拍直接解手给 1250µs，
          地面上电机就转起来了（这是本次改动引入的新隐患，边沿条件就是为它加的）；
       ② guard 的**清零阈值是 100、不是 200**（滞回宽度 100）—— 解手后只把杆收到 150
          仍然回不了停手；若有人把阈值也写成 200，杆在 200 附近抖就会来回抖；
       ③ 先收杆到低位再推 ⇒ 正常解手。 */
    VerticalControl_Init();
    tick(0U, false, H_GROUND, 0.0f, true);            /* 切进定高、st=4，prev=0 */
    for (k = 0U; k < 200U; k++) tick(1000U, false, H_GROUND, 0.0f, true);   /* 上锁期间把杆推到底 */
    check("（前置）上锁推到底仍是 st=4", snap().state == (uint8_t)VERTICAL_STATE_GROUNDED, NULL);
    tick(1000U, true, H_GROUND, 0.0f, true);          /* 解锁，杆还推着 */
    s = snap();
    check("⭐⭐ 锁着时把杆推到底、再解锁 ⇒ **不解手**（边沿挡住；只看电平就会直接给 1250）",
          (s.state == (uint8_t)VERTICAL_STATE_GROUNDED) && (VerticalControl_GetThrottleUs() == MT_PWM_MIN), NULL);
    tick(90U, true, H_GROUND, 0.0f, true);            /* 先收回低位 */
    tick(400U, true, H_GROUND, 0.0f, true);           /* 再推上去 */
    check("先收杆到 90 再推到 400 ⇒ 解手、st=1", snap().state == (uint8_t)VERTICAL_STATE_ALT_HOLD, NULL);

    setupGrounded();
    tick(450U, true, H_GROUND, 0.0f, true);
    for (k = 0U; k < 300U; k++) tick(150U, true, H_GROUND, 0.0f, true);   /* 只收到 150（>100）*/
    s = snap();
    check("⭐ 解手后只收到 150（>100）⇒ 仍 st=1：guard 清零阈值是 100、滞回宽度 100",
          s.state == (uint8_t)VERTICAL_STATE_ALT_HOLD, NULL);
    for (k = 0U; k < 300U; k++) tick(90U, true, H_GROUND, 0.0f, true);    /* 再收到 90 */
    check("再收到 90 ⇒ 回停手", snap().state == (uint8_t)VERTICAL_STATE_GROUNDED, NULL);

    printf("\n========== 5 ⭐ 解手后油门交给控制器（不是 1000）==========\n");
    setupGrounded();
    thr = VerticalControl_GetThrottleUs();
    check("解手前 thr=1000（电机停）", thr == MT_PWM_MIN, NULL);
    tick(250U, true, H_GROUND, 0.0f, true);          /* 杆只推到下降段内 */
    s = snap();
    thr = VerticalControl_GetThrottleUs();
    check("⭐ 杆 250（下降段）+ 解锁 ⇒ st=1", s.state == (uint8_t)VERTICAL_STATE_ALT_HOLD, NULL);
    check("⭐⭐ 解手后 thr **不是** 1000（电机转）", thr != MT_PWM_MIN, NULL);
    check("⭐⭐ 解手后 thr ≥ 地板 1150（姿态环不会被旁路）", (float)thr >= VC_THR_MIN, NULL);
    check("解手那一拍 h_sp≈0（停手时归零 ⇒ 外环没有跳变）", s.h_sp_cm < 1.0f, NULL);
    check("解手那一拍 vzsp 为负（目标 0 < 实际 3cm）", s.vz_sp_cms < 0.0f, NULL);

    printf("\n========== 6 ⭐⭐ 杆停在中间段/下降段：油门会漂到地板并停住 ==========\n");
    /* 飞机在地面上、控制器在干活 ⇒ 外环看到"目标 0 < 实际 3cm" ⇒ vzsp<0
       ⇒ 慢层（悬停油门估计器）往下学 ⇒ 最终停在地板 1150。 */
    setupGrounded();
    tick(450U, true, H_GROUND, 0.0f, true);
    thr = VerticalControl_GetThrottleUs();
    check("刚解手时 thr 接近 1250", (thr > 1200U) && (thr <= 1300U), NULL);
    for (k = 0U; k < 20000U; k++) tick(450U, true, H_GROUND, 0.0f, true);   /* 80 秒 */
    thr = VerticalControl_GetThrottleUs();
    s = snap();
    check("⭐⭐ 80 秒后 thr 停在地板 1150（不是 1000、也没有继续掉）",
          (thr == 1150U) && (s.state == (uint8_t)VERTICAL_STATE_ALT_HOLD), NULL);
    check("慢层 i 也在它的下限上", (float)s.vz_throttle_base_us >= VC_THR_MIN - 0.5f, NULL);

    printf("\n========== 7 ⭐⭐⭐ 不抖：杆停在各个位置各 60 秒，st 必须恒为 1 ==========\n");
    /* 这一条守着本次**最容易写错**的改动：起飞保护的清除条件若还留着"变化率<0"，
       那么杆在下降段时保护立刻失效 ⇒ 500ms 后又判回停手 ⇒ st 在 1 与 4 之间来回跳。 */
    {
        const uint16_t sticks[5] = {201U, 250U, 350U, 450U, 650U};
        int n;
        for (n = 0; n < 5; n++)
        {
            int flips = 0;
            uint8_t prev_st = 0xFFU;
            char name[128];

            setupGrounded();
            tick(sticks[n], true, H_GROUND, 0.0f, true);
            for (k = 0U; k < 15000U; k++)      /* 60 秒 */
            {
                tick(sticks[n], true, H_GROUND, 0.0f, true);
                if (snap().state != prev_st)
                {
                    if (prev_st != 0xFFU) flips++;
                    prev_st = snap().state;
                }
            }
            sprintf(name, "⭐ 杆 %u 停 60 秒：st 恒为 1（翻转次数 %d，必须为 0）", (unsigned)sticks[n], flips);
            check(name, (flips == 0) && (prev_st == (uint8_t)VERTICAL_STATE_ALT_HOLD), NULL);
        }
    }

    printf("\n========== 8 ⭐ 收杆回停手 ==========\n");
    setupGrounded();
    tick(450U, true, H_GROUND, 0.0f, true);
    check("（前置）st=1", snap().state == (uint8_t)VERTICAL_STATE_ALT_HOLD, NULL);
    for (k = 0U; k < 100U; k++) tick(90U, true, H_GROUND, 0.0f, true);   /* 收杆到 90（<400ms） */
    check("收杆 400ms：还没到 500ms 去抖，仍是 st=1", snap().state == (uint8_t)VERTICAL_STATE_ALT_HOLD, NULL);
    for (k = 0U; k < 60U; k++) tick(90U, true, H_GROUND, 0.0f, true);    /* 再 240ms，累计 640ms */
    s = snap();
    check("⭐ 收杆到 90 满 500ms ⇒ 回 st=4、thr=1000",
          (s.state == (uint8_t)VERTICAL_STATE_GROUNDED) && (VerticalControl_GetThrottleUs() == MT_PWM_MIN), NULL);

    printf("\n========== 9 ⭐⭐ 落地自动进停手（进入条件不许加杆量判据）==========\n");
    /* 两条分别对应"杆在中位落地"和"杆在下降段落地"。
       若有人把"杆 ≤ 100"加进**进入条件**，这两条会同时失败。 */
    {
        const uint16_t land_sticks[2] = {450U, 250U};
        int n;
        for (n = 0; n < 2; n++)
        {
            char name[128];

            setupFlying(30.0f, 0.0f);            /* 飞到 30cm ⇒ 起飞保护被高度清掉 */
            for (k = 0U; k < 100U; k++) tick(land_sticks[n], true, 30.0f, 0.0f, true);
            check("（前置）st=1（外环开着，vzsp 会把它往目标拉）",
                  snap().state == (uint8_t)VERTICAL_STATE_ALT_HOLD, NULL);
            for (k = 0U; k < 300U; k++) tick(land_sticks[n], true, H_GROUND, 0.0f, true);   /* 落地 */
            sprintf(name, "⭐ 杆 %u 时落地 ⇒ 自动回 st=4、thr=1000", (unsigned)land_sticks[n]);
            check(name,
                  (snap().state == (uint8_t)VERTICAL_STATE_GROUNDED) && (VerticalControl_GetThrottleUs() == MT_PWM_MIN), NULL);
        }
    }

    printf("\n========== 10 ⭐⭐ 上锁边界：上锁必须把起飞保护清掉 ==========\n");
    /* 不清的话："解手 → 没飞起来（一直在 3cm）→ 上锁 → 再解锁"会带着 true 出来
       ⇒ 进不了停手 ⇒ 解锁后门禁②直接全通、油门给 1250 ⇒ **地面上电机转起来**。 */
    setupGrounded();
    tick(450U, true, H_GROUND, 0.0f, true);      /* 解手，但 h 一直是 3cm（没离地） */
    check("（前置）解手成功、st=1", snap().state == (uint8_t)VERTICAL_STATE_ALT_HOLD, NULL);
    for (k = 0U; k < 50U; k++) tick(450U, false, H_GROUND, 0.0f, true);   /* 上锁 */
    check("上锁时 thr=1000", VerticalControl_GetThrottleUs() == MT_PWM_MIN, NULL);
    for (k = 0U; k < 300U; k++) tick(0U, true, H_GROUND, 0.0f, true);     /* 再解锁（杆最低） */
    s = snap();
    check("⭐⭐ 上锁→解锁（飞机没离过地）⇒ 必须回到 st=4、thr=1000",
          (s.state == (uint8_t)VERTICAL_STATE_GROUNDED) && (VerticalControl_GetThrottleUs() == MT_PWM_MIN), NULL);
    check("（并确认）解锁后没有立刻给油：thr 不是 1250", VerticalControl_GetThrottleUs() != (uint32_t)VC_START_US, NULL);

    printf("\n========== 11 解手那一拍：h_sp 与起飞保护的交接 ==========\n");
    setupGrounded();
    for (k = 0U; k < 20U; k++) tick(0U, true, H_GROUND, 0.0f, true);   /* 解锁 + 杆最低，停留在停手 */
    tick(300U, true, H_GROUND, 0.0f, true);
    hSpAtRelease = snap().h_sp_cm;
    check("解手后 h_sp 从 0 起算（不含停手期间的残留）", hSpAtRelease < 1.0f, NULL);
    /* 立刻收杆到 0（不经过 500ms）：guard 被清 + 进入条件满足 ⇒ 500ms 后回停手 */
    for (k = 0U; k < 200U; k++) tick(0U, true, H_GROUND, 0.0f, true);
    check("解手后马上收杆 ⇒ 回停手（不会卡在控制状态）",
          snap().state == (uint8_t)VERTICAL_STATE_GROUNDED, NULL);

    printf("\n========== 12 H12 回归：解手之后拔 ToF ⇒ 保持 → 慢降 ==========\n");
    setupFlying(30.0f, 0.0f);
    for (k = 0U; k < 10U; k++) tick(450U, true, 30.0f, 0.0f, true);
    check("（前置）st=1", snap().state == (uint8_t)VERTICAL_STATE_ALT_HOLD, NULL);
    tick(450U, true, 30.0f, 0.0f, false);        /* 拔线：读数失效 */
    s = snap();
    check("拔线 ⇒ st=6（保持）、fr=1", (s.state == (uint8_t)VERTICAL_STATE_FAILSAFE_HOLD) && (s.fail_reason == 1U), NULL);
    {
        float thr_locked = (float)VerticalControl_GetThrottleUs();
        int moved = 0;
        for (k = 0U; k < 400U; k++)             /* 1.6s，还在 2s 保持段内 */
        {
            tick(450U, true, 30.0f, 0.0f, false);
            if ((float)VerticalControl_GetThrottleUs() != thr_locked) moved++;
        }
        check("⭐ 保持段 1.6s 内 thr 一点不动（与 H12 一致）", moved == 0, NULL);
    }
    for (k = 0U; k < 600U; k++) tick(450U, true, 30.0f, 0.0f, false);   /* 越过 2s ⇒ 进下降段 */
    s = snap();
    check("越过 2s ⇒ st=7（下降）", s.state == (uint8_t)VERTICAL_STATE_FAILSAFE_DESCEND, NULL);

    printf("\n========== 13 不变式遍历（杆量 × 解锁 × 高度）==========\n");
    {
        const uint16_t sticks[7] = {0U, 50U, 100U, 200U, 250U, 450U, 1000U};
        const float heights[2] = {3.0f, 30.0f};
        int bad_state = 0, bad_thr1 = 0, bad_ground = 0, bad_lock = 0, n, m, o;
        int cases = 0;

        for (n = 0; n < 7; n++)
        {
            for (m = 0; m < 2; m++)
            {
                for (o = 0; o < 2; o++)
                {
                    VerticalControl_Init();
                    tick(0U, false, heights[m], 0.0f, true);
                    armed = (o == 1);
                    for (k = 0U; k < 400U; k++)
                    {
                        uint32_t t;

                        tick(sticks[n], armed, heights[m], 0.0f, true);
                        s = snap();
                        t = VerticalControl_GetThrottleUs();
                        cases++;
                        if (s.state > (uint8_t)VERTICAL_STATE_FAILSAFE_DESCEND) bad_state++;
                        if ((s.state == (uint8_t)VERTICAL_STATE_ALT_HOLD) && (((float)t < VC_THR_MIN - 0.5f) || ((float)t > VC_THR_MAX + 0.5f))) bad_thr1++;
                        if ((s.state == (uint8_t)VERTICAL_STATE_GROUNDED) && (t != MT_PWM_MIN)) bad_ground++;
                        if ((!armed) && (t != MT_PWM_MIN)) bad_lock++;
                    }
                }
            }
        }
        check("st 只取 0/1/4/5/6/7（没有 8：本环节不做起飞段）", bad_state == 0, NULL);
        check("st=1 时 thr 恒在 [1150,1650]", bad_thr1 == 0, NULL);
        check("停手时 thr 恒为 1000", bad_ground == 0, NULL);
        check("上锁时 thr 恒为 1000", bad_lock == 0, NULL);
        printf("      （遍历 %d 拍）\n", cases);
    }

    printf("\n===== 检查 %d 项，失败 %d 项 =====\n", checks, fails);
    return (fails == 0) ? 0 : 1;
}
