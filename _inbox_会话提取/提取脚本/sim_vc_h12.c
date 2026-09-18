/* H12（失效处理：保持 → 平缓降至 1000）的台架仿真：把验收矩阵在**烧录之前**先跑一遍。

   真实参与编译的只有 vertical_control.c 和 pid.c，其余（估计器、参数表）用桩顶掉。

   编译（宿主机 gcc；本文件不碰任何硬件）：
     gcc -std=c99 -Wall -Wextra -Wno-unused-parameter ^
       -I sim_stub -I "ApexFly v2.0/Modules/Control" -I "ApexFly v2.0/Modules/Motor" ^
       -I "ApexFly v2.0/Modules/Communication" ^
       sim_vc_h12.c "ApexFly v2.0/Modules/Control/vertical_control.c" ^
       "ApexFly v2.0/Modules/Control/pid.c" -o sim_vc_h12.exe

   ⚠️ 本环节写代码时**自己踩了两个坑**，都在下面有专门的断言守着（不让它复发）：
     ① **高度越界不在门禁②里** ⇒ "h>125cm 且其它一切正常"那一拍 `IsActive()` 恰好为 true。
        第一版在"门禁全通就 return"那里直接返回，高度失效整条判据变成了死代码。
        ⇒ 断言见「8 高度越界」。
     ② **地面上重新解锁 + ToF 坏的**：停手要求 est_valid，读数坏着就永远进不去停手，
        于是"锁存悬停油门"会在**地面上**把油门顶到 ~1250µs。
        ⇒ 加了 `ever_active` 守卫（"本次解锁后有没有真的飞过"），断言见「4 守卫④」。

   ⚠️ 仿真**不能**代替真机验证。它能证明的是：状态机路径、锁存值、斜坡斜率、
      单调性、恢复的无缝性、守卫的取舍。它证明不了：电机真的转、以及落地有多硬
      （那要用 sim_failsafe_ramp.js 的物理模型 + 真机台架）。 */
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
static float stub_vz_kp = 20.0f;
static float stub_vz_ki = 5.0f;
static float stub_vz_kd = 0.0f;

void PidTable_GetAltitudeParams(float *vz_kp, float *vz_ki, float *vz_kd)
{
    if (vz_kp != NULL) *vz_kp = stub_vz_kp;
    if (vz_ki != NULL) *vz_ki = stub_vz_ki;
    if (vz_kd != NULL) *vz_kd = stub_vz_kd;
}

/* ── 外环（H13 新增）：本仿真**刻意把外环关掉** ──────────────────
   本文件验的是 H10/H12 的行为（慢层积分、失效状态机），而 H13 把慢层的误差来源
   从 `0 − vz` 改成了 `vz_sp − vz` —— 只要让 `h_kp = 0`，外环输出就**恒等于 0**，
   于是整条链路与 H13 之前**逐位等价**。
   ⭐ 这么做让本仿真从一个"过时的脚本"变成一个**真正的回归测试**：
      "H13 的改动在 h_kp=0 时没有改变任何行为"。
   ⚠️ 千万不要把它改成非 0 去"顺便验外环" —— 那是 sim_vc_h13.c 的活。 */
static float stub_h_kp = 0.0f;
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
#define VC_RAMP         30.0f    /* VERTICAL_FS_RAMP_US_PER_S */
#define VC_HOLD_MS      2000U    /* VERTICAL_FS_HOLD_MS */
#define VC_H_MAX        125.0f   /* VERTICAL_FS_H_MAX_CM */
#define VC_H_RECOVER    115.0f   /* VERTICAL_FS_H_RECOVER_CM */
#define VC_TILT         30.0f    /* VERTICAL_CONTROL_MAX_TILT_DEG（2026-09-15 由 25 改成 30） */
#define MT_PWM_MIN      1000U    /* MOTOR_PWM_MIN_US */

static const float DT = 0.004f;   /* 250Hz */
static const float I_START = 1250.0f;  /* VERTICAL_THROTTLE_START_US */

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

/* ⭐ 进定高 + 起飞到"控制启用"（st=1）。
   地面阶段 h=3cm（进停手，杆 0）；解手靠**推杆边沿**（H14 起）：上一拍杆 0、这一拍推 1000
   ⇒ **第一拍就变成 st=1**。旧版要"h_sp 涨过 4cm、约 334 拍"，那个出口已删除。
   起飞后估计值固定成调用方给的那组，飞到哪由参数定。 */
static void setupFlying(float h_fly, float vz_fly)
{
    uint32_t k;

    VerticalControl_Init();
    tick(0U, false, 3.0f, 0.0f, true);          /* 门禁①全通 ⇒ 切进定高 + 立刻判在停手 */
    for (k = 0U; k < 400U; k++)
    {
        tick(1000U, true, h_fly, vz_fly, true);
        if (snap().state == (uint8_t)VERTICAL_STATE_ALT_HOLD) return;
    }
}

/* 在"控制启用"下把慢层 i 推到某个值：喂 vz 让它积分。
   vz<0（下沉）⇒ i 上升，每拍 +|vz|*ki*dt。返回推完之后的实际输出油门（= 失效时的锁存值）。 */
static float raiseBaseByVz(float vz, int frames)
{
    int k;

    for (k = 0; k < frames; k++)
    {
        tick(1000U, true, 50.0f, vz, true);
    }
    return (float)VerticalControl_GetThrottleUs();
}

int main(void)
{
    VerticalControlSnapshot_t s;
    float lock;
    float before;
    float after;
    float prevThr;
    uint32_t k;
    int i;
    int j;

    printf("========== 1 普通模式 / 初始化回归 ==========\n");
    VerticalControl_Init();
    tickReq(0U, 0U, false, 0.0f, 0.0f, 3.0f, 0.0f, true);
    s = snap();
    check("普通模式 st=0", s.state == (uint8_t)VERTICAL_STATE_MANUAL, NULL);
    check("普通模式 thr=1000", VerticalControl_GetThrottleUs() == MT_PWM_MIN, NULL);
    check("普通模式 fs=0 fr=0", (s.failsafe_ms == 0U) && (s.fail_reason == 0U), NULL);

    printf("\n========== 2 停手豁免（H12 最危险的误判）==========\n");
    VerticalControl_Init();
    tick(0U, false, 3.0f, 0.0f, true);
    s = snap();
    check("上锁 + 杆最低 + 读数正常 ⇒ st=4 停手", s.state == (uint8_t)VERTICAL_STATE_GROUNDED, NULL);
    check("停手 thr=1000（**没有**被失效逻辑锁存成悬停值）", VerticalControl_GetThrottleUs() == MT_PWM_MIN, NULL);
    check("停手 fs=0 fr=0（停手不是失效）", (s.failsafe_ms == 0U) && (s.fail_reason == 0U), NULL);
    /* 停手期间读数坏掉（地面上的 ToF 被踩到/挡住）：仍然必须 1000 */
    for (k = 0U; k < 200U; k++) tick(0U, false, 3.0f, 0.0f, false);
    s = snap();
    check("停手中读数失效 ⇒ 仍 st=4、thr=1000（不能锁存）",
          (s.state == (uint8_t)VERTICAL_STATE_GROUNDED) && (VerticalControl_GetThrottleUs() == MT_PWM_MIN), NULL);

    printf("\n========== 3 起飞后正常（st=1 回归）==========\n");
    setupFlying(50.0f, 0.0f);
    s = snap();
    check("st=1", s.state == (uint8_t)VERTICAL_STATE_ALT_HOLD, NULL);
    check("i=1250（初值，没被碰过）", (s.vz_throttle_base_us > 1249.9f) && (s.vz_throttle_base_us < 1250.1f), NULL);
    check("vz=0 ⇒ corr=0、thr=1250", VerticalControl_GetThrottleUs() == 1250U, NULL);

    printf("\n========== 4 ⭐ 守卫④：地面上重新解锁 + 读数坏着（H12 自己引入的风险）==========\n");
    setupFlying(50.0f, 0.0f);                     /* 飞过 ⇒ ever_active=true */
    tick(1000U, false, 50.0f, 0.0f, true);        /* 上锁 ⇒ 守卫① 清 ever_active */
    check("上锁后 st 不再是 1", snap().state != (uint8_t)VERTICAL_STATE_ALT_HOLD, NULL);
    /* 现在在地面上重新解锁，而 ToF 是坏的：停手要求 est_valid ⇒ 永远进不去停手 */
    for (k = 0U; k < 400U; k++) tick(0U, true, 3.0f, 0.0f, false);
    s = snap();
    check("st 不是 6/7（**不允许**对没飞过的飞机做失效保持）",
          (s.state != (uint8_t)VERTICAL_STATE_FAILSAFE_HOLD) && (s.state != (uint8_t)VERTICAL_STATE_FAILSAFE_DESCEND), NULL);
    check("⭐ thr=1000（若没有守卫④，这里会是 1250 = 地面上顶到接近悬停）",
          VerticalControl_GetThrottleUs() == MT_PWM_MIN, NULL);
    check("fs=0 fr=0", (s.failsafe_ms == 0U) && (s.fail_reason == 0U), NULL);

    printf("\n========== 5 ⭐⭐ ToF 失效 ⇒ 保持段（锁存 + 恒定）==========\n");
    setupFlying(50.0f, 0.0f);
    lock = raiseBaseByVz(-20.0f, 500);            /* 把 i 推到 1450 ⇒ corr=+50 ⇒ thr=1500 */
    {
        char buf[96];
        sprintf(buf, "锁存值（= 失效前一刻的输出）thr=%u", (unsigned)lock);
        check("前置：i 已经被推离初值，锁存值明显 > 1250", lock > 1400.0f, buf);
    }
    tick(1000U, true, 50.0f, -20.0f, false);      /* 第一拍失效 */
    s = snap();
    check("立刻 st=6（失效保持）", s.state == (uint8_t)VERTICAL_STATE_FAILSAFE_HOLD, NULL);
    check("fr=1（ToF 读数失效）", s.fail_reason == (uint8_t)VERTICAL_FAIL_SENSOR, NULL);
    check("⭐ thr 立刻等于锁存值（没有掉到 1000）", (float)VerticalControl_GetThrottleUs() == lock, NULL);
    check("fs 从 0 开始", s.failsafe_ms == 0U, NULL);
    {
        float iKept = s.vz_throttle_base_us;
        bool constThr = true;
        prevThr = (float)VerticalControl_GetThrottleUs();
        /* ⚠️ 判据要分成**两段**，不能一个循环里同时验"恒定"和"计时满"：
           · 前 490 拍（1.96s）**还在保持段内** ⇒ 才能验"thr 一点不动"；
           · 再 20 拍跨过 2000ms ⇒ 才能验 fs 涨满。
           第一版用一个 520 拍的循环同时验两件事，结果区间内就转成了下降段，
           恒定判据必然失败 —— 又是"判据没算上实现细节"。
           另外注意：**进入保持的那一拍不累加计时**（它在状态机里直接 return 了），
           所以"2 秒"要 501 拍才满。差 1 拍（4ms）对行为没有影响，但判据写 500 会误报。 */
        for (k = 0U; k < 490U; k++)
        {
            tick(1000U, true, 50.0f, -20.0f, false);
            if ((float)VerticalControl_GetThrottleUs() != prevThr) constThr = false;
            prevThr = (float)VerticalControl_GetThrottleUs();
        }
        s = snap();
        {
            char buf[96];
            sprintf(buf, "1.96 秒后 fs=%lums、st=%u、thr=%u", (unsigned long)s.failsafe_ms, (unsigned)s.state, (unsigned)VerticalControl_GetThrottleUs());
            check("⭐⭐ 保持段内 thr **一点不动**", constThr && (s.state == (uint8_t)VERTICAL_STATE_FAILSAFE_HOLD), buf);
        }
        check("保持段慢层 i 冻结（没被按回 1250）", s.vz_throttle_base_us == iKept, NULL);
        for (k = 0U; k < 20U; k++) tick(1000U, true, 50.0f, -20.0f, false);
        s = snap();
        {
            char buf[96];
            sprintf(buf, "fs=%lums（保持门槛 %ums），st=%u", (unsigned long)s.failsafe_ms, (unsigned)VC_HOLD_MS, (unsigned)s.state);
            check("跨过 2000ms ⇒ fs 涨满并转入下降", (s.failsafe_ms >= VC_HOLD_MS) && (s.state == (uint8_t)VERTICAL_STATE_FAILSAFE_DESCEND), buf);
        }
    }

    printf("\n========== 6 ⭐⭐ 短暂失效恢复 ⇒ 无缝，什么都不发生 ==========\n");
    setupFlying(50.0f, 0.0f);
    lock = raiseBaseByVz(-20.0f, 500);
    before = (float)VerticalControl_GetThrottleUs();
    tick(1000U, true, 50.0f, -20.0f, false);      /* 失效 1 拍 */
    for (i = 0; i < 100; i++) tick(1000U, true, 50.0f, -20.0f, false);   /* 保持 0.4 秒 */
    s = snap();
    check("保持中：st=6", s.state == (uint8_t)VERTICAL_STATE_FAILSAFE_HOLD, NULL);
    tick(1000U, true, 50.0f, -20.0f, true);       /* 读数回来了 */
    s = snap();
    check("⭐ 恢复：st 回 1", s.state == (uint8_t)VERTICAL_STATE_ALT_HOLD, NULL);
    check("恢复：fs=0 fr=0", (s.failsafe_ms == 0U) && (s.fail_reason == 0U), NULL);
    check("⭐⭐ 恢复：慢层 i **没有丢**（还是那次学到的高值）", s.vz_throttle_base_us > 1400.0f, NULL);
    after = (float)VerticalControl_GetThrottleUs();
    {
        char buf[96];
        sprintf(buf, "失效前 %u → 恢复后 %u（差 %.0f）", (unsigned)before, (unsigned)after, (double)(after - before));
        check("⭐⭐ 恢复：thr 与失效前**完全一致**（无扰）", after == before, buf);
    }

    printf("\n========== 7 ⭐⭐ 保持期满 ⇒ 下降段（斜率 / 单调 / 地板）==========\n");
    setupFlying(50.0f, 0.0f);
    lock = raiseBaseByVz(-20.0f, 500);
    tick(1000U, true, 50.0f, -20.0f, false);
    for (k = 0U; k < 600U; k++) tick(1000U, true, 50.0f, -20.0f, false);
    s = snap();
    check("跑过 2 秒后 st=7（失效下降）", s.state == (uint8_t)VERTICAL_STATE_FAILSAFE_DESCEND, NULL);
    check("转入下降那一拍 thr 仍接近锁存值", (float)VerticalControl_GetThrottleUs() <= lock, NULL);
    {
        float t0 = (float)VerticalControl_GetThrottleUs();
        int mono = 1;
        float worst = 0.0f;
        prevThr = t0;
        for (k = 0U; k < 250U; k++)                  /* 1 秒 */
        {
            tick(1000U, true, 50.0f, -20.0f, false);
            {
                float t = (float)VerticalControl_GetThrottleUs();
                if (t > prevThr + 0.001f) mono = 0;
                if ((prevThr - t) > worst) worst = prevThr - t;
                prevThr = t;
            }
        }
        {
            char buf[128];
            sprintf(buf, "1 秒掉了 %.1fµs（期望 %.0f），最快单拍掉 %.1fµs", (double)(t0 - prevThr), (double)VC_RAMP, (double)worst);
            check("⭐⭐ 下降斜率 ≈ 30µs/s", (t0 - prevThr) > (VC_RAMP - 1.0f) && (t0 - prevThr) < (VC_RAMP + 1.0f), buf);
        }
        check("下降单调不增", mono, NULL);
    }
    /* 一直跑到地板 */
    for (k = 0U; k < 8000U; k++) tick(1000U, true, 50.0f, -20.0f, false);
    s = snap();
    check("⭐⭐ 最终停在 1000（电机停转）", VerticalControl_GetThrottleUs() == MT_PWM_MIN, NULL);
    check("到地板后仍 st=7（不自动回停手 —— 读数还坏着，停手进不去）",
          s.state == (uint8_t)VERTICAL_STATE_FAILSAFE_DESCEND, NULL);
    check("下降段慢层 i 依然冻结", s.vz_throttle_base_us > 1400.0f, NULL);
    for (k = 0U; k < 2000U; k++) tick(1000U, true, 50.0f, -20.0f, false);
    check("地板不会漏（再跑 8 秒仍是 1000）", VerticalControl_GetThrottleUs() == MT_PWM_MIN, NULL);

    printf("\n========== 8 ⭐⭐ 高度越界（踩过的坑：它不在门禁②里）==========\n");
    setupFlying(50.0f, 0.0f);
    lock = raiseBaseByVz(-20.0f, 500);
    tick(1000U, true, 130.0f, -20.0f, true);      /* h=130cm，其它一切正常 ⇒ 门禁②全通 */
    s = snap();
    check("门禁②此时确实是全通的（这就是坑的根源）", s.control_gates == VERTICAL_GATE_CT_ALL, NULL);
    check("⭐ 仍然进了 st=6（若在\"门禁全通就 return\"那里返回，这条判据是死的）",
          s.state == (uint8_t)VERTICAL_STATE_FAILSAFE_HOLD, NULL);
    check("fr=3（高度越界）", s.fail_reason == (uint8_t)VERTICAL_FAIL_HEIGHT, NULL);
    check("高度越界时也是锁存而不是掉到 1000", (float)VerticalControl_GetThrottleUs() == lock, NULL);

    printf("\n========== 9 高度滞回：125 进 / 115 出，中间不抖 ==========\n");
    for (k = 0U; k < 300U; k++) tick(1000U, true, 120.0f, -20.0f, true);   /* h=120，在 115~125 之间 */
    s = snap();
    check("h=120（进门槛 125 与出门槛 115 之间）⇒ **仍保持**，没有来回切",
          s.state == (uint8_t)VERTICAL_STATE_FAILSAFE_HOLD, NULL);
    tick(1000U, true, 110.0f, -20.0f, true);       /* 降到 115 以下 */
    s = snap();
    check("h=110 < 115 ⇒ 恢复，st 回 1", s.state == (uint8_t)VERTICAL_STATE_ALT_HOLD, NULL);
    check("恢复后 fs=0 fr=0", (s.failsafe_ms == 0U) && (s.fail_reason == 0U), NULL);

    printf("\n========== 10 ⭐ 姿态超限 ⇒ 不保持、立即 1000（用户 2026-09-15 的决定）==========\n");
    setupFlying(50.0f, 0.0f);
    lock = raiseBaseByVz(-20.0f, 500);
    tickReq(1U, 1000U, true, 30.0f, 0.0f, 50.0f, -20.0f, true);   /* roll=30° */
    s = snap();
    check("⭐ **不进 st=6**（姿态超限跳过保持段）", s.state != (uint8_t)VERTICAL_STATE_FAILSAFE_HOLD, NULL);
    check("⭐ 直接 st=7 且 thr=1000（判定炸机、立即停桨）", (s.state == (uint8_t)VERTICAL_STATE_FAILSAFE_DESCEND) && (VerticalControl_GetThrottleUs() == MT_PWM_MIN), NULL);
    check("fr=2（姿态超限）", s.fail_reason == (uint8_t)VERTICAL_FAIL_ATTITUDE, NULL);
    check("fs=0（没有保持计时）", s.failsafe_ms == 0U, NULL);
    /* 俯仰也要能触发（两个轴各自判断） */
    setupFlying(50.0f, 0.0f);
    tickReq(1U, 1000U, true, 0.0f, -30.0f, 50.0f, 0.0f, true);
    check("pitch=+/-30° 同样触发（fr=2、thr=1000）",
          (snap().fail_reason == (uint8_t)VERTICAL_FAIL_ATTITUDE) && (VerticalControl_GetThrottleUs() == MT_PWM_MIN), NULL);
    /* ⭐ 边界：门槛是 **30°**（含等号取 >=），所以 29° 不许触发、30° 必须触发。
       ⚠️ 这一条是刻意的边界测试：判据写成 `>` 而不是 `>=` 时它会立刻失败。 */
    setupFlying(50.0f, 0.0f);
    tickReq(1U, 1000U, true, 29.0f, 0.0f, 50.0f, 0.0f, true);
    s = snap();
    check("roll=29° < 30° ⇒ 不触发（st 仍 1、fr=0）",
          (s.state == (uint8_t)VERTICAL_STATE_ALT_HOLD) && (s.fail_reason == 0U), NULL);
    setupFlying(50.0f, 0.0f);
    tickReq(1U, 1000U, true, 30.0f, 0.0f, 50.0f, 0.0f, true);
    check("roll=30°（正好在门槛上，判据含等号）⇒ **必须触发** fr=2",
          (snap().fail_reason == (uint8_t)VERTICAL_FAIL_ATTITUDE), NULL);

    printf("\n========== 11 保持期间姿态变坏 ⇒ 立即转为 1000 ==========\n");
    setupFlying(50.0f, 0.0f);
    lock = raiseBaseByVz(-20.0f, 500);
    tick(1000U, true, 50.0f, -20.0f, false);
    for (i = 0; i < 100; i++) tick(1000U, true, 50.0f, -20.0f, false);   /* 先保持 0.4 秒 */
    check("前置：st=6 且 thr=锁存值",
          (snap().state == (uint8_t)VERTICAL_STATE_FAILSAFE_HOLD) && ((float)VerticalControl_GetThrottleUs() == lock), NULL);
    tickReq(1U, 1000U, true, 30.0f, 0.0f, 50.0f, -20.0f, false);          /* 这时姿态歪了 */
    s = snap();
    check("⭐ 保持段中姿态变坏 ⇒ 立刻 st=7、thr=1000、fr=2",
          (s.state == (uint8_t)VERTICAL_STATE_FAILSAFE_DESCEND) && (VerticalControl_GetThrottleUs() == MT_PWM_MIN) &&
          (s.fail_reason == (uint8_t)VERTICAL_FAIL_ATTITUDE), NULL);

    printf("\n========== 12 失效中落地（高度越界那条，读数还是好的）==========\n");
    setupFlying(50.0f, 0.0f);
    tick(1000U, true, 130.0f, -20.0f, true);
    for (k = 0U; k < 600U; k++) tick(1000U, true, 130.0f, -20.0f, true);  /* 保持期满 ⇒ 下降 */
    check("前置：st=7", snap().state == (uint8_t)VERTICAL_STATE_FAILSAFE_DESCEND, NULL);
    /* 落地：高度掉到地面读数、杆收到底（命令下降），停手该接管 */
    for (k = 0U; k < 400U; k++) tick(0U, true, 3.0f, 0.0f, true);
    s = snap();
    check("落地后停手接管：st=4、thr=1000", (s.state == (uint8_t)VERTICAL_STATE_GROUNDED) && (VerticalControl_GetThrottleUs() == MT_PWM_MIN), NULL);
    check("落地后 fs=0 fr=0（失效状态被停手清掉）", (s.failsafe_ms == 0U) && (s.fail_reason == 0U), NULL);

    printf("\n========== 13 上锁 / 复位清失效 ==========\n");
    setupFlying(50.0f, 0.0f);
    lock = raiseBaseByVz(-20.0f, 500);
    tick(1000U, true, 50.0f, -20.0f, false);
    for (k = 0U; k < 600U; k++) tick(1000U, true, 50.0f, -20.0f, false);
    check("前置：st=7", snap().state == (uint8_t)VERTICAL_STATE_FAILSAFE_DESCEND, NULL);
    tick(1000U, false, 50.0f, -20.0f, false);      /* 上锁 */
    s = snap();
    check("上锁 ⇒ thr=1000、fs=0 fr=0", (VerticalControl_GetThrottleUs() == MT_PWM_MIN) && (s.failsafe_ms == 0U) && (s.fail_reason == 0U), NULL);
    /* Reset（模式切换 / IMU 重初始化会调它）也要清干净 */
    setupFlying(50.0f, 0.0f);
    tick(1000U, true, 50.0f, -20.0f, false);
    check("前置：st=6", snap().state == (uint8_t)VERTICAL_STATE_FAILSAFE_HOLD, NULL);
    VerticalControl_Reset();
    s = snap();
    check("Reset() 清失效（fs=0 fr=0，i 回初值）",
          (s.failsafe_ms == 0U) && (s.fail_reason == 0U) && (s.vz_throttle_base_us > I_START - 0.1f) && (s.vz_throttle_base_us < I_START + 0.1f), NULL);

    printf("\n========== 14 遥测字节：bit5-6 = 失效原因 ==========\n");
    setupFlying(50.0f, 0.0f);
    tick(1000U, true, 50.0f, 0.0f, false);
    s = snap();
    check("ToF 失效：telemetry_mode bit5-6 = 1（且 bit0-3 = 模式 1）",
          ((s.telemetry_mode & 0x07U) == 1U) && (((s.telemetry_mode >> 5) & 0x03U) == 1U), NULL);
    setupFlying(50.0f, 0.0f);
    tickReq(1U, 1000U, true, 30.0f, 0.0f, 50.0f, 0.0f, true);
    s = snap();
    check("姿态超限：bit5-6 = 2", ((s.telemetry_mode >> 5) & 0x03U) == 2U, NULL);
    setupFlying(50.0f, 0.0f);
    tick(1000U, true, 50.0f, 0.0f, true);
    s = snap();
    check("正常：bit5-6 = 0", ((s.telemetry_mode >> 5) & 0x03U) == 0U, NULL);

    printf("\n========== 15 ★ 不变式遍历 ==========\n");
    {
        static const uint16_t sticks[4] = { 0U, 300U, 700U, 1000U };
        static const float rolls[3] = { 0.0f, 20.0f, 30.0f };
        static const float hs[3] = { 3.0f, 50.0f, 130.0f };
        static const bool valids[2] = { true, false };
        static const bool arms[2] = { true, false };
        int combos = 0;
        int stBad = 0;
        int thrBad = 0;
        int rangeBad = 0;
        int reasonBad = 0;
        int groundBad = 0;
        int rampBad = 0;
        int holdBad = 0;
        int st1RangeBad = 0;
        uint16_t thrMin = 65535U;
        uint16_t thrMax = 0U;

        for (i = 0; i < 4; i++)
        {
            for (j = 0; j < 3; j++)
            {
                int a;
                for (a = 0; a < 2; a++)
                {
                    int v;
                    for (v = 0; v < 2; v++)
                    {
                        int hi;
                        for (hi = 0; hi < 3; hi++)
                        {
                            uint16_t prev;
                            uint8_t prevState;
                            VerticalControlSnapshot_t s0;

                            combos++;
                            setupFlying(50.0f, 0.0f);
                            prev = 0U;
                            prevState = 0U;
                            for (k = 0U; k < 400U; k++)
                            {
                                uint16_t thr;

                                tickReq(1U, sticks[i], arms[a], rolls[j], 0.0f, hs[hi], -20.0f, valids[v]);
                                s0 = snap();
                                thr = VerticalControl_GetThrottleUs();
                                if (thr < thrMin) thrMin = thr;
                                if (thr > thrMax) thrMax = thr;

                                if ((s0.state != 0U) && (s0.state != 1U) && (s0.state != 4U) && (s0.state != 5U) &&
                                    (s0.state != 6U) && (s0.state != 7U)) stBad++;
                                if ((thr < MT_PWM_MIN) || (thr > (uint16_t)VC_THR_MAX)) thrBad++;
                                if ((s0.state == 1U) && ((thr < (uint16_t)VC_THR_MIN) || (thr > (uint16_t)VC_THR_MAX))) rangeBad++;
                                /* fr 非 0 ⟺ 处于 6/7 */
                                if (((s0.fail_reason != 0U) != ((s0.state == 6U) || (s0.state == 7U)))) reasonBad++;
                                /* 上锁 / 停手 ⇒ 恒 1000 */
                                if (((!arms[a]) || (s0.state == 4U)) && (thr != MT_PWM_MIN)) groundBad++;
                                /* ⚠️ "油门不上升"只对**下降段**成立。
                                   套到所有状态上是错的：st=1 时慢层正在积分，油门本来就该会上升
                                   （本仿真一直喂 vz=-20 ⇒ i 一直在涨 ⇒ thr 一直在涨）。
                                   —— 第一版就是这么误报的。 */
                                if ((prevState == 7U) && (s0.state == 7U) && (prev != 0U) && (thr > prev)) rampBad++;
                                /* st=6 期间必须恒定 */
                                if ((prevState == 6U) && (s0.state == 6U) && (prev != 0U) && (thr != prev)) holdBad++;
                                prev = thr;
                                prevState = s0.state;
                            }
                        }
                    }
                }
            }
        }
        {
            char buf[128];
            sprintf(buf, "%d 种组合 × 400 拍，观察到 thr 范围 %u ~ %u", combos, (unsigned)thrMin, (unsigned)thrMax);
            check("★ st 恒在 {0,1,4,5,6,7}", stBad == 0, buf);
        }
        check("★ thr 恒在 [1000, 1650]", thrBad == 0, NULL);
        check("★ st=1 时 thr ∈ [1150, 1650]（地板挡住姿态环旁路线 1100）", rangeBad == 0, NULL);
        check("★ fr≠0 ⟺ st∈{6,7}", reasonBad == 0, NULL);
        check("★ 上锁 / 停手时 thr 恒 1000", groundBad == 0, NULL);
        check("★ st=7 时 thr 单调不增（下降段不许回升）", rampBad == 0, NULL);
        check("★ st=6 时 thr 恒定（保持段一个字节都不动）", holdBad == 0, NULL);
        (void)st1RangeBad;
    }

    printf("\n========== 16 与真机台架验收的对照量（抄这些数去比对）==========\n");
    {
        setupFlying(50.0f, 0.0f);
        lock = raiseBaseByVz(-20.0f, 500);
        printf("  锁存值（i=1450 + corr=+50）            = %uµs\n", (unsigned)lock);
        tick(1000U, true, 50.0f, -20.0f, false);
        printf("  失效第 1 拍 thr                         = %uµs  （应等于锁存值）\n", (unsigned)VerticalControl_GetThrottleUs());
        for (k = 0U; k < 500U; k++) tick(1000U, true, 50.0f, -20.0f, false);
        printf("  fs 到 2000ms 那一拍 thr                 = %uµs\n", (unsigned)VerticalControl_GetThrottleUs());
        for (k = 0U; k < 250U; k++) tick(1000U, true, 50.0f, -20.0f, false);
        printf("  下降 1 秒后 thr                         = %uµs  （应 ≈ 锁存值 − 30）\n", (unsigned)VerticalControl_GetThrottleUs());
        printf("  真机上相邻两拍打印（500ms）的 thr 差    = ≈ 15µs\n");
        printf("  从 1500µs 降到 1000 需要                = %.1f 秒\n", (double)((lock - 1000.0f) / VC_RAMP));
    }

    printf("\n===== 检查 %d 项，失败 %d 项 =====\n", checks, fails);
    printf("%s\n", (fails == 0) ? "全部通过" : "!! 有失败项，必须先查清再烧录");
    return (fails == 0) ? 0 : 1;
}
