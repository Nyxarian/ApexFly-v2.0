/* H10（悬停油门估计器 = 慢层积分）的台架仿真：把验收矩阵在**烧录之前**先跑一遍。
   真实参与编译的只有 vertical_control.c 和 pid.c，其余（估计器、参数表）用桩顶掉。

   编译（宿主机 gcc；本文件不碰任何硬件）：
     gcc -std=c99 -Wall -Wextra -Wno-unused-parameter ^
       -I sim_stub -I "ApexFly v2.0/Modules/Control" -I "ApexFly v2.0/Modules/Motor" ^
       -I "ApexFly v2.0/Modules/Communication" ^
       sim_vc_h10.c "ApexFly v2.0/Modules/Control/vertical_control.c" ^
       "ApexFly v2.0/Modules/Control/pid.c" -o sim_vc_h10.exe

   说明：motor.h 会 include "main.h"（HAL 聚合头），宿主机上不需要它的任何内容，
        所以用 sim_stub/main.h 顶掉。sim_stub 绝不能再加 Keil 的包含路径。

   本环节（H10）最要紧的三条，都是"算错了会失控"的：
     ① ⭐ **慢层的符号**：匀速上抬（vz>0）必须让 i **下降**。
        写反就是"上升还加油门"的正反馈，与快层那个减号同性质。
     ② ⭐ **积分与输出同锁**：i 和 thr 都钳在 [1150, 1650]，不能只钳一个（= windup）。
     ③ ⭐ **地板 1150 > 姿态环旁路线 1100**：定高模式下永远不许把油门给到 ≤1100。

   ⚠️ 仿真**不能**代替真机验证。它能证明的是：符号、限幅、有界、失效闭嘴、
      参数在线变更的行为。它证明不了：电机真的转、以及**vz_ki 取多少合适**
      （台架上飞机不真飞，积分没有真实工作点可收敛）。真值只能靠首飞整定。
      真机验收步骤见 `优化一下/高度接入施工顺序.md` 的「H10 验收步骤」。 */
#include "vertical_control.h"
#include "vertical_est.h"
#include "pid_table.h"
#include <stdbool.h>
#include <stdio.h>

/* ── 假的估计器：替换掉真的 vertical_est ── */
static VerticalEstState_t fake_est;

void VerticalEst_GetState(VerticalEstState_t *state) { *state = fake_est; }
void VerticalEst_Update(float roll_deg, float pitch_deg, float dt_s) { (void)roll_deg; (void)pitch_deg; (void)dt_s; }
void VerticalEst_Init(void) {}
void VerticalEst_Reset(void) {}

/* ── 假的参数表：替换掉真的 pid_table（它要 FreeRTOS 临界区 + 协议头）── */
static float stub_vz_kp = 20.0f;   /* H9 实测值 */
static float stub_vz_ki = 5.0f;    /* H10 起步值（用户定的） */
static float stub_vz_kd = 0.0f;

void PidTable_GetAltitudeParams(float *vz_kp, float *vz_ki, float *vz_kd)
{
    if (vz_kp != NULL) *vz_kp = stub_vz_kp;
    if (vz_ki != NULL) *vz_ki = stub_vz_ki;
    if (vz_kd != NULL) *vz_kd = stub_vz_kd;
}

/* ── 外环（H13 新增）：本仿真**刻意把外环关掉** ──────────────────
   本文件验的是 H10 的行为（悬停油门估计器），而 H13 把慢层的误差来源
   从 `0 − vz` 改成了 `vz_sp − vz` —— 只要让 `h_kp = 0`，外环输出就恒等于 0，
   于是整条链路与 H13 之前**逐位等价** ⇒ 本仿真变成一个回归测试：
   "H13 的改动在 h_kp=0 时没有改变任何行为"。 */
static float stub_h_kp = 0.0f;

void PidTable_GetHeightParams(float *h_kp, float *h_ki, float *h_kd)
{
    if (h_kp != NULL) *h_kp = stub_h_kp;
    if (h_ki != NULL) *h_ki = 0.0f;
    if (h_kd != NULL) *h_kd = 0.0f;
}

/* ── 从 motor_test.c / pid_table.c 抄过来的常量（改那边就必须改这里）── */
#define MT_PWM_MIN_US          1000U   /* MOTOR_PWM_MIN_US */
#define MT_SLEW_UP_US_PER_S    800.0f
#define MT_SLEW_DOWN_US_PER_S  300.0f
#define MT_STICK_MAX           1000U
#define PT_MAP_MAX_US          2000U   /* pid_table 的满杆默认值（H10 由 1500 改成它）*/

static const float DT = 0.004f;   /* 250Hz */

static int checks = 0;
static int fails = 0;

static void check(const char *name, bool ok, const char *detail)
{
    checks++;
    if (!ok) fails++;
    printf("[%s] %s%s%s\n", ok ? "OK" : "失败", name, (detail != NULL) ? "  " : "", (detail != NULL) ? detail : "");
}

/* ── 与 motor_test.c 的 MotorTest_MapStickToThrottleUs 等价（满杆 = 2000）── */
static uint16_t manual_map(uint16_t stick)
{
    uint32_t span = PT_MAP_MAX_US - MT_PWM_MIN_US;

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

/* ── 驱动封装：请求模式恒为定高（1）── */
static void step(uint16_t stick, bool armed, float dt)
{
    VerticalControl_Update(1U, stick, true, armed, 0.0f, 0.0f, dt);
}

static VerticalControlSnapshot_t snap(void)
{
    VerticalControlSnapshot_t s;

    VerticalControl_GetSnapshot(&s);
    return s;
}

static void set_est(float height_cm, float vz_cms)
{
    fake_est.valid = true;
    fake_est.height_cm = height_cm;
    fake_est.vz_cms = vz_cms;
}

/* 跑到"控制启用"（st=1）。飞机在台架上，h_est 恒 3cm，靠 h_sp 涨过 4cm 退出停手。 */
static int setup_active(void)
{
    int k;

    VerticalControl_Init();
    set_est(3.0f, 0.0f);
    step(0U, false, DT);                 /* 上锁 + 杆最低 → 切进定高，并直接判在停手 */

    for (k = 0; k < 3000; k++)
    {
        step(760U, true, DT);            /* 解锁 + 推杆 */
        if (snap().state == (uint8_t)VERTICAL_STATE_ALT_HOLD) return k + 1;
    }
    return -1;
}

/* 在"控制启用"状态下匀速移动飞机 n 拍（模拟台架用手匀速抬/放）。 */
static void run_vz(float vz_cms, int frames)
{
    int k;

    for (k = 0; k < frames; k++)
    {
        set_est(3.0f, vz_cms);
        step(760U, true, DT);
    }
}

int main(void)
{
    VerticalControlSnapshot_t s;
    float before;
    float after;
    uint16_t thr_min_seen;
    uint16_t thr_max_seen;
    bool all_i_in_range;
    bool all_corr_in_range;
    bool never_below_1100;
    bool all_thr_in_range;
    int frames;
    int vi;
    int ki_i;
    int combo;
    float ramp;

    printf("========== 0 普通模式映射回归（满杆 1500 -> 2000）==========\n");
    check("杆 0   = 1000（与改动前一致）", manual_map(0U) == 1000U, NULL);
    check("杆 450 = 1450（用户说这里能起飞）", manual_map(450U) == 1450U, NULL);
    check("杆 500 = 1500（中点）", manual_map(500U) == 1500U, NULL);
    check("杆 1000 = 2000（满杆，新值）", manual_map(1000U) == 2000U, NULL);
    {
        char buf[64];
        sprintf(buf, "杆 250 = %u", (unsigned)manual_map(250U));
        check("杆 250 = 1250（一格杆 = 1us，正好等于电调分辨率）", manual_map(250U) == 1250U, buf);
    }

    printf("\n========== 1 普通模式 / 初始化：慢层必须是初值，且不参与 ==========\n");
    VerticalControl_Init();
    set_est(3.0f, 0.0f);
    step(0U, false, DT);
    s = snap();
    check("GetThrottleUs() 在普通模式下给 1000（电机停转）", VerticalControl_GetThrottleUs() == MT_PWM_MIN_US, NULL);
    VerticalControl_Init();
    s = snap();
    check("Init 之后 i 初值 = 1250", (s.vz_throttle_base_us > 1249.9f) && (s.vz_throttle_base_us < 1250.1f), NULL);
    check("Init 之后 corr = 0", s.vz_correction_us == 0.0f, NULL);

    printf("\n========== 2 停手 / 上锁：控制器闭嘴，i 回初值 ==========\n");
    VerticalControl_Init();
    set_est(3.0f, 0.0f);
    step(0U, false, DT);
    s = snap();
    check("st = 4（停手）", s.state == (uint8_t)VERTICAL_STATE_GROUNDED, NULL);
    check("停手时 thr = 1000", VerticalControl_GetThrottleUs() == MT_PWM_MIN_US, NULL);
    check("停手时 corr = 0", s.vz_correction_us == 0.0f, NULL);
    check("停手时 i = 1250", (s.vz_throttle_base_us > 1249.9f) && (s.vz_throttle_base_us < 1250.1f), NULL);

    printf("\n========== 3 进入控制启用（st=1）：油门不跳变 ==========\n");
    frames = setup_active();
    check("能在有限拍内进入 st=1", frames > 0, NULL);
    s = snap();
    check("st = 1", s.state == (uint8_t)VERTICAL_STATE_ALT_HOLD, NULL);
    check("ct 全通 0xF", s.control_gates == VERTICAL_GATE_CT_ALL, NULL);
    check("i 仍是 1250（没有因为进控制而跳变）",
          (s.vz_throttle_base_us > 1249.9f) && (s.vz_throttle_base_us < 1250.1f), NULL);
    check("vz=0 时 corr = 0", s.vz_correction_us == 0.0f, NULL);
    check("GetThrottleUs() = 1250", VerticalControl_GetThrottleUs() == 1250U, NULL);

    printf("\n========== 4 快层符号与限幅（H9 回归）==========\n");
    frames = setup_active();
    (void)frames;
    set_est(3.0f, -20.0f);
    step(760U, true, DT);
    s = snap();
    check("vz = -20（下压）-> corr 顶 +50", s.vz_correction_us > 49.9f, NULL);
    set_est(3.0f, +20.0f);
    step(760U, true, DT);
    s = snap();
    check("vz = +20（上抬）-> corr 顶 -50", s.vz_correction_us < -49.9f, NULL);
    /* ⭐ 线性可分辨：不是"碰巧顶在限幅上所以方向看起来对" */
    stub_vz_kp = 20.0f;
    frames = setup_active();
    (void)frames;
    set_est(3.0f, 0.5f);
    step(760U, true, DT);
    s = snap();
    check("vz = +0.5、Kp=20 -> corr = -10（不是碰巧顶在限幅上）",
          (s.vz_correction_us < -9.5f) && (s.vz_correction_us > -10.5f), NULL);

    printf("\n========== 5 ⭐⭐ 慢层符号（本环节核心）==========\n");
    /* 匀速下放 20cm/s 共 2 秒。ki=5 ⇒ i 以 20*5 = 100us/s 上升 ⇒ 约 +200us。
       从初值 1250 出发、天花板 1650，向上有 400us 余量 ⇒ 能看全斜率。 */
    frames = setup_active();
    (void)frames;
    before = snap().vz_throttle_base_us;
    run_vz(-20.0f, 500);                /* 500 拍 × 4ms = 2.0s */
    after = snap().vz_throttle_base_us;
    {
        char buf[96];
        sprintf(buf, "i: %.1f -> %.1f（变化 %+.1f，期望约 +200）", (double)before, (double)after, (double)(after - before));
        check("匀速下放 2s -> i 上升约 200us（符号正确）",
              (after > before + 190.0f) && (after < before + 210.0f), buf);
    }

    /* ⚠️ 上抬必须**先把 i 充到 1450 再测**：从初值 1250 直接上抬，走 100us 就撞到
       地板 1150，只能看到 -100，那会让人误判成"斜率不对"。
       （这条教训与 H8/H9 仿真踩的是同一个坑：判据没算上钳位。） */
    frames = setup_active();
    (void)frames;
    run_vz(-20.0f, 500);                /* 先充到约 1450（向下 200us 的行程就够看斜率） */
    before = snap().vz_throttle_base_us;
    run_vz(+20.0f, 500);
    after = snap().vz_throttle_base_us;
    {
        char buf[96];
        sprintf(buf, "i: %.1f -> %.1f（变化 %.1f，期望约 -200）", (double)before, (double)after, (double)(after - before));
        check("匀速上抬 2s -> i 下降约 200us（符号正确）",
              (after < before - 190.0f) && (after > before - 210.0f), buf);
    }

    /* 顺带把地板也验掉：从初值 1250 直接上抬 2s（本可以走 -200），必须恰好停在地板 1150。 */
    frames = setup_active();
    (void)frames;
    before = snap().vz_throttle_base_us;
    run_vz(+20.0f, 500);
    after = snap().vz_throttle_base_us;
    {
        char buf[96];
        sprintf(buf, "i: %.1f -> %.1f（只能走 100us 就撞地板 1150）", (double)before, (double)after);
        check("从 1250 上抬 2s 恰好停在地板 1150（地板生效）",
              (after > 1149.9f) && (after < 1150.1f), buf);
    }

    printf("\n========== 6 ⭐⭐ 慢层有界 + 没有 windup ==========\n");
    /* 天花板：长时间大幅下沉，把 i 顶到 1650 */
    frames = setup_active();
    (void)frames;
    run_vz(-100.0f, 5000);              /* 20s */
    s = snap();
    check("长时间下沉 -> i 停在天花板 1650（不越界）",
          (s.vz_throttle_base_us > 1649.9f) && (s.vz_throttle_base_us < 1650.1f), NULL);
    check("此时 thr 被钳在 1650（不是 1650+50=1700）", VerticalControl_GetThrottleUs() == 1650U, NULL);
    /* ⭐ 关键：反向之后必须**立刻**开始往回走，而不是卡在天花板 */
    before = snap().vz_throttle_base_us;
    run_vz(+100.0f, 25);                /* 0.1s */
    after = snap().vz_throttle_base_us;
    check("⭐ 反向 0.1s 后 i 立刻开始下降（证明没有 windup 卡死）", after < before - 1.0f, NULL);

    /* 地板 */
    frames = setup_active();
    (void)frames;
    run_vz(+100.0f, 5000);
    s = snap();
    check("长时间上抬 -> i 停在地板 1150（不越界）",
          (s.vz_throttle_base_us > 1149.9f) && (s.vz_throttle_base_us < 1150.1f), NULL);
    check("此时 thr 被钳在 1150（不是 1150-50=1100）", VerticalControl_GetThrottleUs() == 1150U, NULL);

    printf("\n========== 7 ki = 0 时慢层冻结 ==========\n");
    frames = setup_active();
    (void)frames;
    stub_vz_ki = 0.0f;
    step(760U, true, DT);               /* 让参数变更被读进去 */
    before = snap().vz_throttle_base_us;
    run_vz(20.0f, 500);
    after = snap().vz_throttle_base_us;
    check("ki=0 -> 移动飞机 2s，i 一点不动", (after > before - 0.01f) && (after < before + 0.01f), NULL);
    stub_vz_ki = 5.0f;

    printf("\n========== 8 参数在线变更不会丢掉已学到的 i ==========\n");
    frames = setup_active();
    (void)frames;
    run_vz(-20.0f, 500);                /* 先让 i 走一段，学到一个非初值 */
    before = snap().vz_throttle_base_us;
    check("前置：i 已经离开初值", (before > 1251.0f) || (before < 1249.0f), NULL);
    stub_vz_kp = 40.0f;                 /* 改 Kp：会触发 Pid_ApplyParams（清 PID 内部积分） */
    step(760U, true, DT);
    after = snap().vz_throttle_base_us;
    check("改 vz_kp 后 i 不受影响（ApplyParams 只清快层，不动慢层）",
          (after > before - 20.0f) && (after < before + 20.0f), NULL);
    stub_vz_kp = 20.0f;

    printf("\n========== 9 失效闭嘴：门禁②掉了 / 上锁，i 必须回初值 ==========\n");
    frames = setup_active();
    (void)frames;
    run_vz(-20.0f, 500);
    check("前置：i 已经离开初值", snap().vz_throttle_base_us > 1251.0f, NULL);
    set_est(3.0f, 0.0f);
    step(760U, false, DT);              /* 上锁：门禁②掉 ARM，同时也不再是"未在停手" */
    s = snap();
    check("上锁后 i 回 1250", (s.vz_throttle_base_us > 1249.9f) && (s.vz_throttle_base_us < 1250.1f), NULL);
    check("上锁后 corr = 0", s.vz_correction_us == 0.0f, NULL);
    check("上锁后 GetThrottleUs() = 1000", VerticalControl_GetThrottleUs() == MT_PWM_MIN_US, NULL);

    /* 门禁②的另一位：姿态歪超倾角门禁（当前 30°） */
    frames = setup_active();
    (void)frames;
    run_vz(-20.0f, 500);
    check("前置：i 已经离开初值", snap().vz_throttle_base_us > 1251.0f, NULL);
    set_est(3.0f, 0.0f);
    /* 单独调 Update，把 roll 塞成 30 度 */
    VerticalControl_Update(1U, 760U, true, true, 30.0f, 0.0f, DT);
    s = snap();
    check("姿态歪 30° -> ct 掉 ATT_OK", (s.control_gates & VERTICAL_GATE_CT_ATT_OK) == 0U, NULL);
    /* ⚠️ 判据在 H12 之后**更新过**（不是产品代码回退）：
       H10 时代这里期望"i 回 1250"；但 H12 把"姿态超限"改成了
       **判定炸机 ⇒ 立刻停桨（st=7、fr=2）**，而且失效期间两层是**冻结**的
       —— 不再回初值。H13 之后这里仍然不变（外环也被冻结）。
       ⇒ 新判据：i **保持**在失效前的值，且 thr 立即变成 1000。 */
    check("姿态歪 30° -> st=7、fr=2、thr 立即 1000（H12 起：判定炸机，不再回 1250）",
          (s.state == (uint8_t)VERTICAL_STATE_FAILSAFE_DESCEND) &&
          (s.fail_reason == (uint8_t)VERTICAL_FAIL_ATTITUDE) &&
          (VerticalControl_GetThrottleUs() == 1000U), NULL);
    check("姿态歪 30° -> i **保持**（失效期间冻结，H12 刻意如此）",
          s.vz_throttle_base_us > 1251.0f, NULL);

    printf("\n========== 10 dt 异常时慢层不积分（与 pid.c 同判据）==========\n");
    frames = setup_active();
    (void)frames;
    before = snap().vz_throttle_base_us;
    set_est(3.0f, 20.0f);
    step(760U, true, 0.5f);             /* dt = 500ms，远超 100ms 上限 */
    after = snap().vz_throttle_base_us;
    check("dt=0.5s -> i 不动（避免一个巨大 dt 把积分一次推出去）",
          (after > before - 0.01f) && (after < before + 0.01f), NULL);
    set_est(3.0f, 20.0f);
    step(760U, true, 0.0001f);          /* dt 过小 */
    after = snap().vz_throttle_base_us;
    check("dt=0.1ms -> i 不动", (after > before - 0.01f) && (after < before + 0.01f), NULL);

    printf("\n========== 11 ★ 不变式遍历：thr 永不 <1100、i/corr 永远在许可区间 ==========\n");
    {
        static const float vzs[6] = { -100.0f, -20.0f, -1.0f, 1.0f, 20.0f, 100.0f };
        static const float kis[4] = { 0.0f, 1.0f, 5.0f, 50.0f };
        static const float kps[2] = { 5.0f, 100.0f };

        all_i_in_range = true;
        all_corr_in_range = true;
        never_below_1100 = true;
        all_thr_in_range = true;
        thr_min_seen = 65535U;
        thr_max_seen = 0U;
        combo = 0;

        for (vi = 0; vi < 6; vi++)
        {
            for (ki_i = 0; ki_i < 4; ki_i++)
            {
                int kp_i;
                for (kp_i = 0; kp_i < 2; kp_i++)
                {
                    int f;
                    stub_vz_ki = kis[ki_i];
                    stub_vz_kp = kps[kp_i];
                    frames = setup_active();
                    (void)frames;
                    combo++;
                    for (f = 0; f < 1500; f++)      /* 6 秒 */
                    {
                        uint16_t thr;
                        set_est(3.0f, vzs[vi]);
                        step(760U, true, DT);
                        s = snap();
                        thr = VerticalControl_GetThrottleUs();

                        if ((s.vz_throttle_base_us < 1149.9f) || (s.vz_throttle_base_us > 1650.1f)) all_i_in_range = false;
                        if ((s.vz_correction_us < -50.1f) || (s.vz_correction_us > 50.1f)) all_corr_in_range = false;
                        if (thr < 1101U) never_below_1100 = false;
                        if (s.state == (uint8_t)VERTICAL_STATE_ALT_HOLD)
                        {
                            if ((thr < 1150U) || (thr > 1650U)) all_thr_in_range = false;
                            if (thr < thr_min_seen) thr_min_seen = thr;
                            if (thr > thr_max_seen) thr_max_seen = thr;
                        }
                    }
                    stub_vz_ki = 5.0f;
                    stub_vz_kp = 20.0f;
                }
            }
        }

        {
            char buf[128];
            sprintf(buf, "共 %d 种组合，观察到 thr 范围 %u ~ %u（下限恒 >1100）",
                    combo, (unsigned)thr_min_seen, (unsigned)thr_max_seen);
            check("⭐ thr 恒在 [1150, 1650]", all_thr_in_range, buf);
        }
        check("⭐⭐ 不变式：thr 永不 <= 1100（姿态环永不被旁路）", never_below_1100, NULL);
        check("i 恒在 [1150, 1650]", all_i_in_range, NULL);
        check("corr 恒在 [-50, +50]", all_corr_in_range, NULL);
    }

    printf("\n========== 12 两层之和仍被整体钳住 ==========\n");
    frames = setup_active();
    (void)frames;
    run_vz(-100.0f, 5000);              /* i 到天花板 1650，corr 顶 +50 */
    ramp = (float)VerticalControl_GetThrottleUs();
    check("i=1650 且 corr=+50 -> thr 仍是 1650（不是 1700）", ramp == 1650.0f, NULL);
    frames = setup_active();
    (void)frames;
    run_vz(+100.0f, 5000);              /* i 到地板 1150，corr 顶 -50 */
    ramp = (float)VerticalControl_GetThrottleUs();
    check("i=1150 且 corr=-50 -> thr 仍是 1150（不是 1100）", ramp == 1150.0f, NULL);

    printf("\n========== 13 限速器的过渡值（给真机验收时对照用）==========\n");
    {
        /* 进控制那一刻，油门要经限速器从 1000 爬到 1250：+800us/s × 0.004 = 3.2us/拍
           ⇒ 250us / 3.2 = 约 79 拍。真机上看 thr 会有一串 1000~1250 之间的值，
           **那不是越界**，别误判成失败（H8/H9 的仿真都在这里踩过一次）。 */
        float slew = 1000.0f;
        int n = 0;
        while (slew < 1249.5f) { slew = ramp_limit(slew, 1250.0f, DT); n++; }
        {
            char buf[80];
            sprintf(buf, "1000 -> 1250 需要 %d 拍（约 %.2fs）", n, (double)(n * DT));
            check("限速过渡约 79 拍（真机上别把这串值当越界）", (n >= 70) && (n <= 90), buf);
        }
    }

    printf("\n===== 检查 %d 项，失败 %d 项 =====\n", checks, fails);
    printf("%s\n", (fails == 0) ? "全部通过" : "!! 有失败项，必须先查清再烧录");
    return (fails == 0) ? 0 : 1;
}
