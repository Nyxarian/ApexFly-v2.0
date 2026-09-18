/* H13（外环：高度误差 → 目标垂速）的台架仿真：把验收矩阵在**烧录之前**先跑一遍。
   真实参与编译的只有 vertical_control.c 和 pid.c，其余（估计器、参数表）用桩顶掉。

   编译（宿主机 gcc；本文件不碰任何硬件）：
     gcc -std=c99 -Wall -Wextra -Wno-unused-parameter ^
       -I sim_stub -I "ApexFly v2.0/Modules/Control" -I "ApexFly v2.0/Modules/Motor" ^
       -I "ApexFly v2.0/Modules/Communication" ^
       sim_vc_h13.c "ApexFly v2.0/Modules/Control/vertical_control.c" ^
       "ApexFly v2.0/Modules/Control/pid.c" -o sim_vc_h13.exe

   本环节最要紧的三条，都是"算错了会失控"的：
     ① ⭐ **外环符号**：`h_sp > h_est`（命令上升）必须让 `vz_sp` 变**正**。
        写反就是正反馈（高度偏低反而命令下降）⇒ 越掉越低，与 H9/H10 那两个减号同族。
     ② ⭐ **死区**：`|h_sp − h_est| ≤ 2cm` 时 `vz_sp` 必须**恰好为 0**（硬死区）。
     ③ ⭐⭐ **慢层的误差必须用 `vz_sp` 而不是 0** —— 这一条是 H13 独有的：
        用它才有"推杆 ⇒ 外环要速度 ⇒ 慢层开始学 ⇒ **起飞**"；
        用 0 的话「悬停油门估计」会被系统性拉偏（稳态顶上修正量限幅）。

   ⚠️ 仿真**不能**代替真机验证。它能证明的是：符号、死区、限幅、参数在线变更、
      复位、失效冻结、慢层误差来源、以及各状态下的不变式。
      它证明不了：电机真的转、以及**增益该取多少**（一维模型只能定量级，
      真正的延迟/噪声/姿态耦合要 H14/H15 用黑匣子数据说话）。 */
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
static float stub_vz_kp = 5.0f;    /* H13 起首飞建议值（不是设计值 20） */
static float stub_vz_ki = 5.0f;
static float stub_vz_kd = 0.0f;
static float stub_h_kp = 0.3f;     /* H13 起默认值（不是设计值 0.6） */
static float stub_h_ki = 0.0f;
static float stub_h_kd = 0.0f;

void PidTable_GetAltitudeParams(float *vz_kp, float *vz_ki, float *vz_kd)
{
    if (vz_kp != NULL) *vz_kp = stub_vz_kp;
    if (vz_ki != NULL) *vz_ki = stub_vz_ki;
    if (vz_kd != NULL) *vz_kd = stub_vz_kd;
}

void PidTable_GetHeightParams(float *h_kp, float *h_ki, float *h_kd)
{
    if (h_kp != NULL) *h_kp = stub_h_kp;
    if (h_ki != NULL) *h_ki = stub_h_ki;
    if (h_kd != NULL) *h_kd = stub_h_kd;
}

/* ── 从 vertical_control.h / motor.h 抄过来的常量（改那边就必须改这里）── */
#define VC_THR_MIN      1150.0f
#define VC_THR_MAX      1650.0f
#define MT_PWM_MIN      1000U
#define OUT_DEAD        2.0f
#define OUT_MAX         20.0f
#define I_START         1250.0f

static const float DT = 0.004f;
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

static void tickReq(uint8_t req, uint16_t stick, bool armed, float roll, float pitch, float h, float vz, bool valid)
{
    fake_est.valid = valid;
    fake_est.height_cm = h;
    fake_est.vz_cms = vz;
    VerticalControl_Update(req, stick, true, armed, roll, pitch, DT);
}

/* 让飞机进入"控制启用"（st=1）。地面读数 3cm，解手靠**推杆边沿**（H14 起）：
   上一拍杆 0、这一拍推 1000 ⇒ 第一拍就解手；之后继续推到 nFrames 拍把 h_sp 抬到想要的值。
   （旧版是"h_sp 涨过 4cm 要 334 拍"，那个出口已删除；本函数不用改，它本来就从杆 0 起。） */
static void setupFlying(int extraFrames, float h_fly)
{
    int k;

    VerticalControl_Init();
    tickReq(1U, 0U, false, 0.0f, 0.0f, 3.0f, 0.0f, true);
    for (k = 0; k < 400 + extraFrames; k++)
    {
        tickReq(1U, 1000U, true, 0.0f, 0.0f, h_fly, 0.0f, true);
    }
}

/* 杆回中段（350~650 ⇒ 速率 0 ⇒ h_sp 冻结）n 拍 */
static void center(int n, float h, float vz)
{
    int k;

    for (k = 0; k < n; k++) tickReq(1U, 500U, true, 0.0f, 0.0f, h, vz, true);
}

int main(void)
{
    VerticalControlSnapshot_t s;
    float hSp;
    float before;
    float after;
    int i;
    int j;

    printf("========== 0 普通模式 / 停手：外环必须完全不出力 ==========\n");
    VerticalControl_Init();
    tickReq(0U, 0U, false, 0.0f, 0.0f, 3.0f, 0.0f, true);
    s = snap();
    check("普通模式 st=0", s.state == (uint8_t)VERTICAL_STATE_MANUAL, NULL);
    check("普通模式 vzsp=0", s.vz_sp_cms == 0.0f, NULL);
    VerticalControl_Init();
    tickReq(1U, 0U, false, 0.0f, 0.0f, 3.0f, 0.0f, true);
    s = snap();
    check("停手 st=4 且 vzsp=0（停手期间外环不跑）",
          (s.state == (uint8_t)VERTICAL_STATE_GROUNDED) && (s.vz_sp_cms == 0.0f), NULL);
    check("停手 thr=1000", VerticalControl_GetThrottleUs() == MT_PWM_MIN, NULL);

    printf("\n========== 1 ⭐ 外环公式与符号（本环节第一危险项）==========\n");
    setupFlying(500, 3.0f);                   /* h_sp 被推到约 10cm，h_est=3cm */
    s = snap();
    hSp = s.h_sp_cm;
    {
        char buf[128];
        sprintf(buf, "h_sp=%.2f h_est=%.2f ⇒ vzsp=%+.2f（期望 %+.2f）",
                (double)hSp, (double)s.h_est_cm, (double)s.vz_sp_cms, (double)(stub_h_kp * (hSp - 3.0f)));
        check("⭐ 命令上升（h_sp > h_est）⇒ vzsp **为正**", s.vz_sp_cms > 0.0f, buf);
        check("⭐ 数值 ≈ h_kp × (h_sp − h_est)", (s.vz_sp_cms > (stub_h_kp * (hSp - 3.0f)) - 0.05f) &&
                                                  (s.vz_sp_cms < (stub_h_kp * (hSp - 3.0f)) + 0.05f), buf);
    }
    /* 反向：把飞机"抬高"到远高于目标 */
    center(2, hSp + 20.0f, 0.0f);
    s = snap();
    check("⭐⭐ 命令下降（h_sp < h_est）⇒ vzsp **为负**（写反就是正反馈）", s.vz_sp_cms < 0.0f, NULL);
    {
        char buf[96];
        sprintf(buf, "vzsp=%+.2f（期望 %+.2f）", (double)s.vz_sp_cms, (double)(stub_h_kp * (hSp - (hSp + 20.0f))));
        check("反向数值也对（−20cm 误差 ⇒ −6.0cm/s）",
              (s.vz_sp_cms > (stub_h_kp * (-20.0f)) - 0.05f) && (s.vz_sp_cms < (stub_h_kp * (-20.0f)) + 0.05f), buf);
    }

    printf("\n========== 2 ⭐⭐ 死区（硬死区：界内恰好 0）==========\n");
    setupFlying(500, 3.0f);
    hSp = snap().h_sp_cm;
    center(2, hSp, 0.0f);                     /* 误差 = 0 */
    check("误差 0 ⇒ vzsp = 0", snap().vz_sp_cms == 0.0f, NULL);
    center(2, hSp - 1.9f, 0.0f);              /* 误差 +1.9（界内） */
    check("误差 +1.9cm（界内）⇒ vzsp = 0", snap().vz_sp_cms == 0.0f, NULL);
    center(2, hSp + 1.9f, 0.0f);              /* 误差 −1.9（界内） */
    check("误差 −1.9cm（界内）⇒ vzsp = 0", snap().vz_sp_cms == 0.0f, NULL);
    center(2, hSp - 2.0f, 0.0f);              /* 正好在边界上（含等号 ⇒ 界内） */
    check("误差正好 −2.0cm（边界）⇒ vzsp = 0（判据含等号）", snap().vz_sp_cms == 0.0f, NULL);
    center(2, hSp - 2.5f, 0.0f);              /* 出死区一点点 */
    {
        float want = stub_h_kp * 2.5f;
        char buf[96];
        sprintf(buf, "vzsp=%+.3f（期望 %+.3f = 0.3×2.5）", (double)snap().vz_sp_cms, (double)want);
        check("⭐ 刚出死区（误差 +2.5cm）⇒ vzsp ≈ +0.75cm/s（不是跳变、也不留偏差）",
              (snap().vz_sp_cms > want - 0.05f) && (snap().vz_sp_cms < want + 0.05f), buf);
    }

    printf("\n========== 3 输出限幅 ±20cm/s ==========\n");
    setupFlying(500, 3.0f);
    hSp = snap().h_sp_cm;
    center(2, hSp - 200.0f, 0.0f);
    check("误差 +200cm ⇒ vzsp 顶 +20（不会更大）", snap().vz_sp_cms == OUT_MAX, NULL);
    /* ⚠️ 负方向**不能**用 −200cm：那样 h_est = hSp+200 ≈ 210cm 会先触发**高度越界失效**
          （H12 的 125cm 门槛）⇒ st=6、外环冻结 ⇒ 量到的是冻结值而不是限幅值。
          第一版就是这么误报的 —— 注意这是**产品代码的正确行为**，不是 bug。
          ⇒ 用 h_est = 120cm（≤125，不触发）把误差做到 −109cm，足够顶到 −20。 */
    center(2, 120.0f, 0.0f);
    s = snap();
    {
        char buf[128];
        sprintf(buf, "误差 %.1fcm ⇒ vzsp=%+.2f（st=%u，应为 1 且 vzsp=−20）",
                (double)(hSp - 120.0f), (double)s.vz_sp_cms, (unsigned)s.state);
        check("误差 −109cm ⇒ vzsp 顶 −20（且没误触高度失效）",
              (s.vz_sp_cms == -OUT_MAX) && (s.state == (uint8_t)VERTICAL_STATE_ALT_HOLD), buf);
    }

    printf("\n========== 4 参数在线变更（遥控可调，ID 30~32）==========\n");
    setupFlying(500, 3.0f);
    hSp = snap().h_sp_cm;
    center(2, hSp - 10.0f, 0.0f);
    before = snap().vz_sp_cms;
    stub_h_kp = 0.6f;                          /* 0.3 → 0.6 */
    center(2, hSp - 10.0f, 0.0f);
    after = snap().vz_sp_cms;
    {
        char buf[96];
        sprintf(buf, "h_kp 0.3→0.6：vzsp %.2f → %.2f（应约翻倍）", (double)before, (double)after);
        check("⭐ 改 h_kp ⇒ vzsp 跟着变（参数是活的）",
              (after > before * 1.9f) && (after < before * 2.1f), buf);
    }
    stub_h_kp = 0.0f;                          /* 外环"关闭" */
    center(2, hSp - 10.0f, 0.0f);
    check("h_kp = 0 ⇒ vzsp 恒 0（外环关闭，只剩内环阻尼）", snap().vz_sp_cms == 0.0f, NULL);
    stub_h_kp = 0.3f;

    printf("\n========== 5 复位与失效：外环必须冻结 / 清零 ==========\n");
    /* 5a：Reset（模式切换 / IMU 重初始化会调它）⇒ vzsp 清 0 */
    setupFlying(500, 3.0f);
    hSp = snap().h_sp_cm;
    center(2, hSp - 10.0f, 0.0f);
    check("前置：外环正在出力（vzsp > 0）", snap().vz_sp_cms > 0.0f, NULL);
    VerticalControl_Reset();
    s = snap();
    {
        char buf[128];
        sprintf(buf, "Reset 后 h_sp=%.2f h_est=%.2f vzsp=%+.2f", (double)s.h_sp_cm, (double)s.h_est_cm, (double)s.vz_sp_cms);
        check("⭐ Reset() ⇒ vzsp 归 0 且 h_sp 对齐到 h_est（\"切换无扰\"就靠这个）",
              (s.vz_sp_cms == 0.0f) && (s.h_sp_cm == s.h_est_cm), buf);
    }

    /* 5b：失效期间外环**冻结** —— vzsp 保持失效前的值，既不跳也不清 */
    setupFlying(500, 3.0f);
    hSp = snap().h_sp_cm;
    center(2, hSp - 10.0f, 0.0f);
    before = snap().vz_sp_cms;
    check("前置：vzsp > 0", before > 0.0f, NULL);
    tickReq(1U, 1000U, true, 0.0f, 0.0f, 50.0f, 0.0f, false);     /* ToF 失效 ⇒ st=6 */
    check("失效进入 st=6", snap().state == (uint8_t)VERTICAL_STATE_FAILSAFE_HOLD, NULL);
    for (i = 0; i < 300; i++) tickReq(1U, 1000U, true, 0.0f, 0.0f, 50.0f, 0.0f, false);
    after = snap().vz_sp_cms;
    {
        char buf[96];
        sprintf(buf, "失效前 %+.2f → 保持中 %+.2f", (double)before, (double)after);
        check("⭐ 失效期间 vzsp **冻结**（两层都不动：既不跳、也不清成 0）", after == before, buf);
    }
    /* 5c：恢复 ⇒ 外环立刻接着跑（无缝） */
    center(2, hSp - 10.0f, 0.0f);
    s = snap();
    check("恢复后 st=1 且外环重新出力", (s.state == (uint8_t)VERTICAL_STATE_ALT_HOLD) && (s.vz_sp_cms > 0.0f), NULL);
    /* 5d：上锁 ⇒ 清 0 */
    tickReq(1U, 1000U, false, 0.0f, 0.0f, 3.0f, 0.0f, true);
    check("上锁 ⇒ vzsp = 0", snap().vz_sp_cms == 0.0f, NULL);

    printf("\n========== 6 ⭐⭐ 慢层的误差来源（H13 独有的一处改动）==========\n");
    /* 6a：起飞助力 —— 推杆后 h_sp 远高于 h_est ⇒ vzsp>0 ⇒ **i 必须开始涨**
            （这就是"地面上憋油门 5~6 秒然后起飞"的来源；用 0 当目标的话 i 只会降） */
    setupFlying(400, 3.0f);                    /* h_sp ≈ 9.6cm、h_est = 3.0cm */
    s = snap();
    check("前置：推杆把 h_sp 抬到约 9.6cm，vzsp > 0", (s.h_sp_cm > 9.0f) && (s.vz_sp_cms > 0.0f), NULL);
    before = s.vz_throttle_base_us;
    center(500, 3.0f, 0.0f);                   /* 杆回中段 ⇒ h_sp 冻结；h_est 还是 3cm ⇒ 误差 6.6cm */
    after = snap().vz_throttle_base_us;
    s = snap();
    {
        float want = 500.0f * stub_vz_ki * stub_h_kp * (s.h_sp_cm - 3.0f) * DT;
        char buf[144];
        sprintf(buf, "vzsp=%+.2f；2 秒内 i: %.0f → %.0f（+%.0f，期望约 +%.0f）",
                (double)s.vz_sp_cms, (double)before, (double)after, (double)(after - before), (double)want);
        check("⭐⭐ 命令爬升 ⇒ vzsp>0 ⇒ **i 上涨**（= 起飞前憋油门）",
              (s.vz_sp_cms > 0.0f) && (after > before + want * 0.7f) && (after < before + want * 1.3f), buf);
    }
    /* 6b：H10 的判据必须仍然成立 —— 在死区内（vzsp≈0）匀速上抬 ⇒ i 仍然下降 */
    setupFlying(500, 3.0f);
    hSp = snap().h_sp_cm;
    center(200, hSp, 0.0f);                    /* h_est = h_sp ⇒ 误差 0 ⇒ vzsp = 0 */
    before = snap().vz_throttle_base_us;
    /* ⚠️ 只跑 200 拍（预期 −80µs）而不是 500 拍：500 拍要走 −200µs，
          而 `i` 的地板是 1150、起点约 1264 ⇒ **会先撞地板**，
          量到的就是 −114 而不是 −200。第一版就是这么误报的 ——
          与 H10/H12 仿真里踩的是同一个坑：**判据必须先问"这段行程够不够"**。 */
    center(200, hSp, 20.0f);                   /* 匀速"上抬"：vz = +20cm/s */
    after = snap().vz_throttle_base_us;
    s = snap();
    {
        float want = -200.0f * stub_vz_ki * 20.0f * DT;    /* = -80µs */
        char buf[160];
        sprintf(buf, "vzsp=%+.2f、vz=+20、跑 200 拍 ⇒ i: %.0f → %.0f（%.0f，期望约 %.0f）",
                (double)s.vz_sp_cms, (double)before, (double)after, (double)(after - before), (double)want);
        check("⭐ H10 回归：vzsp≈0 时匀速上抬 ⇒ **i 仍然下降**（斜率正确）",
              (after < before + want * 0.7f) && (after > before + want * 1.3f) && (after > 1150.0f), buf);
    }
    /* 6c：反向（匀速下沉、vzsp≈0）⇒ i 上升 —— 与 6b 一起把符号钉死 */
    setupFlying(500, 3.0f);
    hSp = snap().h_sp_cm;
    center(200, hSp, 0.0f);
    before = snap().vz_throttle_base_us;
    center(200, hSp, -20.0f);
    after = snap().vz_throttle_base_us;
    {
        float want = 200.0f * stub_vz_ki * 20.0f * DT;     /* = +80µs */
        char buf[160];
        sprintf(buf, "vzsp=%+.2f、vz=-20、跑 200 拍 ⇒ i: %.0f → %.0f（%+.0f，期望约 %+.0f）",
                (double)snap().vz_sp_cms, (double)before, (double)after, (double)(after - before), (double)want);
        check("⭐ H10 回归：vzsp≈0 时匀速下沉 ⇒ **i 上升**（斜率正确）",
              (after > before + want * 0.7f) && (after < before + want * 1.3f), buf);
    }
    printf("      6a/6b/6c 三条一起说明：慢层用的是 `vz_sp − vz` 而不是 `0 − vz` ——\n");
    printf("      用 0 的话 6a 里 i 只会下降，**飞机永远起飞不了**（H13 的核心动机）。\n");

    printf("\n========== 7 不变式遍历 ==========\n");
    {
        static const float hErrs[7] = { -200.0f, -10.0f, -2.5f, 0.0f, 2.5f, 10.0f, 200.0f };
        static const bool arms[2] = { true, false };
        static const bool valids[2] = { true, false };
        int combos = 0, signBad = 0, deadBad = 0, rangeBad = 0, zeroBad = 0, thrBad = 0, stBad = 0;

        for (i = 0; i < 7; i++)
        {
            for (j = 0; j < 2; j++)
            {
                int v;
                for (v = 0; v < 2; v++)
                {
                    int k;
                    float hEst;

                    combos++;
                    setupFlying(500, 3.0f);
                    hSp = snap().h_sp_cm;
                    hEst = hSp - hErrs[i];
                    for (k = 0; k < 3; k++)
                    {
                        tickReq(1U, 500U, arms[j], 0.0f, 0.0f, hEst, 0.0f, valids[v]);
                    }
                    s = snap();
                    {
                        float e = hSp - hEst;
                        float vsp = s.vz_sp_cms;
                        uint16_t thr = VerticalControl_GetThrottleUs();

                        if ((s.state != 0U) && (s.state != 1U) && (s.state != 4U) && (s.state != 5U) &&
                            (s.state != 6U) && (s.state != 7U)) stBad++;
                        if ((vsp < -OUT_MAX - 0.01f) || (vsp > OUT_MAX + 0.01f)) rangeBad++;
                        if ((thr < MT_PWM_MIN) || (thr > (uint16_t)VC_THR_MAX)) thrBad++;
                        if (s.state == 1U)
                        {
                            /* 死区：界内必须恰好 0 */
                            if ((e <= OUT_DEAD) && (e >= -OUT_DEAD) && (vsp != 0.0f)) deadBad++;
                            /* 出死区：符号必须与误差同号 */
                            if (e > OUT_DEAD && vsp <= 0.0f) signBad++;
                            if (e < -OUT_DEAD && vsp >= 0.0f) signBad++;
                        }
                        else if (s.state != 6U && s.state != 7U)
                        {
                            /* 普通 / 停手 / 未解锁 / 门禁没过：外环必须输出 0 */
                            if (vsp != 0.0f) zeroBad++;
                        }
                    }
                }
            }
        }
        {
            char buf[96];
            sprintf(buf, "%d 种组合（误差 ±200/±10/±2.5/0 × 解锁 × 读数可信）", combos);
            check("★ st 恒在 {0,1,4,5,6,7}", stBad == 0, buf);
        }
        check("★ vzsp 恒在 [−20, +20]", rangeBad == 0, NULL);
        check("★ st=1 且误差在死区内 ⇒ vzsp 恰好 0", deadBad == 0, NULL);
        check("⭐ st=1 且误差出死区 ⇒ vzsp 符号与误差一致", signBad == 0, NULL);
        check("★ 非 st=1/6/7（普通/停手/未解锁/门禁没过）⇒ vzsp 恒 0", zeroBad == 0, NULL);
        check("★ thr 恒在 [1000, 1650]", thrBad == 0, NULL);
    }

    printf("\n========== 8 与真机台架验收的对照量（抄这些数去比对）==========\n");
    {
        setupFlying(500, 3.0f);
        s = snap();
        printf("  进入 st=1 后推杆到底约 2 秒：h_sp = %.1fcm、h_est = 3.0cm\n", (double)s.h_sp_cm);
        printf("     ⇒ vzsp = %+.2f cm/s（期望 0.3 × %.1f = %.2f）\n",
               (double)s.vz_sp_cms, (double)(s.h_sp_cm - 3.0f), (double)(0.3f * (s.h_sp_cm - 3.0f)));
        printf("     ⇒ 此时 i = %.0f、thr = %u\n", (double)s.vz_throttle_base_us, (unsigned)VerticalControl_GetThrottleUs());
        printf("  死区边界：误差 2.0cm ⇒ vzsp=0；2.5cm ⇒ vzsp=+0.75；10cm ⇒ +3.00；20cm ⇒ +6.00\n");
        printf("  ⚠️ 真机上「推杆 ⇒ i 上涨 ⇒ 约 5~6 秒后离地」是**预期现象**（见 sim_h13_takeoff.js）\n");
    }

    printf("\n===== 检查 %d 项，失败 %d 项 =====\n", checks, fails);
    printf("%s\n", (fails == 0) ? "全部通过" : "!! 有失败项，必须先查清再烧录");
    return (fails == 0) ? 0 : 1;
}
