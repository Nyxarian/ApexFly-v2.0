/**
 * @file vertical_control.c
 * @brief 定高模式的状态机、两层门禁、摇杆映射、基础油门与垂速内环实现。
 *
 * ── 本文件做什么、不做什么 ────────────────────────────────────
 * 做：模式切换门禁（门禁①）、控制启用门禁（门禁②）、油门杆三段映射、
 *     h_sp 积分、停手（地面待机）状态、定高模式下的基础油门（H8 起）、
 *     **垂速内环 P**（H9 起）。
 * 不做：不写电机、不打印、不做外环（H13）、不做内环积分（H10）、不做退出判据（H12）。
 *
 * ── 垂速内环（H9）───────────────────────────────────────────
 * 控制律 ∆µs = Kp_v × (vz_sp − vz_est)，本阶段只有 P（Ki/Kd 走参数表，默认 0）。
 * `vz_sp` **固定 0**（VERTICAL_INNER_VZ_SP_CMS），含义是"把垂速压到 0"：
 *   飞机被抬起来（vz>0）→ 减油门；被压下去（vz<0）→ 加油门。一个纯阻尼器。
 *
 * ⚠️⚠️ **符号是这一环唯一真正危险的东西，而危险恰恰在于它"看起来显然"。**
 *    回路上有两个半截，两个**本来就都是正的**：
 *      油门↑ → 推力↑ → 向上加速度↑（执行器，`mixer.h` 已实测确认四路极性）
 *      真实 vz↑ → vz_est↑（估计器，H3 已验"匀速抬手符号正确"）
 *    所以负反馈所需要的**唯一一个反号**，就是 `pid.c` 里那一句
 *    `error = setpoint − measured`。setpoint=0 时它自动给出 −vz_est。
 *    ⇒ 参数顺序写反（`Pid_Update(&pid, vz_est, 0.0f, dt)`）就是正反馈，
 *      现象是"飞机被抬起来就一直往上飞"。台架验收的 ⑤⑥ 两条专门抓这个，
 *      而这是**唯一会真正导致失控**的错误，所以它排在整个环节的第一位。
 *
 * ── 基础油门为什么要由本模块出（H8）────────────────────────────
 * 定高模式下油门杆已经被"三段映射 → h_sp"占用了。若不接管基础油门，
 * 同一个杆会同时命令"爬升"（本模块）和"加大油门"（motor_test 里的手动映射），
 * 两套逻辑抢同一个输入。接管之后基础油门只由本模块决定。
 *
 * ── 两层门禁为什么要拆开 ──────────────────────────────────────
 * 模式切换发生在**地面（已上锁）**，而控制只在天上（已解锁）才需要。
 * 混成一层会自相矛盾——"要求已解锁才能切换"和"切换必须在地面"打架。
 *
 * ── 每拍的执行顺序（就是下面 Update 里的顺序，不要重排）────────
 *   ⓪ 刷新内环参数与限幅（H9）
 *   → ① 读估计器 → ② 填杆量与请求 → ③ 算门禁① → ④ 切换（可能复位）
 *   → ⑤ 积分 h_sp → ⑥ 停手判定 → ⑦ 算门禁② → ⑧ 定状态
 *   → ⑨ 垂速内环（H9） → ⑩ 遥测字节
 * ⑤ 必须在 ④ 之后：切换时复位出来的 h_sp 是这一拍积分的起点。
 * ⑦ 必须在 ⑥ 之后：门禁②里有一位就是"不处于停手"。
 * ⑨ 必须在 ⑧ 之后：内环只在"门禁②全通且不在停手"时才动，而那两件事正是 ⑥⑦⑧ 算出来的。
 */

#include "vertical_control.h"

/* 基础油门的两个取值来源。方向是 Control → Motor/Mixer，和 motor_test.c 引用本模块
   的方向相反但不冲突：这里只需要两个常量，不需要任何电机状态。
   pid / pid_table 同属 Control 层，方向一致，不引入新的跨层依赖。 */
#include "mixer.h"
#include "motor.h"
#include "pid.h"
#include "pid_table.h"
#include "vertical_est.h"
#include <string.h>

/* 整个模块的状态。只在控制环里写，打印/遥测那边读。
   读写不是原子的（结构体拷贝），打印出来的字段理论上可能来自相邻两拍。
   和 vertical_est 是同一个已知局限，对诊断没有影响。 */
static VerticalControlSnapshot_t vertical_snapshot;

/* 【临时，H6 验证完删除】串口命令覆盖的请求模式，0xFF 表示不覆盖。
   写方是黑匣子任务的串口解析，读方是控制环；单字节读写天然原子，不需要临界区。 */
static volatile uint8_t vertical_debug_requested_mode = 0xFFU;

/* 起飞保护：退出停手时置位，用来阻止"刚推杆又被按回去"的死循环。
   详见 vertical_control.h 里 VERTICAL_TAKEOFF_GUARD_CLEAR_H_CM 的说明。 */
static bool vertical_takeoff_guard;

/* ── 垂速内环（H9）────────────────────────────────────────────
   只有一份 Pid_t，**不是** CascadePid_t：本阶段只有内环，外环在 H13。
   不先用 CascadePid_t 占位，是因为 CascadePid_UpdateInner 依赖 UpdateOuter
   写进去的 inner_setpoint —— 直接调它很容易让后来的人以为外环已经在跑了。
   （而且 H13 的外环有 2cm 死区和 ±20cm/s 钳位，CascadePid 装不下，届时本来就要重排。） */
static Pid_t vertical_inner_pid;

/* 参数缓存：每拍读参数表，**只有真的变了才 Pid_ApplyParams**。
   为什么要缓存：Pid_ApplyParams 内部会清积分。H10 打开 Ki 之后，
   每拍无脑 Apply 会把积分器永远按在 0 上（= 悬停油门学习器直接失效）。
   ⚠️ 满杆油门上限也必须进缓存：它决定动态限幅，用户只改它而不改三个增益时，
      只比较增益会漏掉"限幅该重算"这件事。
   ⚠️ 遥控在解锁中禁止改参（PidTable_HandleCommand 的既有约束），
      所以这个"变了"只会在台架上发生，空中不会因为调参而清积分。 */
static float vertical_inner_kp;
static float vertical_inner_ki;
static float vertical_inner_kd;
static uint16_t vertical_inner_max_throttle_us;
static bool vertical_inner_params_valid;

/**
 * @brief 自己写绝对值，为了不引 math.h（和 motor_test.c 里的做法一致，省 flash）。
 */
static float VerticalControl_Absf(float value)
{
    return (value < 0.0f) ? -value : value;
}

/**
 * @brief 把油门杆映射成目标高度的变化率，单位厘米每秒，向上为正。
 *
 * 三段式，段与段之间速率连续（350 和 650 处都是 0）：
 *     杆 ≤ 350         下降段，0 → −2.0cm/s（杆越低降得越快）
 *     350 < 杆 < 650   保持段，恒 0（h_sp 冻结，松手就稳住）
 *     杆 ≥ 650         上升段，0 → +3.0cm/s（杆越高升得越快）
 *
 * 为什么是"杆给变化率"而不是"杆给高度"：高度无界，没法像角度那样一一映射到杆的行程上，
 * 而"杆=速度"能自然做到中位就是 0。代价是多了一层积分（见方案的逐级对照表）。
 */
static float VerticalControl_StickToRateCms(uint16_t stick)
{
    float span;

    if (stick <= VERTICAL_STICK_HOLD_LOW)
    {
        /* 杆越低降得越快：350 → 0，0 → −2.0。 */
        span = (float)(VERTICAL_STICK_HOLD_LOW - stick);
        return -(VERTICAL_RATE_DOWN_CMS * span) / (float)VERTICAL_STICK_HOLD_LOW;
    }

    if (stick >= VERTICAL_STICK_HOLD_HIGH)
    {
        /* 杆越高升得越快：650 → 0，1000 → +3.0。 */
        span = (float)(stick - VERTICAL_STICK_HOLD_HIGH);
        return (VERTICAL_RATE_UP_CMS * span) / (float)(VERTICAL_STICK_MAX - VERTICAL_STICK_HOLD_HIGH);
    }

    return 0.0f;
}

/**
 * @brief 算门禁①（模式切换）的通过位。
 * @note 用到快照里的 stick / est_valid / h_est_cm，调用前必须已经填好。
 */
static uint8_t VerticalControl_EvalSwitchGates(bool link_ok, bool armed)
{
    uint8_t gates = 0U;

    /* "在地面上" = 链路正常 + 已上锁。上锁意味着电机停转，
       是唯一不依赖高度读数就能 100% 确定飞机在地面的判据（地面读数只有 8mm，拿高度判不可靠）。
       ⚠️ 必须同时看 link_ok：失联时 armed 也会变成 false，
       只看 armed 会把"空中失联"误判成"在地面上"，从而在半空中把模式切了。 */
    if (link_ok && !armed)
    {
        gates |= VERTICAL_GATE_SW_ON_GROUND;
    }

    /* 杆在最低位：保证切进去之后不会立刻收到爬升/下降命令，
       而且和普通模式"杆收到底"的状态一致，切换天然无扰。
       写 ≤50 而不是 =0，是为了容忍摇杆的机械与校准余量。 */
    if (vertical_snapshot.stick <= VERTICAL_SWITCH_STICK_MAX)
    {
        gates |= VERTICAL_GATE_SW_STICK_LOW;
    }

    /* 读数可信：复用估计器那套 300ms 去抖的判据，比直接看 ToF 的 fresh 稳。 */
    if (vertical_snapshot.est_valid)
    {
        gates |= VERTICAL_GATE_SW_EST_VALID;
    }

    /* 高度不超上限。下限故意不设："在地面上就切进定高"是正常用法，而地面 h_est≈0.8cm。
       注意这一条同时要求 est_valid——读数不可信时 h_est 是冻结的旧值，不能拿它做判断。 */
    if (vertical_snapshot.est_valid && (vertical_snapshot.h_est_cm <= VERTICAL_MODE_MAX_HEIGHT_CM))
    {
        gates |= VERTICAL_GATE_SW_HEIGHT_OK;
    }

    return gates;
}

/**
 * @brief 算门禁②（控制启用）的通过位。
 * @param grounded 本拍停手判定的结果，必须是**已经算完**的值。
 */
static uint8_t VerticalControl_EvalControlGates(bool armed, float roll_deg, float pitch_deg, bool grounded)
{
    uint8_t gates = 0U;

    if (armed)
    {
        gates |= VERTICAL_GATE_CT_ARMED;
    }

    if (vertical_snapshot.est_valid)
    {
        gates |= VERTICAL_GATE_CT_EST_VALID;
    }

    /* 姿态要平稳：歪太多 cosθ 倾斜补偿就不成立，而且这时飞行员在手动救飞机，
       不该让定高插进来。 */
    if ((VerticalControl_Absf(roll_deg) < VERTICAL_CONTROL_MAX_TILT_DEG) && (VerticalControl_Absf(pitch_deg) < VERTICAL_CONTROL_MAX_TILT_DEG))
    {
        gates |= VERTICAL_GATE_CT_ATT_OK;
    }

    if (!grounded)
    {
        gates |= VERTICAL_GATE_CT_NOT_GROUND;
    }

    return gates;
}

/**
 * @brief 控制器是否真的在干活（H9）。
 *
 * 判据就是"处于定高模式 + 不在停手 + 门禁②全通"，也就是 `state == ALT_HOLD`。
 * ⚠️ 抽成一个函数，是为了让"算修正量"（Update 的 ⑨）和"给基础油门"（GetThrottleUs）
 *    用**同一个判据**。两处各写一遍迟早会不一致，那时会出现
 *    "修正量在算、但基础油门给的是 1000"这种自相矛盾的状态。
 */
static bool VerticalControl_IsActive(void)
{
    return (vertical_snapshot.mode == (uint8_t)VERTICAL_MODE_ALT_HOLD) &&
           (!vertical_snapshot.grounded) &&
           (vertical_snapshot.control_gates == VERTICAL_GATE_CT_ALL);
}

/**
 * @brief 每拍刷新内环参数与动态限幅（H9），参数没变就直接返回。
 *
 * ── 为什么限幅是"动态"的 ──────────────────────────────────────
 * 修正量的可用范围取决于**当前基准油门**和**满杆油门上限**，两者都是遥控可改的参数：
 *     向上：满杆上限 − 基准 − 姿态保留余量
 *           （混控在基础油门贴上限时会统一缩放三轴修正，被牺牲的永远是姿态）
 *     向下：混控有效下限 − 基准
 *           （**绝不能碰到 1100**：那会让 motor_test 的四路等值分支命中，
 *             姿态环被旁路并 AttitudeControl_Reset()）
 * 算出来的两个值再与硬限幅 VERTICAL_INNER_CORR_LIMIT_US 取小。
 *
 * 参数表默认（满杆 1500）时：(1500 − 1250 − 100) = +150 → 取小 → **+30**
 *                            (1100 − 1250)       = −150 → 取小 → **−30**
 * ⇒ 对称的 ±30，正好让 H9 的"该减油门时减"也验得了（这正是基准必须 > 1100 的原因）。
 */
static void VerticalControl_RefreshInnerParams(void)
{
    float kp;
    float ki;
    float kd;
    float out_max;
    float out_min;
    uint16_t max_throttle_us;

    PidTable_GetAltitudeParams(&kp, &ki, &kd);
    max_throttle_us = PidTable_GetMaxThrottleUs();

    if (vertical_inner_params_valid &&
        (kp == vertical_inner_kp) && (ki == vertical_inner_ki) && (kd == vertical_inner_kd) &&
        (max_throttle_us == vertical_inner_max_throttle_us))
    {
        return;
    }

    out_max = (float)max_throttle_us - (float)VERTICAL_BASE_PWM_US - VERTICAL_ATTITUDE_RESERVE_US;
    if (out_max > VERTICAL_INNER_CORR_LIMIT_US) out_max = VERTICAL_INNER_CORR_LIMIT_US;
    /* 满杆上限低于"基准 + 余量"时**不给**向上的权限（0），而不是给一个负的上限：
       负的上限会让 Pid_Update 的夹取逻辑反过来把输出推到负值去。 */
    if (out_max < 0.0f) out_max = 0.0f;

    out_min = (float)MIXER_PWM_EFFECTIVE_MIN_US - (float)VERTICAL_BASE_PWM_US;
    if (out_min > 0.0f) out_min = 0.0f;
    if (out_min < -VERTICAL_INNER_CORR_LIMIT_US) out_min = -VERTICAL_INNER_CORR_LIMIT_US;

    /* 积分限幅按**向上余量**的比例推导，沿用 attitude_control 的既有约定
       （"改输出限幅连带改积分限幅"是刻意避免的，见 attitude_control.h:41-47）。
       用向上余量而不是对称限幅：积分是用来补推力的，真正的约束在向上那一侧。
       H9 阶段 Ki=0，本值暂不起作用；放在这里是为了 H10 不用再动限幅语义。 */
    Pid_ApplyParams(&vertical_inner_pid, kp, ki, kd, out_max * VERTICAL_INNER_I_LIMIT_RATIO, out_max, out_min);

    vertical_inner_kp = kp;
    vertical_inner_ki = ki;
    vertical_inner_kd = kd;
    vertical_inner_max_throttle_us = max_throttle_us;
    vertical_inner_params_valid = true;
}

void VerticalControl_Init(void)
{
    memset(&vertical_snapshot, 0, sizeof(vertical_snapshot));
    vertical_snapshot.mode = (uint8_t)VERTICAL_MODE_MANUAL;
    vertical_snapshot.state = (uint8_t)VERTICAL_STATE_MANUAL;
    vertical_snapshot.h_sp_cm = VERTICAL_H_SP_MIN_CM;
    vertical_takeoff_guard = false;

    /* 内环先给一组"输出恒 0"的合法初值，再立刻去读真实参数。
       万一读的时候参数表还没初始化（现在不会：motor_test.c 里 PidTable_Init 在前），
       这一组零值让内环表现成"什么都不做"，而不是拿未初始化的限幅去驱动电机。
       参数表就绪后，下一拍 RefreshInnerParams 会因为缓存不匹配而自动重新应用。 */
    Pid_Init(&vertical_inner_pid, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    vertical_inner_max_throttle_us = 0U;
    vertical_inner_params_valid = false;
    VerticalControl_RefreshInnerParams();

    /* 调试覆盖不在这里清：那是台架手段，不该被 IMU 重初始化顺手清掉。 */
}

void VerticalControl_Reset(void)
{
    /* 设定值对齐到当前值。估计还没起步时（上电第一次）用下限 0。 */
    vertical_snapshot.h_sp_cm = vertical_snapshot.est_valid ? vertical_snapshot.h_est_cm : VERTICAL_H_SP_MIN_CM;
    vertical_snapshot.grounded = false;
    vertical_snapshot.ground_timer_ms = 0U;
    /* 模式刚在地面上切进来，飞行员不可能是"正在起飞"，保护要清掉，
       否则紧接着的停手判定会被它挡住。 */
    vertical_takeoff_guard = false;
    /* 内环也要复位：模式切换是"从零开始"，积分和微分历史都不能留
       （H10 打开 Ki 之后，留着旧积分等于带着上一轮的悬停油门估计值进新一次飞行）。
       ⚠️ 只清运行状态，不重读参数——参数没变就没必要 ApplyParams。 */
    Pid_Reset(&vertical_inner_pid);
    vertical_snapshot.vz_sp_cms = 0.0f;
    vertical_snapshot.vz_correction_us = 0.0f;
    /* mode 不动：本函数也会在"模式切换成功"时被调用，那时新模式已经写进去了。
       杆量、门禁位图每拍都会被覆盖，不需要清。 */
}

void VerticalControl_Update(uint8_t requested_mode, uint16_t throttle_stick, bool link_ok, bool armed, float roll_deg, float pitch_deg, float dt_s)
{
    VerticalEstState_t est;
    uint32_t elapsed_ms;

    /* ⓪ 参数与限幅（H9）。放在最前面：本拍所有用到内环的地方都要用最新的限幅。
        内部有缓存比对，参数没变时只是一次临界区读 + 几次整数/浮点比较。 */
    VerticalControl_RefreshInnerParams();

    /* ① 读估计器。valid 为 false 时 h/vz 是"最后一次可信值"而不是当前值，
          所以下面每一处拿高度做判断的地方都要先看 est.valid。 */
    VerticalEst_GetState(&est);
    vertical_snapshot.est_valid = est.valid;
    vertical_snapshot.h_est_cm = est.height_cm;
    /* 垂速原样转抄给快照，供打印。它和内环输出的符号关系就是 H9 的验收核心，
       转抄一份比让打印那边再跨模块取一次要少一条耦合。 */
    vertical_snapshot.vz_est_cms = est.vz_cms;

    /* ② 杆量按协议量程收一下，顺手填进快照——门禁①要用它。 */
    vertical_snapshot.stick = (throttle_stick > VERTICAL_STICK_MAX) ? VERTICAL_STICK_MAX : throttle_stick;

    /* 【临时，H6 验证完删除】串口 `vm 0/1` 覆盖遥控请求。 */
    if (vertical_debug_requested_mode != 0xFFU)
    {
        requested_mode = vertical_debug_requested_mode;
    }
    vertical_snapshot.requested_mode = requested_mode;

    /* ③ 门禁① */
    vertical_snapshot.switch_gates = VerticalControl_EvalSwitchGates(link_ok, armed);

    /* ④ 切换。三个条件缺一不可：门禁①全通、请求与实际不同、请求值是我们认识的两个模式之一。
          不认识的值（2/3 或乱码）只拒绝、不改模式——宁可不动，也不猜遥控想干什么。
          ⚠️ 用"请求 vs 实际"来比较，效果就是"模式一旦进入，要退出必须再满足一次门禁①"，
          这正是"一次飞行只能用一个模式"的实现方式，不要改成"让模式跟着请求走"。 */
    if ((vertical_snapshot.switch_gates == VERTICAL_GATE_SW_ALL) && (requested_mode != vertical_snapshot.mode) && ((requested_mode == (uint8_t)VERTICAL_MODE_MANUAL) || (requested_mode == (uint8_t)VERTICAL_MODE_ALT_HOLD)))
    {
        vertical_snapshot.mode = requested_mode;
        VerticalControl_Reset();

        /* 模式是**在地面上**切进来的（门禁①已经保证了上锁）。
           如果此刻读数也在停手门槛内，直接把停手计时置满，省掉 500ms 去抖。
           不这么做会留下一个 500ms 的窗口，让控制器以为自己该干活：
           等 H8 接管油门之后，那 500ms 里会把油门压到 1100µs 并触发失效判据，
           表现成"定高在地面上自己退出"。 */
        if (vertical_snapshot.est_valid && (vertical_snapshot.h_est_cm <= VERTICAL_GROUND_ENTER_H_CM))
        {
            vertical_snapshot.ground_timer_ms = VERTICAL_GROUND_ENTER_HOLD_MS;
        }
    }

    /* ⑤ 积分。⚠️ 停手期间这一句也必须跑：h_sp 是停手唯一的出口，
          把它停掉，飞机落地之后就再也起飞不了了（方案 §15.3）。 */
    vertical_snapshot.stick_rate_cms = VerticalControl_StickToRateCms(vertical_snapshot.stick);
    if (dt_s > 0.0f)
    {
        vertical_snapshot.h_sp_cm += vertical_snapshot.stick_rate_cms * dt_s;
    }
    if (vertical_snapshot.h_sp_cm < VERTICAL_H_SP_MIN_CM) vertical_snapshot.h_sp_cm = VERTICAL_H_SP_MIN_CM;
    if (vertical_snapshot.h_sp_cm > VERTICAL_H_SP_MAX_CM) vertical_snapshot.h_sp_cm = VERTICAL_H_SP_MAX_CM;

    /* ⑥ 停手。只要处于定高模式就评估，**不看是否解锁**：
          上锁时不控制，但停手状态本身照样成立（飞机就在地面上），
          这样解锁那一刻就已经是停手，不会留出"刚解锁就以为自己在飞"的窗口。 */
    if (vertical_snapshot.mode == (uint8_t)VERTICAL_MODE_ALT_HOLD)
    {
        /* 起飞保护先更新：飞机明显离地、或飞行员把杆收回下降段，都算这次起飞结束。 */
        if (vertical_takeoff_guard)
        {
            if ((vertical_snapshot.est_valid && (vertical_snapshot.h_est_cm > VERTICAL_TAKEOFF_GUARD_CLEAR_H_CM)) || (vertical_snapshot.stick_rate_cms < 0.0f))
            {
                vertical_takeoff_guard = false;
            }
        }

        if (vertical_snapshot.grounded)
        {
            /* 出口只有一个：杆积分出来的 h_sp 超过门槛。
               ⚠️ 不能改成 h_est——停手把油门压到最低，飞机离不开地，
               h_est 永远涨不上去，那样会永久卡死在停手，推杆也起不来。 */
            if (vertical_snapshot.h_sp_cm > VERTICAL_GROUND_EXIT_H_SP_CM)
            {
                vertical_snapshot.grounded = false;
                vertical_snapshot.ground_timer_ms = 0U;
                /* 置起飞保护：飞行员要起飞了，在他真正离地之前不许再判停手，
                   否则推力偏弱时会变成"刚推杆又被按回去"的死循环。 */
                vertical_takeoff_guard = true;
            }
        }
        else if (!vertical_takeoff_guard && vertical_snapshot.est_valid && (vertical_snapshot.h_est_cm <= VERTICAL_GROUND_ENTER_H_CM) && (vertical_snapshot.stick_rate_cms <= 0.0f))
        {
            /* 进入条件要连续满足，单拍跌破不动作：否则起飞擦地时会在"停手 ↔ 控制"之间来回抖。
               还要看"本拍有没有在命令爬升"：否则飞机在地面上、飞行员推杆要起飞时，
               它会在 500ms 后又被判成停手，把刚发出的爬升指令按回去。
               而"被气流或障碍物压到地面、杆还停在中位"这种情况杆的变化率是 0，
               照样能进——这正是当初决定不要求"杆必须在最低位"的那个场景。 */
            elapsed_ms = (uint32_t)((dt_s * 1000.0f) + 0.5f);
            vertical_snapshot.ground_timer_ms += elapsed_ms;
            if (vertical_snapshot.ground_timer_ms >= VERTICAL_GROUND_ENTER_HOLD_MS)
            {
                vertical_snapshot.grounded = true;
                /* 进停手时把设定值归零（下限），**不是**对齐到当前高度。
                   为什么不能对齐到 h_est：地面读数是"镜头到地面的距离"，本来就不小
                   （不垫高约 1cm，垫高约 4cm），而对齐出来的 h_sp 只要超过退出门槛 4cm，
                   下一拍就会立刻满足"退出"条件 → 进入/退出来回抖。
                   归零之后这个边角彻底消失，而且退出的无扰性不受影响：
                   退出时 h_sp≈4cm、h_est≈地面读数，外环误差仍然接近 0。 */
                vertical_snapshot.h_sp_cm = VERTICAL_H_SP_MIN_CM;
            }
        }
        else
        {
            /* 条件不满足：计时清零，必须重新连续满足。 */
            vertical_snapshot.ground_timer_ms = 0U;
        }
    }
    else
    {
        vertical_snapshot.grounded = false;
        vertical_snapshot.ground_timer_ms = 0U;
        vertical_takeoff_guard = false;
    }

    /* ⑦ 门禁②。必须放在停手判定之后：里面有一位就是"不处于停手"。 */
    vertical_snapshot.control_gates = VerticalControl_EvalControlGates(armed, roll_deg, pitch_deg, vertical_snapshot.grounded);

    /* ⑧ 定状态。模式回答"切没切进定高"，状态还要回答"控制器有没有真的在干活"，
          两个都要能观测，否则出问题时分不清是哪一种。 */
    if (vertical_snapshot.mode != (uint8_t)VERTICAL_MODE_ALT_HOLD)
    {
        vertical_snapshot.state = (uint8_t)VERTICAL_STATE_MANUAL;
    }
    else if (vertical_snapshot.grounded)
    {
        vertical_snapshot.state = (uint8_t)VERTICAL_STATE_GROUNDED;
    }
    else if (vertical_snapshot.control_gates == VERTICAL_GATE_CT_ALL)
    {
        vertical_snapshot.state = (uint8_t)VERTICAL_STATE_ALT_HOLD;
    }
    else
    {
        vertical_snapshot.state = (uint8_t)VERTICAL_STATE_IDLE;
    }

    /* ⑨ 垂速内环（H9）。⚠️ 必须在 ⑧ 之后：它只在"门禁②全通且不在停手"时才动，
          而那两件事正是 ⑥⑦⑧ 算出来的。
          不满足就**复位并输出 0**，不是"保持上一拍"：
          ① 停手时控制器必须什么都不做，包括不留残值；
          ② "保持上一拍"是 H12 要做的事（空中门禁失效时保住悬停油门），
             那需要内环积分器存在（H10）才有东西可保持，现在没有，所以宁可闭嘴。 */
    if (VerticalControl_IsActive())
    {
        /* ⚠️⚠️ 参数顺序**绝对不能反**：Pid_Update(pid, setpoint, measured, dt)
              内部算的是 error = setpoint − measured。vz_sp = 0 所以 error = −vz_est，
              **这个减号就是整个回路里唯一的反号**，也就是负反馈的全部来源。
              写成 `Pid_Update(&vertical_inner_pid, vertical_snapshot.vz_est_cms, 0.0f, dt_s)`
              就是正反馈：飞机被抬起来反而加油门，会一直往上飞。
              编译器不会报错，静态检查也查不出来 —— 这是 H9 台架验收 ⑤⑥ 存在的全部理由。 */
        vertical_snapshot.vz_sp_cms = VERTICAL_INNER_VZ_SP_CMS;
        vertical_snapshot.vz_correction_us = Pid_Update(&vertical_inner_pid, VERTICAL_INNER_VZ_SP_CMS, vertical_snapshot.vz_est_cms, dt_s);
    }
    else
    {
        vertical_snapshot.vz_sp_cms = 0.0f;
        vertical_snapshot.vz_correction_us = 0.0f;
        Pid_Reset(&vertical_inner_pid);
    }

    /* ⑩ 遥测字节：实际模式；请求与实际不一致就在最高位标一下，
          飞行员在遥控器上能一眼看出"我按了，但飞机没照做"。 */
    vertical_snapshot.telemetry_mode = vertical_snapshot.mode;
    if (requested_mode != vertical_snapshot.mode)
    {
        vertical_snapshot.telemetry_mode |= 0x80U;
    }
}

uint16_t VerticalControl_GetThrottleUs(void)
{
    float throttle_us;

    /* 三种"控制器不干活"的情况一律返回最小油门（1000，电机停转）。
       判据与 Update 的 ⑨ 共用同一个 VerticalControl_IsActive()，不各写一遍：
         ① 普通模式（调用方按模式分叉，正常不会问到这里）。这里给最小值而不是给一个
            "看起来合理"的值——万一将来有人误接，表现是"油门不动"，而不是"油门乱动"；
         ② 停手：飞机在地面上，控制器什么都不做。这一条正是 H6 里缺的那一半执行权限；
         ③ 已进定高但门禁②没过（未解锁 / 读数不可信 / 姿态歪太多 / 仍判在停手）。
            ✅ **在地面上**这完全正确：门禁没过就什么都不做，和停手一致。
            ⚠️⚠️ **在空中这是一个已知缺口、不许飞定高**（待办 5 / 方案 §6.1 后的注）：
               空中丢读数时返回 1000 等于**空中停车**，而方案 §6.2 第 3 条要求的是
               "传感器失效时**退回基准油门继续飞**"（§2.3：内环积分器 = 悬停油门估计器）。
               正确做法是**保持上一次的控制输出**，而那要等 H10 把内环积分接上才做得了。
               在此之前用 1000 是因为"宁可停机也不给一个来路不明的油门"——
               在拆桨的台架上这个取值风险为零，也正是 H6/H8 要验的东西。
               ⇒ H12 完成自动退出/自动降落之前，定高只许台架验证。 */
    if (!VerticalControl_IsActive())
    {
        return MOTOR_PWM_MIN_US;
    }

    /* 控制启用（st=1）：**基准油门 + 垂速内环修正量**（H8 的占位值在 H9 被这段取代）。
       ⚠️ 修正量已经在 VerticalControl_RefreshInnerParams() 算出的 [out_min, out_max]
          内限幅过，这里不再夹取。但 out_min 的来源保证了结果**永远不会碰到
          混控有效下限 1100**（见 VERTICAL_BASE_PWM_US 的三条理由），
          否则 motor_test 的"四路等值"分支会命中，姿态环被旁路并 Reset。
       ⚠️ dt_s 不合理时 Pid_Update 返回 0，于是这里退化成"只给基准油门"——
          这是刻意的安全侧回退：断拍时保持一个已知安全的推力，而不是冲出去。 */
    throttle_us = (float)VERTICAL_BASE_PWM_US + vertical_snapshot.vz_correction_us;
    return (uint16_t)(throttle_us + 0.5f);
}

void VerticalControl_GetSnapshot(VerticalControlSnapshot_t *snapshot)
{
    if (snapshot == NULL) return;
    *snapshot = vertical_snapshot;
}

void VerticalControl_SetDebugRequestedMode(uint8_t mode)
{
    vertical_debug_requested_mode = mode;
}
