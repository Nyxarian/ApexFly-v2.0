/**
 * @file vertical_control.c
 * @brief 定高模式的状态机、两层门禁与摇杆映射实现（H6：修正量恒 0）。
 *
 * ── 本文件做什么、不做什么 ────────────────────────────────────
 * 做：模式切换门禁（门禁①）、控制启用门禁（门禁②）、油门杆三段映射、
 *     h_sp 积分、停手（地面待机）状态。
 * 不做：不写电机、不打印、不做外环内环、不产生任何油门修正量。
 *     所以本环节对飞机行为**零影响**——这是刻意安排的：
 *     门禁和状态机是最容易出人命的逻辑，先用"完全没有执行权限"的方式在台架上验完，
 *     再让它碰油门。
 *
 * ── 两层门禁为什么要拆开 ──────────────────────────────────────
 * 模式切换发生在**地面（已上锁）**，而控制只在天上（已解锁）才需要。
 * 混成一层会自相矛盾——"要求已解锁才能切换"和"切换必须在地面"打架。
 *
 * ── 每拍的执行顺序（就是下面 Update 里的顺序，不要重排）────────
 *   ① 读估计器 → ② 填杆量与请求 → ③ 算门禁① → ④ 切换（可能复位）
 *   → ⑤ 积分 h_sp → ⑥ 停手判定 → ⑦ 算门禁② → ⑧ 定状态 → ⑨ 遥测字节
 * ⑤ 必须在 ④ 之后：切换时复位出来的 h_sp 是这一拍积分的起点。
 * ⑦ 必须在 ⑥ 之后：门禁②里有一位就是"不处于停手"。
 */

#include "vertical_control.h"

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

void VerticalControl_Init(void)
{
    memset(&vertical_snapshot, 0, sizeof(vertical_snapshot));
    vertical_snapshot.mode = (uint8_t)VERTICAL_MODE_MANUAL;
    vertical_snapshot.state = (uint8_t)VERTICAL_STATE_MANUAL;
    vertical_snapshot.h_sp_cm = VERTICAL_H_SP_MIN_CM;
    vertical_takeoff_guard = false;
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
    /* mode 不动：本函数也会在"模式切换成功"时被调用，那时新模式已经写进去了。
       杆量、门禁位图每拍都会被覆盖，不需要清。 */
}

void VerticalControl_Update(uint8_t requested_mode, uint16_t throttle_stick, bool link_ok, bool armed, float roll_deg, float pitch_deg, float dt_s)
{
    VerticalEstState_t est;
    uint32_t elapsed_ms;

    /* ① 读估计器。valid 为 false 时 h/vz 是"最后一次可信值"而不是当前值，
          所以下面每一处拿高度做判断的地方都要先看 est.valid。 */
    VerticalEst_GetState(&est);
    vertical_snapshot.est_valid = est.valid;
    vertical_snapshot.h_est_cm = est.height_cm;

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

    /* ⑨ 遥测字节：实际模式；请求与实际不一致就在最高位标一下，
          飞行员在遥控器上能一眼看出"我按了，但飞机没照做"。 */
    vertical_snapshot.telemetry_mode = vertical_snapshot.mode;
    if (requested_mode != vertical_snapshot.mode)
    {
        vertical_snapshot.telemetry_mode |= 0x80U;
    }
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
