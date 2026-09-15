/**
 * @file vertical_control.c
 * @brief 定高模式的状态机、两层门禁、摇杆映射、基础油门与垂速内环实现。
 *
 * ── 本文件做什么、不做什么 ────────────────────────────────────
 * 做：模式切换门禁（门禁①）、控制启用门禁（门禁②）、油门杆三段映射、
 *     h_sp 积分、停手（地面待机）状态、定高模式下的基础油门（H8 起）、
 *     垂速内环 P（H9 起）、悬停油门估计器（H10 起）、
 *     **失效处理（H12 起）**。
 * 不做：不写电机、不打印、不做外环（H13）、不做自动降落（待 H16）。
 *
 * ── 垂速内环（H9）───────────────────────────────────────────
 * 快层控制律 ∆µs = Kp_v × (vz_sp − vz_est)，本阶段只有 P（Ki/Kd 走参数表，默认 0）。
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
 *    这个"减号"在 H9 已实测通过（强信号 85/85 反号、等效 Kp 19.9）。
 *
 * ── 悬停油门估计器（H10）──────────────────────────────────────
 * ⭐⭐ 定高模式的基础油门是**两层之和**（见 vertical_control.h 的常量段）：
 *      慢层 = 积分器（本文件的 `vertical_throttle_base_us`），回答"悬停要多少油门"
 *      快层 = 上面的垂速内环 P，回答"此刻要加还是减"
 *    为什么慢层必须手写、不能直接用 `Pid_t` 的内部积分：
 *      `pid.c` 的积分钳位是**对称的**（±i_limit），而我们要的区间
 *      [1150, 1650] 相对 0 是**不对称**的。硬套只能玩"把积分当偏移、中心挪到 1400"，
 *      可读性差、以后必被改错。⇒ 拆成两个部件，各自职责单一。
 *    ⚠️⚠️ 因此 `vertical_inner_pid` 的 **ki 必须传 0**（RefreshInnerParams 里写死），
 *       让 `pid.c` 的内部积分永不运行。否则**两个积分器会打架**：
 *       一个在 [1150,1650] 上累加绝对油门，另一个在 ±50 上累加修正量，
 *       表现是积分行为不可预测。这是本环节最容易被后人改错的一处。
 *
 * 慢层的三个动作（全部必须显式做，漏一个就是 bug）：
 *   ① 每拍（仅在控制启用时）按 error = vz_sp − vz_est 累加，再钳 [1150, 1650]；
 *   ② 不活跃（普通模式 / 停手 / 门禁②没过）时**回初值 1250**，不留残值；
 *   ③ 模式切换（Reset）时回初值 1250。
 *   ⚠️ ② 有一个**例外**（H12）：处于失效保持/下降（`st=6/7`）时慢层**冻结、不回初值** ——
 *      那正是"短暂失效恢复后必须无缝"的全部依据。见下面「失效处理」一节。
 *
 * ── 失效处理（H12）─────────────────────────────────────────────
 * ⭐⭐ 这一节解决的是**唯一一个"不改就不许飞"的问题**：门禁②一抖就返回 1000，
 *    而 1000 既等于空中停车、又会旁路姿态环（`motor_test.c` 判据 `thr <= 1100`）
 *    ⇒ 飞机翻滚着砸下来。H10 实测 451 个样本里 **41 个（9% 的时间）**在这个状态。
 *
 * 三层结构，顺序不能换：
 *
 *   ⑨  EvaluateFailReason()   —— 现在失效了没有、什么原因（进入门槛）
 *   →  失效状态机（NONE / HOLD / DESCEND）
 *   →  ⑩  内环与慢层            —— 失效期间**冻结**（不积分、不 Pid_Reset）
 *   →  GetThrottleUs()          —— 失效期间返回锁存/斜坡值，**早于** IsActive() 判断
 *
 * 三个最容易写错的地方，都在代码里就地注释了：
 *   ① **锁存必须在 ⑩ 之前**：那一拍 `IsActive()` 已经为 false，`i` 会被 else 分支按回 1250。
 *      只有 `vertical_fs_state` 先置位，⑩ 的第二个分支才会豁免它。
 *   ② **GetThrottleUs 里失效分支必须排在 `!IsActive()` 前面**：门禁②没过正是进入失效的原因，
 *      顺序反了就永远返回 1000，整套失效逻辑变成死代码。
 *   ③ **进入门槛与恢复门槛不是同一对数字**（高度那条有 125/115 的滞回），
 *      否则会在边界上"保持 ↔ 控制"来回切。见头文件 VERTICAL_FS_H_RECOVER_CM。
 *
 * ⚠️ 失效只在"定高模式 + **已解锁** + 不在停手"时评估。三个条件缺一不可：
 *    普通模式定高没参与；上锁时 `motor_test.c` 的 `!armed` 分支本来就把油门压到 1000；
 *    **停手是正常状态而不是失效** —— 少了这一条，地面上正常停手会被锁存成一个悬停油门，
 *    飞机自己就起飞了。这是本节最危险的一处误判。
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
 *   ⓪ 刷新内环参数（H9）
 *   → ① 读估计器 → ② 填杆量与请求 → ③ 算门禁① → ④ 切换（可能复位）
 *   → ⑤ 积分 h_sp → ⑥ 停手判定 → ⑦ 算门禁② → ⑧ 定状态
 *   → ⑨ 失效处理（H12） → ⑩ 垂速内环快层 + 悬停油门慢层（H9/H10）
 *   → ⑪ 遥测字节
 * ⑤ 必须在 ④ 之后：切换时复位出来的 h_sp 是这一拍积分的起点。
 * ⑦ 必须在 ⑥ 之后：门禁②里有一位就是"不处于停手"。
 * ⑨ 必须在 ⑧ 之后：失效要覆盖 ⑧ 定出来的状态（`IDLE` → `st=6/7`）。
 * ⑩ 必须在 ⑨ 之后：失效期间要**冻结**两层，而"是否冻结"由 ⑨ 决定。
 */

#include "vertical_control.h"

/* 基础油门要用到"电机停转"这个值。方向是 Control → Motor，和 motor_test.c
   引用本模块的方向相反但不冲突：这里只需要一个常量，不需要任何电机状态。
   pid / pid_table 同属 Control 层，方向一致，不引入新的跨层依赖。
   ⚠️ H10 起**不再 include mixer.h**：地板/天花板已经改成本模块自己的常量
      （VERTICAL_CONTROL_THROTTLE_MIN/MAX_US），不再需要读混控有效下限。
      那两个常量本身已经保证了"永远不碰到 1100"，理由写在 vertical_control.h 里。 */
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

/* ── 悬停油门估计器（H10）：慢层 ────────────────────────────────
   它**不是** PID 的积分项，而是手写的一阶累加器，单位微秒，直接表示"当前认为的
   悬停所需基础油门"。为什么手写：`pid.c` 的积分钳位是对称的，装不下
   [1150, 1650] 这种相对 0 不对称的区间。详见文件头。
   只在本模块内部读写，不跨模块暴露（打印走快照里的 vz_throttle_base_us）。 */
static float vertical_throttle_base_us;

/* 参数缓存：每拍读参数表，**只有真的变了才 Pid_ApplyParams**。
   为什么要缓存：Pid_ApplyParams 内部会清积分与微分历史。
   ⚠️ 高度环的慢层积分器（vertical_throttle_base_us）**不受它影响**——
      那是刻意的：遥控改 `vz_kp` 不该把已经学到的悬停油门估计丢掉。
   ⚠️ 不再缓存 `max_throttle_us`：H9 的动态限幅在 H10 已被常量区间取代，
      那个参数与本模块再无关系（它只影响普通模式的满杆油门）。
   ⚠️ 遥控在解锁中禁止改参（PidTable_HandleCommand 的既有约束），
      所以这个"变了"只会在台架上发生，空中不会因为调参而清积分。 */
static float vertical_inner_kp;
static float vertical_inner_ki;
static float vertical_inner_kd;
static bool vertical_inner_params_valid;

/* ── 失效处理（H12）────────────────────────────────────────────
   内部子状态。为什么不直接拿对外的 `VERTICAL_STATE_FAILSAFE_HOLD/DESCEND` 当状态：
   那两个是**要进黑匣子、要打印**的细分状态，而这里要回答的是"失效走到哪一步了"。
   两者一一对应（NONE↔不在失效 / HOLD↔st=6 / DESCEND↔st=7），
   但分成两个变量之后，"对外状态"永远是唯一一处推导出来的，不会出现
   "state 写了一套、失效逻辑以为是另一套"的错位。
   对外的 st 在 Update 的 ⑨ 之后由本变量推出。 */
typedef enum
{
    /* 没失效，控制器正常干活。 */
    VERTICAL_FS_NONE = 0U,
    /* 失效保持：油门锁存、计时，等门禁恢复。 */
    VERTICAL_FS_HOLD = 1U,
    /* 失效下降：从锁存值按斜率降到 1000，**单向不回头**。 */
    VERTICAL_FS_DESCEND = 2U
} VerticalControlFsState_t;

static VerticalControlFsState_t vertical_fs_state;

/* 失效期间的**实际输出油门**，单位微秒。
   · 进入保活那一刻 = `慢层 + 快层`（钳过）的当拍值 —— 也就是"失效前飞机正在用的推力"，
     这是方案 §2.3 说的"内环积分器即悬停油门估计器"的直接用法。
     ⚠️ 比任何常数都好：它就是当前重量/电压下真正能悬停的那个值。
   · 进入下降之后 = 从这个值按 VERTICAL_FS_RAMP_US_PER_S 线性往下走的当前值。 */
static float vertical_fs_us;

/* 进入失效的原因，取 VerticalControlFailReason_t。与快照里的 fail_reason 同步。 */
static uint8_t vertical_fail_reason;

/* ⭐ "本次解锁之后，控制器真的干过活没有"（H12）。
   用途只有一个，但**不能省**：区分"飞行中掉了读数"与"在地面上压根没飞起来"。
   没飞起来时绝不能锁存悬停油门 —— 那是在**地面上**把油门顶到 1250µs，
   而这时停手因为"读数坏 ⇒ 进不去"正好是失效的，没有任何东西能挡住飞机窜出去。
   置位：门禁②全通那一拍。清除：上锁，或模式回到普通。
   ⚠️ 停手**不**清它（飞机刚落地还没上锁，这份记忆要留着）。 */
static bool vertical_ever_active;

/**
 * @brief 把油门整体钳到定高模式允许的区间 [地板, 天花板]，单位微秒。
 *
 * ⚠️ 单独抽出来是为了让"锁存失效前输出"与"算正常输出"用**同一对常量**。
 *    两处各写一遍迟早会不一致，那时会出现"锁存值比正常输出还大"这种查不出来的怪事。
 */
static float VerticalControl_ClampThrottle(float throttle_us)
{
    if (throttle_us < VERTICAL_CONTROL_THROTTLE_MIN_US) throttle_us = VERTICAL_CONTROL_THROTTLE_MIN_US;
    if (throttle_us > VERTICAL_CONTROL_THROTTLE_MAX_US) throttle_us = VERTICAL_CONTROL_THROTTLE_MAX_US;
    return throttle_us;
}

/**
 * @brief 把失效相关的三个模块变量与两个快照字段一起清干净。
 *
 * ⚠️ 必须**整组**清，不能只清其中几个：`fs_state` 清了而 `failsafe_ms` 没清的话，
 *    下一次进保持会从一个残留的计时开始，2 秒保持会立刻到期。
 */
static void VerticalControl_ClearFailsafe(void)
{
    vertical_fs_state = VERTICAL_FS_NONE;
    vertical_fs_us = 0.0f;
    vertical_fail_reason = (uint8_t)VERTICAL_FAIL_NONE;
    vertical_snapshot.failsafe_ms = 0U;
    vertical_snapshot.fail_reason = (uint8_t)VERTICAL_FAIL_NONE;
}

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
 * @brief 控制器是否真的在干活（H9；H12 起把"失效期间"也算作不干活）。
 *
 * 判据 = "处于定高模式 + 不在停手 + 门禁②全通 + **不在失效状态**"，也就是 `state == ALT_HOLD`。
 * ⚠️ 抽成一个函数，是为了让"算修正量"（Update 的 ⑩）和"给基础油门"（GetThrottleUs）
 *    用**同一个判据**。两处各写一遍迟早会不一致，那时会出现
 *    "修正量在算、但基础油门给的是 1000"这种自相矛盾的状态。
 * ⚠️ H12 加上的最后一条（`fs_state == NONE`）不是可有可无的：
 *    失效期间两层必须**冻结**（不积分、不 Pid_Reset），靠的就是它返回 false。
 *    ⚠️ 但 `GetThrottleUs()` 里**不能**先问它 —— 失效时门禁②本来就不过，
 *       先问就会直接返回 1000，等于把 H12 整套逻辑变成死代码。
 */
static bool VerticalControl_IsActive(void)
{
    return (vertical_snapshot.mode == (uint8_t)VERTICAL_MODE_ALT_HOLD) &&
           (!vertical_snapshot.grounded) &&
           (vertical_snapshot.control_gates == VERTICAL_GATE_CT_ALL) &&
           (vertical_fs_state == VERTICAL_FS_NONE);
}

/**
 * @brief 判定"现在失效了没有、什么原因"（H12）。返回 VerticalControlFailReason_t。
 *
 * @param h_limit_cm 高度越界的门槛，单位厘米。
 *        ⚠️ **进入和恢复用的不是同一个值**（125 / 115，见头文件）：
 *           高度是个连着变的量，两个门槛取同一个值就会在边界上来回穿
 *           （h_sp 上限就是 120cm，飞机很容易在 124~126 之间游走）。
 *           姿态与读数两条判据是状态量、天然不带滞回，不受影响。
 *
 * ── 优先级：姿态 > 读数 > 高度（顺序是刻意的）──────────────────
 * 飞机真撞了的时候姿态**一定**先坏，而 ToF 往往同时也坏（镜头朝侧面了）。
 * 那时若先报"读数失效"，日志上就会写成"传感器问题"，把真正的原因盖掉 ——
 * 而这两条的**处理方式完全不同**（姿态是立即停桨，读数是慢降）。
 */
static uint8_t VerticalControl_EvalFailReason(float roll_deg, float pitch_deg, float h_limit_cm)
{
    if ((VerticalControl_Absf(roll_deg) >= VERTICAL_CONTROL_MAX_TILT_DEG) ||
        (VerticalControl_Absf(pitch_deg) >= VERTICAL_CONTROL_MAX_TILT_DEG))
    {
        return (uint8_t)VERTICAL_FAIL_ATTITUDE;
    }

    if (!vertical_snapshot.est_valid)
    {
        return (uint8_t)VERTICAL_FAIL_SENSOR;
    }

    /* ⚠️ 必须先确认 est_valid：读数不可信时 h_est 是**冻结的旧值**，
       拿它判高度会拿一个几分钟前的高度反复触发失效。 */
    if (vertical_snapshot.h_est_cm > h_limit_cm)
    {
        return (uint8_t)VERTICAL_FAIL_HEIGHT;
    }

    return (uint8_t)VERTICAL_FAIL_NONE;
}

/**
 * @brief 失效状态机（H12）。每拍调用一次，必须在 ⑧（定状态）之后、⑩（内环/慢层）之前。
 *
 * ⚠️ 必须在 ⑩ 之前：进入失效时要**先锁存**"慢层 + 快层"的当拍值，
 *    而那一拍 `IsActive()` 已经为 false，⑩ 的 else 分支会把慢层按回 1250。
 *    只有 `vertical_fs_state` 先置位，⑩ 的第二个分支才会豁免它（见 ⑩ 的注释）。
 *
 * ── 四个守卫的顺序是刻意的，不要重排 ──────────────────────────
 *   ① 没进定高 / 上锁   → 本次飞行结束，连"飞过没有"的记忆一起清掉；
 *   ② 停手             → 正常状态、不是失效（**但不清"飞过没有"**）；
 *   ③ 门禁②全通        → 一切正常，顺手记下"这次真的在飞"；
 *   ④ **没飞过**        → 不做失效处理，交给 GetThrottleUs 返回 1000。
 *
 * @param armed 当前是否已解锁。失效只在**已解锁**时评估（上锁 = 电机停转，另有分支）。
 */
static void VerticalControl_UpdateFailsafe(bool armed, float roll_deg, float pitch_deg, float dt_s)
{
    uint8_t entry_reason;
    uint32_t elapsed_ms;

    /* ── 守卫①：没进定高 / 上锁 ────────────────────────────────
       这是"一次飞行"的边界，所以**连 `vertical_ever_active` 一起清**。
       ⚠️ 少了它，上一次飞行留下的"我飞过"会跨到下一次解锁，把守卫④废掉。 */
    if ((vertical_snapshot.mode != (uint8_t)VERTICAL_MODE_ALT_HOLD) || (!armed))
    {
        vertical_ever_active = false;
        VerticalControl_ClearFailsafe();
        return;
    }

    /* ── 守卫②：停手是正常状态，不是失效 ────────────────────────
       ⚠️ 少了这一条，地面上正常停手会被当成失效并锁存一个悬停油门 ⇒ 飞机自己起飞。
       ⚠️ 但**不清 `vertical_ever_active`**：飞机刚落地还没上锁，这份记忆要留着，
          万一飞行员又推杆起飞，不该因为"中间停过一次手"就整段失效处理失效。 */
    if (vertical_snapshot.grounded)
    {
        VerticalControl_ClearFailsafe();
        return;
    }

    /* ── 守卫③：门禁②全通 ⇒ 记下"这次真的在飞" ────────────────────
       ⚠️⚠️ **这里绝不能直接 return**，这是本函数最容易写错的一处：
       **高度越界不在门禁②里**，所以"h > 125cm 而其它一切正常"那一拍
       `IsActive()` 恰好为 true；直接返回会让高度失效整条判据变成死代码
       （写代码时就是这么错了一次，靠通读抓回来的）。
       ⇒ 只置位、不返回，继续往下查失效原因。 */
    if (VerticalControl_IsActive())
    {
        vertical_ever_active = true;
    }

    /* ── 守卫④：门禁②没过，但**本次解锁后控制器从来没真的干过活** ────
       ⭐⭐ 这一条是 H12 自己引入的风险的补丁，不是可有可无的保险：
       典型场景是"上一次飞行把 ToF 摔坏了 → 落地 → 上锁 → 重新解锁（ToF 还没恢复）"。
       这时停手判据要求 `est_valid`，而读数坏着 ⇒ **停手永远进不去** ⇒ `grounded` 恒 false
       ⇒ 如果没有守卫④，就会锁存 `慢层 + 快层`（≈1250µs）
       ⇒ **在**地面上**把油门顶到接近悬停值**，飞机自己就窜出去了。
       判据用"飞过没有"而不是"高度"，是因为高度在读数坏的时候正好不可用。
       ⇒ 没飞过就什么都不做，交给 `GetThrottleUs()` 返回 1000（= 改动前的行为）。 */
    if (!vertical_ever_active)
    {
        VerticalControl_ClearFailsafe();
        return;
    }

    /* 计时只在 dt 合理时推进，判据与 `pid.c` / 停手完全相同。
       断拍时**不推进**，于是保持段会被拉长 —— 这是安全侧（多等一会儿再降）。 */
    if ((dt_s > 0.0005f) && (dt_s <= 0.10f))
    {
        elapsed_ms = (uint32_t)((dt_s * 1000.0f) + 0.5f);
    }
    else
    {
        elapsed_ms = 0U;
    }

    entry_reason = VerticalControl_EvalFailReason(roll_deg, pitch_deg, VERTICAL_FS_H_MAX_CM);

    if (vertical_fs_state == VERTICAL_FS_NONE)
    {
        if (entry_reason == (uint8_t)VERTICAL_FAIL_NONE)
        {
            /* **一切正常**（绝大多数拍都走这里）：什么都不做，⑩ 会走正常控制路径。
               顺手整组清零，让 `[V]` 里的 fs= 在没失效时就是 0。
               用 ClearFailsafe() 而不是手写两个字段：清失效状态只有一处实现，
               以后加字段不会漏（漏一个就是"下次进保持立刻到期"这类难查的 bug）。 */
            VerticalControl_ClearFailsafe();
            return;
        }

        vertical_fail_reason = entry_reason;
        vertical_snapshot.fail_reason = entry_reason;
        vertical_snapshot.failsafe_ms = 0U;

        if (entry_reason == (uint8_t)VERTICAL_FAIL_ATTITUDE)
        {
            /* ⭐ 决策（2026-09-15 用户定）：姿态超限 = 判定撞击炸机，**不保持、不斜坡**。
               理由：保持段的用途是"抓瞬时失效"，而姿态歪着不是瞬时事件、它什么也抓不到；
               真做 2s 保持 + 30µs/s 斜坡的话，撞地之后桨还要转约 16 秒
               （要先把 1420µs 降到 1000），与"不管了、赶紧停桨"的本意正相反。 */
            vertical_fs_us = (float)MOTOR_PWM_MIN_US;
            vertical_fs_state = VERTICAL_FS_DESCEND;
        }
        else
        {
            /* 读数失效 / 高度越界：**飞机姿态是好的** ⇒ 先保持 2 秒等它恢复。
               锁存值是"失效前一刻的实际输出"，也就是当前重量电压下真正能悬停的推力。
               ⚠️ 不能取常数、也不能取 1000：
                  取 1000 = 空中停车（这就是 H12 要修的东西）；
                  取常数 = 忽略了电压/载荷，只是"看起来合理"。
               ⚠️ 只在**已经飞起来过**的前提下才锁存（守卫④）。 */
            vertical_fs_us = VerticalControl_ClampThrottle(vertical_throttle_base_us + vertical_snapshot.vz_correction_us);
            vertical_fs_state = VERTICAL_FS_HOLD;
        }
        return;
    }

    if (vertical_fs_state == VERTICAL_FS_HOLD)
    {
        /* 保持期间姿态变坏 ⇒ 立刻按"炸机"处理。上面那条规则的延续，
           不加这一条的话，一架在保持段里被风刮翻的飞机会继续被"温柔地"保持住。 */
        if (entry_reason == (uint8_t)VERTICAL_FAIL_ATTITUDE)
        {
            vertical_fs_us = (float)MOTOR_PWM_MIN_US;
            vertical_fail_reason = (uint8_t)VERTICAL_FAIL_ATTITUDE;
            vertical_snapshot.fail_reason = (uint8_t)VERTICAL_FAIL_ATTITUDE;
            vertical_fs_state = VERTICAL_FS_DESCEND;
            return;
        }

        /* ⭐ 恢复判据：用**更严**的高度门槛（115 而不是 125），其余同上。 */
        if (VerticalControl_EvalFailReason(roll_deg, pitch_deg, VERTICAL_FS_H_RECOVER_CM) == (uint8_t)VERTICAL_FAIL_NONE)
        {
            /* ⭐⭐ 恢复：只要把失效状态清干净就完事 —— 慢层/快层整段没被碰过
               （⑩ 的第二个分支一个字节都没改），所以油门**不会有任何跳变**。
               这就是方案 §15.1 要求的"短暂失效不该有任何动作"。 */
            VerticalControl_ClearFailsafe();
            return;
        }

        vertical_snapshot.failsafe_ms += elapsed_ms;
        if (vertical_snapshot.failsafe_ms >= VERTICAL_FS_HOLD_MS)
        {
            /* 保持期满 ⇒ 转入下降。**单向，不再回头**：
               允许半路恢复的话，"高度越界"会在 125cm 上下反复进出，
               而"读数失效"恢复后油门要重新爬回悬停值 —— 两件事都不好收尾，
               而且飞行员看到的行为会变成"抽一下、抽一下"。 */
            vertical_fs_state = VERTICAL_FS_DESCEND;
        }
        return;
    }

    /* ── VERTICAL_FS_DESCEND：从锁存值按固定斜率降到 1000（电机停转），到底停住 ──
       ⚠️ 30µs/s 远低于 `motor_test.c` 的下降限速 300µs/s ⇒ **限速器不参与**，
          实际曲线就是这里算出来的这条直线（台架验收直接量它）。
       ⚠️ 姿态环在 `thr > 1100` 期间照常工作（`motor_test.c` 的判据是 `<= 1100` 才旁路），
          所以飞机是**水平着地**的；只有最后 1100→1000 那几秒没有姿态修正，
          而那时飞机已经落地了。这正是 H12 相对"立刻 1000"最大的收益。 */
    if (elapsed_ms > 0U)
    {
        vertical_fs_us -= VERTICAL_FS_RAMP_US_PER_S * dt_s;
    }
    if (vertical_fs_us < (float)MOTOR_PWM_MIN_US)
    {
        vertical_fs_us = (float)MOTOR_PWM_MIN_US;
    }
    vertical_snapshot.failsafe_ms += elapsed_ms;
}

/**
 * @brief 每拍刷新快层参数（H9），参数没变就直接返回。
 *
 * ── H10 起这里不再算"动态限幅" ─────────────────────────────────
 * H9 时快层的限幅是从"基准油门 + 满杆上限 + 姿态余量"推导出来的动态值；
 * H10 把慢层（积分器）拿来之后，那个推导失去了意义——因为"基准"本身
 * 已经变成会自己走的积分器了，拿它去算一个静态限幅没有物理含义。
 * 现在限幅就是一个常量 ±VERTICAL_INNER_CORR_LIMIT_US（= 50µs），
 * **整个油门权限由 [地板, 天花板] 这一对常量负责**（见 GetThrottleUs）。
 *
 * ⚠️⚠️ `ki` 读出来但**传给 Pid_ApplyParams 的是 0**：
 *    参数表里的 `vz_ki` 驱动的是本文件手写的慢层积分器（见 Update 的 ⑨），
 *    不是 `Pid_t` 的内部积分。两个积分器同时跑会打架，所以这里必须写 0。
 *    这行是"看起来像笔误、实际是刻意"的典型，改之前先读文件头。
 */
static void VerticalControl_RefreshInnerParams(void)
{
    float kp;
    float ki;
    float kd;

    PidTable_GetAltitudeParams(&kp, &ki, &kd);

    if (vertical_inner_params_valid &&
        (kp == vertical_inner_kp) && (ki == vertical_inner_ki) && (kd == vertical_inner_kd))
    {
        return;
    }

    /* ki 传 0：慢层由本文件自己积分，不能用 PID 的内部积分。详见函数注释。 */
    Pid_ApplyParams(&vertical_inner_pid, kp, 0.0f, kd, 0.0f,
                    VERTICAL_INNER_CORR_LIMIT_US, -VERTICAL_INNER_CORR_LIMIT_US);

    vertical_inner_kp = kp;
    vertical_inner_ki = ki;
    vertical_inner_kd = kd;
    vertical_inner_params_valid = true;
}

void VerticalControl_Init(void)
{
    memset(&vertical_snapshot, 0, sizeof(vertical_snapshot));
    vertical_snapshot.mode = (uint8_t)VERTICAL_MODE_MANUAL;
    vertical_snapshot.state = (uint8_t)VERTICAL_STATE_MANUAL;
    vertical_snapshot.h_sp_cm = VERTICAL_H_SP_MIN_CM;
    vertical_takeoff_guard = false;

    /* 失效状态整组清零（H12）。上电 / IMU 重初始化时绝不能带着上一次的锁存油门。 */
    VerticalControl_ClearFailsafe();
    vertical_ever_active = false;

    /* 慢层积分器回初值。上电时我们不知道悬停要多少油门，只能从"台架观测基准"起步。 */
    vertical_throttle_base_us = VERTICAL_THROTTLE_START_US;
    vertical_snapshot.vz_throttle_base_us = VERTICAL_THROTTLE_START_US;

    /* 内环先给一组"输出恒 0"的合法初值，再立刻去读真实参数。
       万一读的时候参数表还没初始化（现在不会：motor_test.c 里 PidTable_Init 在前），
       这一组零值让内环表现成"什么都不做"，而不是拿未初始化的限幅去驱动电机。
       参数表就绪后，下一拍 RefreshInnerParams 会因为缓存不匹配而自动重新应用。 */
    Pid_Init(&vertical_inner_pid, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
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
       （H10 打开慢层之后，留着旧积分等于带着上一轮的悬停油门估计值进新一次飞行，
        而两次飞行的重量/电压可能不同）。
       ⚠️ 只清运行状态，不重读参数——参数没变就没必要 ApplyParams。 */
    Pid_Reset(&vertical_inner_pid);
    vertical_throttle_base_us = VERTICAL_THROTTLE_START_US;
    vertical_snapshot.vz_sp_cms = 0.0f;
    vertical_snapshot.vz_correction_us = 0.0f;
    vertical_snapshot.vz_throttle_base_us = VERTICAL_THROTTLE_START_US;
    /* 失效状态也要清（H12）：模式切换是"从零开始"，
       留着上一次的锁存油门会让新一次飞行在失效那一刻用上一架次的推力。
       ⚠️ 上锁时**不**走这里（Reset 只在模式切换与 IMU 重初始化时调用）——
          上锁的清理由 UpdateFailsafe 的守卫分支负责，那一条同时覆盖了"上锁"与"回普通模式"。 */
    VerticalControl_ClearFailsafe();
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

    /* ⑨ 失效处理（H12）。⚠️ 必须在 ⑧ 之后：它要**覆盖** ⑧ 定出来的状态
          （失效时门禁②本来就不过，⑧ 给的是 IDLE）。
       ⚠️ 必须在 ⑩ 之前：进入失效那一刻要锁存"慢层 + 快层"的当拍值，
          而那一拍 `IsActive()` 已经为 false，晚了就会被 ⑩ 的 else 分支按回 1250。 */
    VerticalControl_UpdateFailsafe(armed, roll_deg, pitch_deg, dt_s);
    if (vertical_fs_state == VERTICAL_FS_HOLD)
    {
        vertical_snapshot.state = (uint8_t)VERTICAL_STATE_FAILSAFE_HOLD;
    }
    else if (vertical_fs_state == VERTICAL_FS_DESCEND)
    {
        vertical_snapshot.state = (uint8_t)VERTICAL_STATE_FAILSAFE_DESCEND;
    }

    /* ⑩ 垂速内环（H9）+ 悬停油门估计器（H10）。
       ⚠️ 必须在 ⑨ 之后：失效期间要**冻结**两层，而"是否冻结"正是 ⑨ 决定的。
       三种情况：*/
    if (VerticalControl_IsActive())
    {
        /* ⚠️⚠️ 参数顺序**绝对不能反**：Pid_Update(pid, setpoint, measured, dt)
              内部算的是 error = setpoint − measured。vz_sp = 0 所以 error = −vz_est，
              **这个减号就是整个回路里唯一的反号**，也就是负反馈的全部来源。
              写成 `Pid_Update(&vertical_inner_pid, vertical_snapshot.vz_est_cms, 0.0f, dt_s)`
              就是正反馈：飞机被抬起来反而加油门，会一直往上飞。
              编译器不会报错，静态检查也查不出来 —— 这是 H9 台架验收 ⑤⑥ 存在的全部理由，
              H9 已实测通过（强信号 85/85 反号）。 */
        vertical_snapshot.vz_sp_cms = VERTICAL_INNER_VZ_SP_CMS;
        vertical_snapshot.vz_correction_us = Pid_Update(&vertical_inner_pid, VERTICAL_INNER_VZ_SP_CMS, vertical_snapshot.vz_est_cms, dt_s);

        /* ── 慢层：悬停油门估计器 ────────────────────────────────
           与快层**用同一个误差、同一个符号约定**（这是刻意的，两层的方向必须一致）：
             飞机上升（vz>0）→ error<0 → 积分下降 → 少给油 → 把它压回去；
             飞机下沉（vz<0）→ error>0 → 积分上升 → 多给油 → 把它托起来。
           ⚠️ 这个减号写反就是正反馈，性质与快层那个减号完全相同，也会导致失控。
              台架验收的 ⑥⑦ 两条专门抓它（匀速上抬 → i 必须往下降）。
           ⚠️ dt_s 用与 Pid_Update **完全相同**的有效性判据（[0.5ms, 100ms]）：
              断拍/任务阻塞后一个巨大的 dt 会把积分一次推出去很远。
              pid.c 里遇到这种 dt 是"本拍不动作"，这里照做，行为才一致。
           ⚠️ ki == 0 时完全不积分（与 pid.c 的 `if (pid->ki != 0.0f)` 一致），
              这样遥控把 vz_ki 调成 0 就等价于"冻结悬停油门估计"，便于台架取证。 */
        if ((dt_s >= 0.0005f) && (dt_s <= 0.10f) && (vertical_inner_ki != 0.0f))
        {
            vertical_throttle_base_us += (VERTICAL_INNER_VZ_SP_CMS - vertical_snapshot.vz_est_cms) * vertical_inner_ki * dt_s;
        }

        /* ⭐ 积分的钳位，与最终输出的钳位是**同一对数字**（见头文件的常量段）。
           只钳输出不钳积分 = windup：积分偷偷涨到几千而表面看不出来，
           表现成"长期死死顶在天花板"，一旦条件变化立刻猛冲。 */
        if (vertical_throttle_base_us < VERTICAL_CONTROL_THROTTLE_MIN_US) vertical_throttle_base_us = VERTICAL_CONTROL_THROTTLE_MIN_US;
        if (vertical_throttle_base_us > VERTICAL_CONTROL_THROTTLE_MAX_US) vertical_throttle_base_us = VERTICAL_CONTROL_THROTTLE_MAX_US;
        vertical_snapshot.vz_throttle_base_us = vertical_throttle_base_us;
    }
    else if (vertical_fs_state != VERTICAL_FS_NONE)
    {
        /* ── 失效期间（H12）：两层**冻结**，一个字节都不改 ──────────────
           这是"短暂失效恢复后必须无缝"的**全部依据**：恢复时 `i` 与 `corr` 还是
           失效前的值，所以 `GetThrottleUs()` 立刻回到失效前的输出，油门没有任何跳变。
           ⚠️ **这里绝对不能 `Pid_Reset`**：那会清掉微分历史，恢复瞬间产生一个假的 D 尖峰
              （同样的道理，慢层也不能回初值 1250 —— 那等于把学到的悬停油门丢掉）。
           ⚠️ 快照要照抄 `vertical_throttle_base_us`：不抄的话打印里 `i=` 会显示成上一拍的
              旧值，看起来像"积分被清了"，而实际上它只是没变。 */
        vertical_snapshot.vz_throttle_base_us = vertical_throttle_base_us;
    }
    else
    {
        vertical_snapshot.vz_sp_cms = 0.0f;
        vertical_snapshot.vz_correction_us = 0.0f;
        Pid_Reset(&vertical_inner_pid);
        /* 慢层回初值。只在**真的没在干活**时（普通模式 / 停手 / 上锁 / 门禁②没过且
           没进失效）才会走到这里 —— 失效那一种已经被上面那个分支接走了，
           所以 H12 之后"飞行中门禁抖一下丢掉悬停油门"这件事不再发生。 */
        vertical_throttle_base_us = VERTICAL_THROTTLE_START_US;
        vertical_snapshot.vz_throttle_base_us = VERTICAL_THROTTLE_START_US;
    }

    /* ⑪ 遥测字节：实际模式；请求与实际不一致就在最高位标一下，
          飞行员在遥控器上能一眼看出"我按了，但飞机没照做"。
          bit5-6 = 失效原因（H12）：⚠️ 遥控端**现在不解析**它（弹窗按用户要求缓做），
          先占位是刻意的 —— 将来加弹窗只改遥控端，飞控不用再烧一次。
          位分配与方案 §15.2 一致：bit0-3 模式、bit4 留给"定点开关"、bit5-6 失效原因、bit7 请求被拒。 */
    vertical_snapshot.telemetry_mode = (uint8_t)(vertical_snapshot.mode | (uint8_t)(vertical_snapshot.fail_reason << 5));
    if (requested_mode != vertical_snapshot.mode)
    {
        vertical_snapshot.telemetry_mode |= 0x80U;
    }
}

uint16_t VerticalControl_GetThrottleUs(void)
{
    float throttle_us;

    /* ⭐⭐ H12：失效状态**优先于一切**。
       ⚠️⚠️ 这一段的顺序是本环节最容易写反的地方：门禁②没过**正是**进入失效的原因，
           所以如果按老代码先问 `!IsActive()`，就会直接返回 1000，
           H12 整套状态机变成死代码、现象与改动前一模一样（空中停车）。
           ⇒ 判据必须是"失效子状态"，而不是"门禁通过没有"。 */
    if (vertical_fs_state != VERTICAL_FS_NONE)
    {
        /* 保持段返回锁存值（恒定），下降段返回斜坡当前值。
           两者都在 [1000, 1650] 之内，且**都远高于姿态环旁路线 1100**（斜坡末段除外）。 */
        return (uint16_t)(vertical_fs_us + 0.5f);
    }

    /* 剩下三种"控制器不干活"的情况一律返回最小油门（1000，电机停转）。
       判据与 Update 的 ⑩ 共用同一个 VerticalControl_IsActive()，不各写一遍：
         ① 普通模式（调用方按模式分叉，正常不会问到这里）。这里给最小值而不是给一个
            "看起来合理"的值——万一将来有人误接，表现是"油门不动"，而不是"油门乱动"；
         ② 停手：飞机在地面上，控制器什么都不做。这一条正是 H6 里缺的那一半执行权限；
         ③ 未解锁 / 已进定高但门禁②没过且**没进失效**（地面上读数不可信等）。
       ⚠️ ③ 剩下这种情况全部发生在地面上（空中门禁失效都由 ⑨ 接走了），
          所以返回 1000 是正确的 —— "地面不会自己起飞"这条安全性没有削弱。 */
    if (!VerticalControl_IsActive())
    {
        return MOTOR_PWM_MIN_US;
    }

    /* 控制启用（st=1）：**慢层积分器 + 快层修正量**，再整体钳到 [地板, 天花板]。
       （H8 的占位常数在 H9 被"固定基准 + 修正量"取代，H10 再把基准换成积分器。）

       ── 为什么这里还要再夹一次 ──────────────────────────────────
       慢层自己已经钳在 [1150, 1650]、快层已钳在 ±50，但**两者之和**可以到
       [1100, 1700]。所以这一层夹取不是冗余，是真正决定"电脑能动用的油门范围"的那一道。
       ⚠️ 它与慢层的钳位是**同一对常量**：改其中一个必须同时想清楚另一个。
          只钳这里不钳慢层 = windup（积分偷偷涨到几千，表面顶在天花板，条件一变就猛冲）。

       ⚠️ 地板 1150 > 混控有效下限 1100，所以结果**永远不会命中** `motor_test.c` 的
          "四路等值 + AttitudeControl_Reset()"分支，姿态环在定高模式下始终有效。
       ⚠️ dt_s 不合理时 Pid_Update 返回 0、慢层也不积分，于是这里退化成
          "保持上一拍的悬停油门估计" —— 这是刻意的安全侧回退：断拍时保持一个
          已知能悬停的推力，而不是掉到最小值（那等于空中停车）。 */
    throttle_us = VerticalControl_ClampThrottle(vertical_throttle_base_us + vertical_snapshot.vz_correction_us);
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
