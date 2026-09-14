/**
 * @file vertical_control.h
 * @brief 定高模式的状态机、两层门禁与摇杆映射对外接口。
 *
 * 本模块只回答三个问题：
 *   1. 现在处于哪个模式（普通 / 定高），以及遥控的请求要不要接受；
 *   2. 目标高度 h_sp 是多少（油门杆三段映射 + 积分得到）；
 *   3. 飞机在地面上时进入"停手"，控制器什么都不做。
 *
 * 本模块**不产生任何油门修正量、不写电机、不做外环内环**——那些是后面几个环节的事。
 * 当前阶段（H6）修正量恒 0，所以本模块对飞机行为零影响，只输出可打印、可记录的状态。
 * 这样安排的用意：门禁、模式状态机、停手是最容易出人命的逻辑，
 * 先用"完全没有执行权限"的方式在台架上验完，再让它碰油门。
 *
 * ── 谁驱动它、谁看它 ──────────────────────────────────────────
 * 驱动：`motor_test.c` 的控制环每拍调用一次 `VerticalControl_Update()`（250Hz）。
 *       姿态角、解锁状态、链路状态、遥控杆量全部由调用方传进来，本模块不自己跨模块取数。
 *       它内部只读一个东西：`vertical_est` 的高度/垂速估计。
 * 查看：`tof.c` 的低优先级任务每 500ms 调一次 `VerticalControl_GetSnapshot()` 打印。
 *       控制环里**禁止打印**（一行日志约 10ms，会吃掉两拍 4ms 预算），所以打印留在那边。
 *       ⚠️ 一写多读，结构体拷贝不是原子的，打印出来的字段理论上可能来自相邻两拍。
 *       对诊断没有影响，但知道这件事比不知道好。
 *
 * ── 定高是一个"独立模式"，不是一个开关 ────────────────────────
 * 切进这个模式后油门杆不再直通油门，而是三段式（下降 / 保持 / 上升）。
 * 模式**只在地面、且杆在最低位时才允许切换**（门禁①），所以一次飞行只能用一个模式。
 * 完整设计（含为什么取消"待命段"、为什么必须有"停手"）见
 * `优化一下/高度接入飞控方案.md` 第 5 节。
 */

#ifndef VERTICAL_CONTROL_H
#define VERTICAL_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── 摇杆三段 ──────────────────────────────────────────────────
   定高模式下油门杆的三段边界与目标高度变化率，单位厘米每秒，向上为正。
   段与段之间速率连续（在 350 和 650 处都是 0），所以边界上不会抖。
   满杆下拉 = 2cm/s 缓慢下降，不是停油门、更不是掉下来：
   这是取消"待命段"之后的保护，飞行员慌乱中把杆收到底也不会失控。 */
#define VERTICAL_STICK_HOLD_LOW 350U
#define VERTICAL_STICK_HOLD_HIGH 650U
#define VERTICAL_STICK_MAX 1000U
#define VERTICAL_RATE_DOWN_CMS 2.0f
#define VERTICAL_RATE_UP_CMS 3.0f

/* 目标高度 h_sp 的钳位，单位厘米。
   下限必须是 0 而不是 2：飞机放在平面上 h_est≈0.8cm，下限若高于它，
   外环会永远存在一个正误差，把飞机在地面上就顶起来。
   上限 120cm 给 ToF 的 1.3m 量程留 10cm 余量。 */
#define VERTICAL_H_SP_MIN_CM 0.0f
#define VERTICAL_H_SP_MAX_CM 120.0f

/* ── 门禁阈值 ────────────────────────────────────────────────── */

/* 门禁①"杆在最低位"的上限。写 50 而不是 0，是为了容忍摇杆的机械与校准余量。 */
#define VERTICAL_SWITCH_STICK_MAX 50U

/* 门禁①"高度上限"，单位厘米。超过它认为飞机已经在可信工作区之外。 */
#define VERTICAL_MODE_MAX_HEIGHT_CM 120.0f

/* 门禁②"姿态平稳"的倾角上限，单位度（横滚、俯仰各自判断）。
   超过它 cosθ 倾斜补偿就不再有意义，高度读数不能用。 */
#define VERTICAL_CONTROL_MAX_TILT_DEG 25.0f

/* ── 停手（地面待机）──────────────────────────────────────────
   飞机在地面上时控制器必须什么都不做：油门压最低、内外环冻结。
   不做这件事的后果是确定性的，不是概率性的：
   杆在最低位给的是 −2cm/s，h_sp 被钳到 0，而地面 h_est≈0.8cm，
   外环误差为负 → 内环把油门往下压 → 输出掉到 1100µs 以下
   → 命中"输出 < 1100µs 持续 > 500ms"的失效判据 → 定高在地面上自己退出。 */

/* 停手进入门槛：估计高度不超过这么多厘米，**并且本拍摇杆没有在命令爬升**
   （目标变化率 ≤ 0），且连续满足下面的时间去抖。
   为什么要加"没在命令爬升"这一条：飞机在地面上、飞行员推杆要起飞时，h_est 还是地面读数，
   不加它就会在推杆 500ms 之后被重新判成停手，把刚发出的爬升指令按回去。
   而"被气流或障碍物压到地面、杆还停在中位"那种情况变化率是 0，照样能进——
   这正是当初决定**不**要求"杆必须在最低位"的那个场景。

   ⚠️⚠️ 现在的 6.0 是【台架测试值】，不是最终值。
   2026-09-14 台架把飞机**垫高**了，地面读数从约 1cm 变成约 4cm，
   用原来的 2.0 门槛停手永远进不去（实测 st 一直是 1、gt 一直 0ms）。
   正式起降面的实测读数是约 **1cm**，那时门槛应改回 **2.0**（或 3.0 留余量）。
   ⇒ 垫高台架用 6.0；恢复不垫高用 2.0。改的时候记得同步
      `优化一下/高度接入施工顺序.md` 的待办清单。 */
#define VERTICAL_GROUND_ENTER_H_CM 6.0f

/* 停手进入去抖时间，单位毫秒。
   没有它，起飞擦地时 h_est 瞬时跌破 2cm 会造成"停手 ↔ 控制"约 1.5Hz 的抖动
   （周期 = (4−2)/3 cm/s ≈ 0.67s）。 */
#define VERTICAL_GROUND_ENTER_HOLD_MS 500U

/* 停手退出门槛：目标高度超过这么多厘米就退出停手。
   ⚠️ 必须用 h_sp，**不能**改成 h_est：停手会把油门压到最低，飞机永远离不开地，
   h_est 永远涨不上去，用 h_est 会永久卡死在停手（push 杆也起不来）。
   h_sp 是摇杆积分出来的，停手期间它是唯一还会变的量，也是唯一的出口。
   详见 `优化一下/高度接入飞控方案.md` 第 15.3 节。 */
#define VERTICAL_GROUND_EXIT_H_SP_CM 4.0f

/* 停手退出后的"起飞保护"清零高度，单位厘米。
   ⚠️ 这一条是 H6 写代码时用仿真抓出来的，不加会有一个**飞机永远起飞不了**的死循环：
   退出停手只说明"飞行员命令了爬升"，飞机此刻还在地面上（h_est 仍是 0.8cm）。
   如果不加保护，退出后 500ms 去抖一满就又被判成停手，
   于是 h_sp 被按回 0.8、油门压最低 —— 推杆、被按回、再推杆，来回死循环。
   电池电压低或载重大（推力偏弱、500ms 内离不了 2cm）时必然发生。
   清掉保护的两个条件（任一满足）：
     ① 高度已经明显离地（> 这个值），说明这次起飞真的成功了；
     ② 杆回到下降段（目标变化率 < 0）—— 飞行员主动放弃了这次起飞。 */
#define VERTICAL_TAKEOFF_GUARD_CLEAR_H_CM 10.0f

/* 门禁①（模式切换）的通过位，全通才允许切换。
   ⚠️ 位 0 同时要求"链路正常"：失联时 armed 也是 false，
   光看 armed 会把"空中失联"误判成"在地面上"，从而在半空中把模式切了。 */
#define VERTICAL_GATE_SW_ON_GROUND 0x01U
#define VERTICAL_GATE_SW_STICK_LOW 0x02U
#define VERTICAL_GATE_SW_EST_VALID 0x04U
#define VERTICAL_GATE_SW_HEIGHT_OK 0x08U
#define VERTICAL_GATE_SW_ALL 0x0FU

/* 门禁②（控制启用）的通过位，全通才算控制器真的在干活。 */
#define VERTICAL_GATE_CT_ARMED 0x01U
#define VERTICAL_GATE_CT_EST_VALID 0x02U
#define VERTICAL_GATE_CT_ATT_OK 0x04U
#define VERTICAL_GATE_CT_NOT_GROUND 0x08U
#define VERTICAL_GATE_CT_ALL 0x0FU

/**
 * @brief 实际生效的模式。给遥测 flight_mode 用。数值与协议约定一致，不要改。
 */
typedef enum
{
    /* 普通 / 手动：油门杆直通油门。 */
    VERTICAL_MODE_MANUAL = 0U,
    /* 定高：油门杆三段式。 */
    VERTICAL_MODE_ALT_HOLD = 1U
} VerticalControlMode_t;

/**
 * @brief 高度控制的细分状态。给黑匣子 altitude_state 与串口打印用。
 *
 * 为什么"模式"和"状态"要分开：模式只回答"切没切进定高"，
 * 状态还要回答"控制器有没有真的在干活"。混成一个量，
 * 出问题时分不清是"根本没切进模式"还是"切进去了但门禁没过"。
 */
typedef enum
{
    /* 普通模式，定高完全没参与。 */
    VERTICAL_STATE_MANUAL = 0U,
    /* 定高模式，门禁②全通，控制器在干活。 */
    VERTICAL_STATE_ALT_HOLD = 1U,
    /* 自动起飞（预留，本阶段不会产生）。 */
    VERTICAL_STATE_AUTO_TAKEOFF = 2U,
    /* 自动降落（预留，本阶段不会产生）。 */
    VERTICAL_STATE_AUTO_LAND = 3U,
    /* 定高模式，处于停手（地面待机）。 */
    VERTICAL_STATE_GROUNDED = 4U,
    /* 定高模式已切进去，但门禁②没过，控制器还没动。 */
    VERTICAL_STATE_IDLE = 5U
} VerticalControlState_t;

/**
 * @brief 一次 Update 之后的完整状态，供打印、遥测与黑匣子读取。
 */
typedef struct
{
    /* 遥控请求的模式，取协议 flight_mode 的低 4 位。非 0/1 视为未知，只拒绝不切换。 */
    uint8_t requested_mode;
    /* 实际生效的模式，取 VerticalControlMode_t。 */
    uint8_t mode;
    /* 细分状态，取 VerticalControlState_t。 */
    uint8_t state;
    /* 门禁①的通过位，VERTICAL_GATE_SW_xxx 或起来。等于 VERTICAL_GATE_SW_ALL 才算通过。 */
    uint8_t switch_gates;
    /* 门禁②的通过位，VERTICAL_GATE_CT_xxx 或起来。等于 VERTICAL_GATE_CT_ALL 才算通过。 */
    uint8_t control_gates;
    /* 直接给遥测用的模式字节：实际模式；请求与实际不一致（被拒）时或上 0x80。 */
    uint8_t telemetry_mode;
    /* true 表示处于停手（地面待机）。 */
    bool grounded;
    /* true 表示高度估计可信，转抄估计器的 valid。 */
    bool est_valid;
    /* 本拍油门杆原始量，0～1000（超过 1000 的按 1000 处理）。 */
    uint16_t stick;
    /* 本拍摇杆给出的目标高度变化率，单位厘米每秒，向上为正。 */
    float stick_rate_cms;
    /* 本拍目标高度，单位厘米。停手期间仍随杆积分——它是停手的唯一出口。 */
    float h_sp_cm;
    /* 本拍估计高度，单位厘米。只用来打印时和 h_sp 对齐；
       本模块拿它做判断的唯一地方是"停手进入"，那里会先看 est_valid。 */
    float h_est_cm;
    /* 停手进入条件的累计满足时间，单位毫秒；0 表示计时未开始。 */
    uint32_t ground_timer_ms;
} VerticalControlSnapshot_t;

/**
 * @brief 初始化。上电创建任务时调用一次，IMU 重新初始化时也用它做冷复位。
 *
 * 与 Reset 的区别：Init 把模式也清回普通，Reset 保留模式。
 */
void VerticalControl_Init(void);

/**
 * @brief 复位控制器的运行状态，但**不动模式**。
 *
 * 复位内容：目标高度对齐到当前估计高度、停手计时清零、停手标志清除。
 * 调用时机：① IMU 重新初始化；② **模式切换成功的那一刻**。
 *
 * ② 是必须的：进模式时如果 h_sp 还留着上一轮的旧值，
 * 解除停手的瞬间会朝着一个完全不相干的高度产生巨大的跳变指令。
 * 对齐到"当前高度"是标准做法——设定值等于当前值，进来就没有误差。
 */
void VerticalControl_Reset(void);

/**
 * @brief 推进一拍。控制环每拍调用一次，**上锁时也必须调用**。
 *
 * ⚠️ 必须在 `if (!armed)` 分支**之前**调用：模式切换的前置条件就是"已上锁"，
 * 把调用塞进解锁分支里，上锁时就永远切不进模式。
 * ⚠️ 必须在 `VerticalEst_Update()` **之后**调用：它内部要读估计器的最新结果。
 *
 * @param requested_mode 遥控请求的模式，取协议 flight_mode 的低 4 位。
 * @param throttle_stick 油门杆原始量，0～1000。
 * @param link_ok 链路正常且指令有效。失联时 armed 也是 false，必须单独传进来，
 *                否则门禁①会把"空中失联"误判成"在地面上"。
 * @param armed 当前是否已解锁（与写电机用的是同一个判据）。
 * @param roll_deg 机体系横滚角，单位度，用于门禁②的姿态判据。
 * @param pitch_deg 机体系俯仰角，单位度，同上。
 * @param dt_s 距上一拍的时间，单位秒，正常情况下约 0.004。数值离谱时按 0 处理，不积分。
 */
void VerticalControl_Update(uint8_t requested_mode, uint16_t throttle_stick, bool link_ok, bool armed, float roll_deg, float pitch_deg, float dt_s);

/**
 * @brief 取最近一次结果。只拷贝一个结构体，不做任何计算，可在低频任务里调用。
 * @param snapshot 输出，不能为空。
 */
void VerticalControl_GetSnapshot(VerticalControlSnapshot_t *snapshot);

/**
 * @brief 【临时，H6 验证完删除】覆盖遥控请求的模式，供台架用串口命令 `vm` 切换模式。
 *
 * 为什么需要它：遥控端 flight_mode 现在硬编码 1，台架上没法在 0 与 1 之间来回切，
 * 而"请求被拒"和"请求被接受"两种情况都要验。
 * 本函数由黑匣子任务的串口命令解析调用，写的是一个 volatile 单字节，天然原子。
 *
 * @param mode 0 = 强制请求普通，1 = 强制请求定高，0xFF = 取消覆盖（回到遥控实际发来的值）。
 */
void VerticalControl_SetDebugRequestedMode(uint8_t mode);

#ifdef __cplusplus
}
#endif

#endif
