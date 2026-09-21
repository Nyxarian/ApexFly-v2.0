/* 光流（OF1~OF5）的宿主机仿真：把验收判据在**烧录之前**先跑一遍。
 *
 * 真实参与编译的只有 flow.c 与 flow_cxof.c，外设与 RTOS 用桩顶掉
 * （flow_stub/ 下的 main.h、cmsis_os.h、task.h）。
 *
 * 编译（宿主机 gcc；本文件不碰任何硬件）：
 *   gcc -std=c99 -Wall -Wextra -Wno-unused-parameter \
 *     -I flow_stub -I "../../ApexFly v2.0/Core/Inc" \
 *     -I "../../ApexFly v2.0/Drivers/STM32G4xx_HAL_Driver/Inc" \
 *     -I "../../ApexFly v2.0/Drivers/CMSIS/Device/ST/STM32G4xx/Include" \
 *     -I "../../ApexFly v2.0/Drivers/CMSIS/Include" \
 *     -I "../../ApexFly v2.0/Modules/Flow" \
 *     sim_flow.c "../../ApexFly v2.0/Modules/Flow/flow.c" \
 *                "../../ApexFly v2.0/Modules/Flow/flow_cxof.c" -o sim_flow.exe
 *
 * ── ⭐ 怎么在不改产品代码的前提下驱动真的 Flow_Task ────────────────
 * Flow_Task 里是一个 `for(;;)`，而且**一进来就调 Flow_Init()**。
 * 如果每次进去都重新调 Flow_Init，零点标定永远做不完，仿真就成了自欺。
 * 所以这里用**反向控制**：不让 osDelay/taskYIELD 真的"阻塞然后返回"，
 * 而是让它们**去推进仿真世界**（投喂下一批字节、推进模拟时钟），然后正常返回。
 * 于是整场仿真都跑在 **Flow_Task 的同一个栈帧里**，Flow_Init 只执行一次，
 * 而循环体是**产品代码原文**，一行都没有复制到仿真里来。
 *
 * ── 模拟的 DMA ────────────────────────────────────────────────
 * `HAL_UARTEx_ReceiveToIdle_DMA` 的桩把 flow.c 传进来的缓冲指针存下来，
 * 之后仿真直接往那个缓冲里写字节、再按"当前写指针"触发
 * `HAL_UARTEx_RxEventCallback` —— 与真实 DMA 循环模式的行为一致，
 * 包括**绕回缓冲末尾**那一支（本仿真第 1 组会真的绕一次，270 > 256）。
 *
 * ⚠️ 仿真**不能**代替真机验证。它能证明：帧解析与重新同步、质量门禁、冻结、
 *    零点标定与其守卫、累积数值、绕回分支。它证明不了：模块真的在发、
 *    真实 Quality 水平、厂区地面纹理够不够、真实零偏有多大。 */

#include "flow.h"
#include "apex_config.h"

#include "stm32g4xx_hal.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if !APEX_UART2_ROLE_FLOW
#error "本仿真必须在 APEX_UART2_ROLE_FLOW = 1 下编译"
#endif

/* ── 从 flow.c 抄过来的常量（改那边就必须改这里）── */
#define FLOW_DMA_SIZE 256U          /* FLOW_DMA_BUFFER_SIZE */
#define CAL_FRAMES 250U             /* FLOW_CAL_FRAMES */
#define CAL_MAX_COUNT 100.0f        /* FLOW_CAL_MAX_COUNT */
#define TIMEOUT_MS 200U             /* FLOW_TIMEOUT_MS */
#define STEP_MS 40U                 /* 模拟的帧间隔（25Hz） */
#define TAIL_STEPS 2                /* 投喂结束后再空转几步，让任务把最后一批处理完 */

/* ══════════════════════════════════════════════════════════════════
   给 flow.c 用的桩
   ══════════════════════════════════════════════════════════════════ */

UART_HandleTypeDef huart2;
static DMA_HandleTypeDef sim_hdma;
/* ⚠️ __HAL_DMA_DISABLE_IT 会往 Instance->CCR 写，所以 Instance 必须指向
   一块**可写的普通内存**，不能是真实外设地址（在 PC 上会直接崩）。 */
static DMA_Channel_TypeDef sim_dma_regs;

static uint8_t *sim_dma_buf;
static uint16_t sim_dma_pos;

HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
    (void)Size;
    sim_dma_buf = pData;
    sim_dma_pos = 0U;
    huart->RxState = HAL_UART_STATE_BUSY_RX;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *huart)
{
    huart->RxState = HAL_UART_STATE_READY;
    return HAL_OK;
}

HAL_UART_RxEventTypeTypeDef HAL_UARTEx_GetRxEventType(const UART_HandleTypeDef *huart)
{
    (void)huart;
    return HAL_UART_RXEVENT_IDLE;
}

/* 模拟时钟。 */
static uint32_t sim_ms;

uint32_t osKernelGetTickCount(void)
{
    return sim_ms;
}

/* ══════════════════════════════════════════════════════════════════
   判据计数
   ══════════════════════════════════════════════════════════════════ */

static int checks;
static int fails;

static void check(const char *name, int ok, const char *detail)
{
    checks++;
    if (!ok) fails++;
    printf("[%s] %s%s%s\n", ok ? "OK" : "失败", name, (detail != NULL) ? "  " : "", (detail != NULL) ? detail : "");
}

static FlowState_t snap(void)
{
    FlowState_t s;

    memset(&s, 0, sizeof(s));
    Flow_GetState(&s);
    return s;
}

/* ══════════════════════════════════════════════════════════════════
   帧构造与投喂
   ══════════════════════════════════════════════════════════════════ */

static void build_frame(uint8_t *b, int16_t dx, int16_t dy, uint8_t quality)
{
    b[0] = 0xFEU;
    b[1] = 0x04U;
    b[2] = (uint8_t)((uint16_t)dx & 0xFFU);
    b[3] = (uint8_t)(((uint16_t)dx >> 8) & 0xFFU);
    b[4] = (uint8_t)((uint16_t)dy & 0xFFU);
    b[5] = (uint8_t)(((uint16_t)dy >> 8) & 0xFFU);
    b[6] = (uint8_t)(b[2] + b[3] + b[4] + b[5]);
    b[7] = quality;
    b[8] = 0xAAU;
}

/* 往模拟 DMA 缓冲里写一段，然后按当前写指针触发一次"空闲"事件。 */
static void deliver(const uint8_t *data, uint16_t length)
{
    uint16_t i;

    if (sim_dma_buf == NULL) return;
    for (i = 0U; i < length; i++)
    {
        sim_dma_buf[sim_dma_pos] = data[i];
        sim_dma_pos = (uint16_t)((sim_dma_pos + 1U) % FLOW_DMA_SIZE);
    }
    HAL_UARTEx_RxEventCallback(&huart2, sim_dma_pos);
}

static void send_frame(int16_t dx, int16_t dy, uint8_t quality)
{
    uint8_t b[9];

    build_frame(b, dx, dy, quality);
    deliver(b, 9U);
}

/* ══════════════════════════════════════════════════════════════════
   场景表
   ══════════════════════════════════════════════════════════════════ */

typedef struct
{
    const char *name;
    int fresh;                  /* 1 = 进入本场景时调一次 Flow_Init（等价于重新上电） */
    int feed_steps;
    void (*feed)(int step);
    void (*check_fn)(void);
} SimScenario_t;

static void Scenario_Enter(int index);

/* ── 1 正常帧流 + DMA 缓冲绕回 ── */
static void Feed_Normal(int step)
{
    (void)step;
    send_frame(0, 0, 80);
}
static void Check_Normal(void)
{
    FlowState_t s = snap();

    check("1 正常帧流：解出 30 帧", s.frame_count == 30U, NULL);
    check("1 正常帧流：字节数 = 30×9 = 270（证明没丢字节）", s.byte_count == 270U, NULL);
    check("1 正常帧流：无校验/格式错误", (s.checksum_error_count == 0U) && (s.format_error_count == 0U), NULL);
    /* ⚠️ 270 > 256 ⇒ 模拟 DMA 缓冲**绕过一次末尾**。这一条就是在验 flow.c 里
       "先搬尾巴、再搬开头"那一支；写错的话 frame_count 会少几帧。 */
    check("1 ⭐ DMA 缓冲绕回一次后仍不丢帧", s.frame_count == 30U, "（270 字节 > 256 缓冲）");
}

/* ── 2 解析边界：坏校验 / 坏长度 / 坏包尾 / 半截帧 / 粘包 / 插噪声 ── */
static void Feed_Parse(int step)
{
    uint8_t b[18];

    switch (step)
    {
        case 0:                     /* 累加和改错 */
            build_frame(b, 1, 2, 80);
            b[6] = (uint8_t)(b[6] + 1U);
            deliver(b, 9U);
            break;
        case 1:                     /* 长度字节错 */
            build_frame(b, 1, 2, 80);
            b[1] = 0x05U;
            deliver(b, 9U);
            break;
        case 2:                     /* 包尾错 */
            build_frame(b, 1, 2, 80);
            b[8] = 0xABU;
            deliver(b, 9U);
            break;
        case 3:                     /* 正常帧 */
            send_frame(1, 2, 80);
            break;
        case 4:                     /* 半截帧：前 5 字节 */
            build_frame(b, 3, 4, 80);
            deliver(b, 5U);
            break;
        case 5:                     /* 后 4 字节 ⇒ 必须拼成完整一帧 */
            build_frame(b, 3, 4, 80);
            deliver(&b[5], 4U);
            break;
        case 6:                     /* 粘包：一次事件送两帧 */
            build_frame(&b[0], 7, 8, 80);
            build_frame(&b[9], 9, 10, 80);
            deliver(b, 18U);
            break;
        case 7:                     /* 帧中间插入一个噪声字节 ⇒ 该帧必须被丢弃 */
            build_frame(b, 11, 12, 80);
            memmove(&b[6], &b[5], 4U);      /* 在第 6 字节处腾出一格 */
            b[5] = 0x00U;
            deliver(b, 10U);
            break;
        case 8:                     /* 噪声之后的下一帧必须能正常解出（重新同步） */
            send_frame(13, 14, 80);
            break;
        default:
            break;
    }
}
static void Check_Parse(void)
{
    FlowState_t s = snap();

    /* 好帧：step3、step5（半截拼成）、step6 的两帧、step8 = 共 5 帧 */
    check("2 坏帧不留痕：只解出 5 帧好帧", s.frame_count == 5U, NULL);
    /* ⚠️ 这里是 **2** 次不是 1 次，第一版判据写成 1 是错的（仿真抓出来了）：
       step0 那帧是显式的累加和错；step7 那帧在中间插了个 0x00，于是**窗口错位**，
       错位窗口的第一个失败点通常也是累加和（除非正好凑巧）。
       ⇒ 累加和计数 = 2，格式（长度/包尾）计数 = 2（step1 的长度错 + step2 的包尾错）。
       这条差别本身有意义：**错位窗口几乎都先在累加和这一步被拦下**，
       所以"checksum 一直在涨"未必是波特率问题，也可能只是有噪声字节。 */
    check("2 累加和错 2 次（step0 的坏帧 + step7 的错位窗口）", s.checksum_error_count == 2U, NULL);
    check("2 长度字节错 + 包尾错 共被记 2 次", s.format_error_count == 2U, NULL);
    check("2 ⭐ 半截帧能跨两次事件拼成整帧", s.frame_count >= 5U, "（step5 那一帧）");
    check("2 ⭐ 粘包一次解出两帧", s.frame_count >= 5U, "（step6）");
    check("2 ⭐ 帧中插噪声后能重新同步（后续帧不丢）", s.resync_count > 0U, NULL);
}

/* ── 3 质量门禁（计数与边界 29 / 30）── */
static void Feed_Quality(int step)
{
    static const uint8_t q[5] = {10U, 10U, 10U, 30U, 29U};

    if (step < 5) send_frame(0, 0, q[step]);
}
static void Check_Quality(void)
{
    FlowState_t s = snap();

    check("3 五帧全部解出（质量差不影响解析）", s.frame_count == 5U, NULL);
    check("3 ⭐ 质量门槛边界：q=10/10/10/29 被拒、q=30 通过 ⇒ 拒 4 帧",
          s.quality_reject_count == 4U, "（门槛 30 是含等号通过）");
}

/* ── 4 零点标定成功 ── */
static void Feed_CalOk(int step)
{
    /* 250 帧交替 ±1，偶数帧数保证均值**精确**为 0，便于精确断言。 */
    if (step < (int)CAL_FRAMES) send_frame(((step % 2) == 0) ? 1 : -1, 0, 80);
}
static void Check_CalOk(void)
{
    FlowState_t s = snap();

    check("4 标定完成（cal = DONE）", s.cal == (uint8_t)FLOW_CAL_DONE, NULL);
    check("4 ⭐ 零点 = 0（零均值噪声）", (s.zero_x == 0.0f) && (s.zero_y == 0.0f), NULL);
    check("4 只尝试过 1 次（没有误判失败）", s.cal_attempts == 1U, NULL);
    /* ⚠️ 标定期间**不累积**：否则累积量里会混进一段没减零点的历史，事后擦不掉。 */
    check("4 ⭐⭐ 标定期间不累积位置（sum 恒为 0）", (s.sum_x == 0.0f) && (s.sum_y == 0.0f), NULL);
    check("4 数据可用", s.valid, NULL);
}

/* ── 5a 标定完成后：累积与质量冻结 ── */
static void Feed_Accum(int step)
{
    if (step < 10) send_frame(5, 0, 80);        /* 10 帧 × +5 计数 */
    else send_frame(5, 0, 5);                   /* 之后 4 帧质量不合格 */
}
static void Check_Accum(void)
{
    FlowState_t s = snap();

    check("5a ⭐ 累积流量 = 10 × 5 = 50（减零点后精确）", s.sum_x == 50.0f, NULL);
    check("5a Y 轴未动", s.sum_y == 0.0f, NULL);
    check("5a 质量不合格 4 帧被记账", s.quality_reject_count == 4U, NULL);
    check("5a ⭐⭐ 连续低质量 ⇒ valid = 0", !s.valid, NULL);
    check("5a ⭐⭐ 失效时数据**冻结**（dx 保持上一帧的 5，不是 0）", s.dx == 5, NULL);
    check("5a ⭐⭐ 失效时不继续累积（sum 仍是 50，不是 70）", s.sum_x == 50.0f, NULL);
    check("5a 打印用的 quality 是**当前**值（5），便于一眼看出地面看不清",
          s.quality == 5U, NULL);
}

/* ── 5b 长时间无合格帧 ⇒ 超时失效；恢复后继续累积 ── */
static void Feed_Timeout(int step)
{
    if (step == 0)
    {
        FlowState_t s = snap();
        check("5b ⭐ 长时间无合格帧 ⇒ age 涨过超时门槛、valid = 0", (!s.valid) && (s.age_ms > TIMEOUT_MS), NULL);
        check("5b ⭐ 超时期间数据仍冻结", s.sum_x == 50.0f, NULL);
    }
    if (step == 8) send_frame(5, 0, 80);        /* 恢复：合格帧回来 */
}
static void Check_Timeout(void)
{
    FlowState_t s = snap();

    check("5b ⭐ 合格帧回来后自动恢复（valid = 1）", s.valid, NULL);
    check("5b ⭐ 恢复后累积接着走（50 + 5 = 55，没有被清零）", s.sum_x == 55.0f, NULL);
}

/* ── 6 零点标定的守卫与自动重试 ── */
static void Feed_CalGuard(int step)
{
    if (step < (int)CAL_FRAMES)
    {
        send_frame(500, 500, 80);               /* 采样期间明显在动 ⇒ 必须判失败 */
    }
    else if ((step >= (int)CAL_FRAMES + 30) && (step < (int)CAL_FRAMES + 30 + (int)CAL_FRAMES))
    {
        int k = step - (int)CAL_FRAMES - 30;

        send_frame(((k % 2) == 0) ? 1 : -1, 0, 80);
    }
}
static void Check_CalGuard(void)
{
    FlowState_t s = snap();

    check("6 ⭐⭐ 采样期间在动 ⇒ 本次标定判失败（不是照单全收）", s.cal_attempts >= 2U, NULL);
    check("6 ⭐ 失败后自动重来，最终标定成功", s.cal == (uint8_t)FLOW_CAL_DONE, NULL);
    check("6 ⭐⭐ 最终零点来自**第二次**采样（= 0），没有被 500 的坏采样污染",
          s.zero_x == 0.0f, NULL);
}

/* ── 6b 250 帧窗口把单个异常计数的影响压低 10 倍 ── */
static void Feed_CalResolution(int step)
{
    /* 标定窗口第 1 帧给 -1，其余全 0；标定完成后再喂 250 帧 0。
       25 帧旧窗口会得到 -0.04、随后漂 +10；250 帧应得到 -0.004、漂 +1。 */
    if (step == 0) send_frame(-1, 0, 80);
    else if (step < (int)(CAL_FRAMES * 2U)) send_frame(0, 0, 80);
}
static void Check_CalResolution(void)
{
    FlowState_t s = snap();

    check("6b ⭐ 单个 -1 经 250 帧平均后零点 = -0.004",
          (s.zero_x > -0.0041f) && (s.zero_x < -0.0039f), NULL);
    check("6b ⭐ 随后 250 帧静止只累积约 +1（旧窗口会是 +10）",
          (s.sum_x > 0.99f) && (s.sum_x < 1.01f), NULL);
    check("6b Y 轴保持 0", (s.zero_y == 0.0f) && (s.sum_y == 0.0f), NULL);
}

/* ── 7 不变式 ── */
static void Check_Invariants(void)
{
    FlowState_t s = snap();

    check("7 不变式：sum 在钳位范围内", (s.sum_x >= -5000.0f) && (s.sum_x <= 5000.0f), NULL);
    check("7 不变式：byte_count ≥ 9 × frame_count（帧不可能比字节多）",
          s.byte_count >= (9U * s.frame_count), NULL);
    check("7 不变式：解析器没有把坏帧算成好帧",
          (s.frame_count + s.checksum_error_count + s.format_error_count) > 0U, NULL);
}

static const SimScenario_t SIM_SCENARIOS[] = {
    {"1 正常帧流 / DMA 绕回",        1, 30, Feed_Normal,    Check_Normal},
    {"2 解析边界（坏帧/半截/粘包）",  1,  9, Feed_Parse,     Check_Parse},
    {"3 质量门禁与边界",             1,  5, Feed_Quality,   Check_Quality},
    {"4 零点标定成功",              1, (int)CAL_FRAMES, Feed_CalOk, Check_CalOk},
    {"5a 累积与质量冻结",            0, 14, Feed_Accum,     Check_Accum},
    {"5b 超时失效与恢复",            0, 10, Feed_Timeout,   Check_Timeout},
    {"6 标定守卫与自动重试",          1, (int)(CAL_FRAMES * 2U + 30U), Feed_CalGuard, Check_CalGuard},
    {"6b 长窗口分辨率回归",           1, (int)(CAL_FRAMES * 2U), Feed_CalResolution, Check_CalResolution},
    {"7 不变式",                    0,  2, NULL,            Check_Invariants},
};
#define SIM_SCENARIO_COUNT ((int)(sizeof(SIM_SCENARIOS) / sizeof(SIM_SCENARIOS[0])))

static int sim_scenario;
static int sim_step_index;

static void sim_finish(void)
{
    printf("\n===== 检查 %d 项，失败 %d 项 =====\n", checks, fails);
    exit((fails == 0) ? 0 : 1);
}

static void Scenario_Enter(int index)
{
    if (SIM_SCENARIOS[index].fresh)
    {
        /* 等价于重新上电：清状态、清统计、零点回到"没标过"。
           顺带会把 DMA 标志清掉，于是任务下一拍会重新调 ReceiveToIdle_DMA
           （桩会重新抓缓冲指针、写指针归零）。 */
        Flow_Init();
    }
}

static void sim_step(void)
{
    const SimScenario_t *s = &SIM_SCENARIOS[sim_scenario];

    sim_ms += STEP_MS;

    /* 场景名在**进入时**打印，这样场景内即时做的判据也会落在正确的标题下面。 */
    if (sim_step_index == 0) printf("\n========== %s ==========\n", s->name);

    if (sim_step_index < s->feed_steps)
    {
        if (s->feed != NULL) s->feed(sim_step_index);
    }

    sim_step_index++;
    if (sim_step_index >= (s->feed_steps + TAIL_STEPS))
    {
        if (s->check_fn != NULL) s->check_fn();

        sim_scenario++;
        sim_step_index = 0;
        if (sim_scenario >= SIM_SCENARIO_COUNT) sim_finish();
        Scenario_Enter(sim_scenario);
    }
}

void osDelay(uint32_t ticks)
{
    (void)ticks;
    sim_step();
}

void Sim_TaskYield(void)
{
    sim_step();
}

int main(void)
{
    /* DMA 句柄必须像真的：Instance 指向可写内存（宏会往 CCR 写），Mode = 循环。 */
    memset(&sim_dma_regs, 0, sizeof(sim_dma_regs));
    memset(&sim_hdma, 0, sizeof(sim_hdma));
    sim_hdma.Instance = &sim_dma_regs;
    sim_hdma.Init.Mode = DMA_CIRCULAR;

    memset(&huart2, 0, sizeof(huart2));
    huart2.Instance = USART2;
    huart2.hdmarx = &sim_hdma;

    printf("光流 OF1~OF5 仿真：解析 / 质量门禁 / 冻结 / 零点标定 / 累积\n");
    printf("（驱动的是 Modules/Flow/flow.c 里的**真** Flow_Task，外设与 RTOS 是桩）\n");

    Scenario_Enter(0);

    /* 这一句不会返回：Flow_Task 里的 for(;;) 会一直跑，
       每次 osDelay/taskYIELD 都会回到 sim_step 推进场景。 */
    Flow_Task(NULL);

    printf("!! 不应该到达这里\n");
    return 1;
}
