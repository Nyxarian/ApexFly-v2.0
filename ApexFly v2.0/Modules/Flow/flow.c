/**
 * @file flow.c
 * @brief CX-OF 光流的串口接收、有效性判定、零点标定与累积流量实现。
 *
 * ── 本模块这一阶段**不接任何控制** ────────────────────────────
 * 只产出一份给打印看的快照。不写姿态、不写电机、不碰油门、不碰定高。
 *
 * ── 收数据的方式照抄 gps.c（成熟做法，不另起炉灶）──────────────
 * `HAL_UARTEx_ReceiveToIdle_DMA` + DMA 循环模式 + 环形缓冲：
 * 中断里只把 DMA 缓冲里**新到的那一段**搬进环形缓冲，其余全在任务里做。
 * 这样中断路径短且确定，也不在中断里做解析。
 *
 * ⚠️ 与 GPS 的**互斥关系**（这是本文件最容易出人命的一处）：
 *   USART2 同一时刻只能归一个人用，所以本文件里那两个 HAL 回调
 *   （`HAL_UARTEx_RxEventCallback` / `HAL_UART_ErrorCallback`）与
 *   `Modules/GPS/gps.c` 里的同名函数**必须互斥编译**，两边都用
 *   `APEX_UART2_ROLE_FLOW` 这个开关控制。同时定义会报
 *   `L6200E: Symbol HAL_UART_ErrorCallback multiply defined`。
 *   2026-09 做串口回显测试时同一个符号已经踩过一次，别再来第二次。
 */

#include "flow.h"

#include "apex_config.h"
#include "cmsis_os.h"
#include "flow_cxof.h"
#include "task.h"
#include "usart.h"

#include <stddef.h>

/* DMA 缓冲大小。25Hz × 9 字节 ≈ 225 字节/秒，256 已经非常宽松。 */
#define FLOW_DMA_BUFFER_SIZE 256U
/* 环形缓冲大小，取 2 的幂以便用掩码代替取模。 */
#define FLOW_RING_BUFFER_SIZE 256U
#define FLOW_RING_BUFFER_MASK (FLOW_RING_BUFFER_SIZE - 1U)

/* 任务轮询周期，单位毫秒。取 10ms 是为了读得比 DMA 慢不了多少，同时让出 CPU。 */
#define FLOW_POLL_MS 10U
/* 超过这么久没有**合格**帧就判失效，单位毫秒。25Hz 下相当于连丢 5 帧才报警。 */
#define FLOW_TIMEOUT_MS 200U
/* 连续这么多帧质量不达标才判失效。单帧质量抖动是常态，不该立刻失效。 */
#define FLOW_QUALITY_BAD_MAX 3U
/* 零点标定的采样帧数。25Hz 下 250 帧 = 10 秒。
   真机留样（2026-09-21）确认：25 帧窗口里只要偶发 1 个 ±1 计数，
   均值就会变成 ±0.04 计数/帧；之后即使原始值全为 0，也会被减零点
   主动积成约 1 计数/秒的单向漂移。拉长 10 倍后，单个异常计数的
   影响降到 0.004 计数/帧，而且与电调 + IMU 的上电准备时间重叠。 */
#define FLOW_CAL_FRAMES 250U
/* 标定采样的合法性上限，单位计数/帧。
   ⚠️ **占位值**：真实零偏有多大还没实测过（见方案 §8 第 4 条）。
   取这个数只是为了挡住"采样时明显在动"（被人拿着、或者放在振动的台面上），
   宁可放宽也不要收紧 —— 太紧会把好标定判成失败。第一次实测后按实测零偏的 3~5 倍重定。 */
#define FLOW_CAL_MAX_COUNT 100.0f
/* 标定失败后隔多久自动重来，单位毫秒。不需要重新上电。 */
#define FLOW_CAL_RETRY_MS 1000U
/* 累积流量钳位，单位计数。
   ⚠️ **占位值**：真实"1 计数 = 多少位移"还没实测，所以这个上限现在只能说"足够大、
   不至于因为长时间无反馈而积分到溢出"。OF6 算出真实尺度后要重新定。 */
#define FLOW_SUM_LIMIT_COUNTS 5000.0f
/* 算流量率时的最小时间间隔，单位毫秒。小于它说明这一拍 dt 不可信，保持上一次的速率。 */
#define FLOW_RATE_DT_MIN_MS 1U

static FlowCxofDevice_t flow_device;
static FlowState_t flow_snapshot;

static uint8_t flow_dma_buffer[FLOW_DMA_BUFFER_SIZE];
static uint8_t flow_ring_buffer[FLOW_RING_BUFFER_SIZE];

static volatile uint16_t flow_ring_head;
static volatile uint16_t flow_ring_tail;
static volatile uint16_t flow_dma_last_position;
static volatile bool flow_dma_running;
static volatile bool flow_dma_restart_required;
static volatile uint32_t flow_uart_error_count;
static volatile uint32_t flow_dma_drop_count;

/* ── 零点标定 ────────────────────────────────────────────────── */
static FlowCalState_t flow_cal_state;
static uint32_t flow_cal_frame_count;
static float flow_cal_sum_x;
static float flow_cal_sum_y;
static uint32_t flow_cal_retry_tick;
static float flow_zero_x;
static float flow_zero_y;

/* ── 帧节拍与失效判定 ────────────────────────────────────────── */
static uint32_t flow_last_good_tick;
static uint32_t flow_last_frame_tick;
static uint8_t flow_quality_bad_streak;
static bool flow_ever_got_frame;

/** @brief 取绝对值。不用 fabsf 是为了不引入 libm 依赖。 */
static float Flow_Absf(float value)
{
    return (value < 0.0f) ? -value : value;
}

static void Flow_RingPushFromISR(const uint8_t *data, uint16_t length)
{
    uint16_t index;

    if (data == NULL) return;
    for (index = 0U; index < length; index++)
    {
        uint16_t next_head = (uint16_t)((flow_ring_head + 1U) & FLOW_RING_BUFFER_MASK);
        if (next_head == flow_ring_tail)
        {
            flow_dma_drop_count++;
            continue;
        }
        flow_ring_buffer[flow_ring_head] = data[index];
        flow_ring_head = next_head;
    }
}

static bool Flow_RingPop(uint8_t *byte)
{
    if ((byte == NULL) || (flow_ring_tail == flow_ring_head)) return false;
    *byte = flow_ring_buffer[flow_ring_tail];
    flow_ring_tail = (uint16_t)((flow_ring_tail + 1U) & FLOW_RING_BUFFER_MASK);
    return true;
}

static HAL_StatusTypeDef Flow_StartDmaReception(void)
{
    HAL_StatusTypeDef status;

    if (huart2.hdmarx == NULL) return HAL_ERROR;

    flow_dma_last_position = 0U;
    status = HAL_UARTEx_ReceiveToIdle_DMA(&huart2, flow_dma_buffer, FLOW_DMA_BUFFER_SIZE);
    if (status == HAL_OK)
    {
        /* 关掉半传输中断：只要"空闲"事件，不要每个半缓冲都进一次中断。 */
        __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
        flow_dma_running = true;
    }
    return status;
}

/**
 * @brief 处理一帧（已通过四重校验）。
 *
 * 里面有三段，顺序不能换：
 *   ① 质量不合格 → 记账后**直接返回**（数据保持上一次 = 冻结）；
 *   ② 零点标定没完成 → 只喂给标定器求平均，**不累积位置**；
 *   ③ 标定完成 → 减零点、累积、算流量率。
 * ② 必须在 ③ 之前：标定没做完就累积，累积量里会混进一段"没减零点"的历史，
 * 那个偏置会一直留在积分里，事后减零点也擦不掉。
 */
static void Flow_OnFrame(const FlowCxofFrame_t *frame)
{
    uint32_t now = osKernelGetTickCount();
    uint32_t dt_ms = now - flow_last_frame_tick;

    flow_snapshot.quality = frame->quality;
    flow_last_frame_tick = now;

    /* ① 质量不够：本帧作废。**冻结**而不是清零——清零会被下游读成"飞机完全静止"。 */
    if (!frame->quality_ok)
    {
        if (flow_quality_bad_streak < 255U) flow_quality_bad_streak++;
        return;
    }

    flow_quality_bad_streak = 0U;
    flow_last_good_tick = now;
    flow_ever_got_frame = true;

    flow_snapshot.dx = frame->dx;
    flow_snapshot.dy = frame->dy;
    flow_snapshot.cal = (uint8_t)flow_cal_state;

    /* ② 标定采样期（或标定失败等重试）：只求平均，不累积。 */
    if (flow_cal_state != FLOW_CAL_DONE)
    {
        if (flow_cal_state == FLOW_CAL_RUNNING)
        {
            flow_cal_sum_x += (float)frame->dx;
            flow_cal_sum_y += (float)frame->dy;
            flow_cal_frame_count++;

            if (flow_cal_frame_count >= FLOW_CAL_FRAMES)
            {
                float mean_x = flow_cal_sum_x / (float)FLOW_CAL_FRAMES;
                float mean_y = flow_cal_sum_y / (float)FLOW_CAL_FRAMES;

                flow_snapshot.cal_attempts++;
                if ((Flow_Absf(mean_x) > FLOW_CAL_MAX_COUNT) || (Flow_Absf(mean_y) > FLOW_CAL_MAX_COUNT))
                {
                    /* 采样期间飞机在动（被拿着、或放在振动的台面上）⇒ 本次标定作废。
                       宁可重来：一个坏的零点会变成**持续的漂移源**，比不标定更糟。 */
                    flow_cal_state = FLOW_CAL_FAILED;
                    flow_cal_retry_tick = now + FLOW_CAL_RETRY_MS;
                }
                else
                {
                    flow_zero_x = mean_x;
                    flow_zero_y = mean_y;
                    flow_snapshot.zero_x = mean_x;
                    flow_snapshot.zero_y = mean_y;
                    flow_cal_state = FLOW_CAL_DONE;
                }

                flow_cal_frame_count = 0U;
                flow_cal_sum_x = 0.0f;
                flow_cal_sum_y = 0.0f;
            }
        }

        flow_snapshot.cal = (uint8_t)flow_cal_state;
        return;
    }

    /* ③ 正常累积：减掉零点之后才是"飞机真的动了多少"。 */
    {
        float counts_x = (float)frame->dx - flow_zero_x;
        float counts_y = (float)frame->dy - flow_zero_y;

        /* ⚠️ 这里直接累加**计数**，**不乘 dt**。
           依据：DX/DY 的定义是"距上次更新以来的位移计数"——它本身已经是一帧的位移量，
           不是速度；再乘一次帧间隔等于把时间权重算了两遍。
           （商家 demo 写的是 `sum += vx * 0.05f`，把每帧计数当成速度又乘了 0.05s；
             在恒定帧率下那只是差一个常数比例，方向不受影响，所以他们的定点也能跑。
             但如果帧率有抖动，乘 dt 会给慢帧额外的权重，反而不准。）
           位置环（OF6）要的反馈量就是它，目标是把它压回 0 —— 所以尺度不知道也能闭环。 */
        flow_snapshot.sum_x += counts_x;
        flow_snapshot.sum_y += counts_y;
        if (flow_snapshot.sum_x > FLOW_SUM_LIMIT_COUNTS) flow_snapshot.sum_x = FLOW_SUM_LIMIT_COUNTS;
        if (flow_snapshot.sum_x < -FLOW_SUM_LIMIT_COUNTS) flow_snapshot.sum_x = -FLOW_SUM_LIMIT_COUNTS;
        if (flow_snapshot.sum_y > FLOW_SUM_LIMIT_COUNTS) flow_snapshot.sum_y = FLOW_SUM_LIMIT_COUNTS;
        if (flow_snapshot.sum_y < -FLOW_SUM_LIMIT_COUNTS) flow_snapshot.sum_y = -FLOW_SUM_LIMIT_COUNTS;

        /* 瞬时流量率（计数/秒），给速度环（OF6）用。
           dt 不合理（同一毫秒内两帧，25Hz 下不该发生）时保持上一次的值。 */
        if (dt_ms >= FLOW_RATE_DT_MIN_MS)
        {
            float dt_s = (float)dt_ms / 1000.0f;
            flow_snapshot.vx_cps = counts_x / dt_s;
            flow_snapshot.vy_cps = counts_y / dt_s;
        }
    }
}

/**
 * @brief 每个轮询周期更新一次有效性与标定状态。
 *
 * 两条失效判据（任一命中即失效，数据冻结）：
 *   ① 连续 FLOW_QUALITY_BAD_MAX 帧质量不达标（约 120ms）；
 *   ② 超过 FLOW_TIMEOUT_MS 没有合格帧（模块掉线、线被拔）。
 * 用"连续几帧"而不是"单帧"：质量抖动是常态，单帧就失效会让下游每 40ms 抖一次。
 */
static void Flow_UpdateValidity(void)
{
    uint32_t now = osKernelGetTickCount();

    if (!flow_ever_got_frame)
    {
        /* 上电后一帧都没收到过：age 没有意义，直接报无效。 */
        flow_snapshot.valid = false;
        flow_snapshot.age_ms = 0U;
    }
    else
    {
        uint32_t age = now - flow_last_good_tick;

        flow_snapshot.age_ms = age;
        if (flow_quality_bad_streak >= FLOW_QUALITY_BAD_MAX) flow_snapshot.valid = false;
        else if (age > FLOW_TIMEOUT_MS) flow_snapshot.valid = false;
        else flow_snapshot.valid = true;
    }

    /* 标定失败后等一会儿自动重来，不需要重新上电。 */
    if (flow_cal_state == FLOW_CAL_FAILED)
    {
        if ((int32_t)(now - flow_cal_retry_tick) >= 0)
        {
            flow_cal_state = FLOW_CAL_RUNNING;
            flow_cal_frame_count = 0U;
            flow_cal_sum_x = 0.0f;
            flow_cal_sum_y = 0.0f;
        }
    }

    flow_snapshot.cal = (uint8_t)flow_cal_state;
}

void Flow_Init(void)
{
    /* ⚠️ 所有跨拍的异步标志都要在这里清干净。留一个陈旧的标志，
       下一次启动就会静默地卡在错误状态里（尤其是 dma_restart_required
       和 dma_running 这两个——不清的话任务会以为 DMA 已经在跑，实际没有）。 */
    (void)FlowCxof_Init(&flow_device);

    flow_dma_last_position = 0U;
    flow_dma_running = false;
    flow_dma_restart_required = false;
    flow_uart_error_count = 0U;
    flow_dma_drop_count = 0U;
    flow_ring_head = 0U;
    flow_ring_tail = 0U;

    flow_last_good_tick = 0U;
    flow_last_frame_tick = 0U;
    flow_quality_bad_streak = 0U;
    flow_ever_got_frame = false;

    flow_zero_x = 0.0f;
    flow_zero_y = 0.0f;
    flow_cal_frame_count = 0U;
    flow_cal_sum_x = 0.0f;
    flow_cal_sum_y = 0.0f;
    flow_cal_retry_tick = 0U;
    /* ⚠️ 上电即开始标定，而且是**故意的**：上电那一刻飞机必然停在地面上，
       这比"等解锁信号"更可靠。
       为什么不看 `armed`：那要给本模块新增一条跨模块依赖（control_link），
       而门禁①已经保证"上电时一定没解锁"，那个信息拿不到也不影响正确性——
       采样期间飞机若真的在动，均值守卫会判失败并自动重来。 */
    flow_cal_state = FLOW_CAL_RUNNING;

    flow_snapshot.valid = false;
    flow_snapshot.cal = (uint8_t)FLOW_CAL_RUNNING;
    flow_snapshot.quality = 0U;
    flow_snapshot.dx = 0;
    flow_snapshot.dy = 0;
    flow_snapshot.zero_x = 0.0f;
    flow_snapshot.zero_y = 0.0f;
    flow_snapshot.sum_x = 0.0f;
    flow_snapshot.sum_y = 0.0f;
    flow_snapshot.vx_cps = 0.0f;
    flow_snapshot.vy_cps = 0.0f;
    flow_snapshot.age_ms = 0U;
    flow_snapshot.cal_attempts = 0U;
    flow_snapshot.frame_count = 0U;
    flow_snapshot.checksum_error_count = 0U;
    flow_snapshot.format_error_count = 0U;
    flow_snapshot.quality_reject_count = 0U;
    flow_snapshot.resync_count = 0U;
    flow_snapshot.byte_count = 0U;
    flow_snapshot.uart_error_count = 0U;
    flow_snapshot.dma_drop_count = 0U;
}

void Flow_Task(void *argument)
{
    (void)argument;

#if APEX_UART2_ROLE_FLOW
    bool received_byte;
    uint8_t byte;
    FlowCxofFrame_t frame;

    Flow_Init();

    for (;;)
    {
        if (flow_dma_restart_required)
        {
            /* 出过错（ORE/FE 等）：中止再重开，否则 DMA 会永久停在半路。 */
            flow_dma_restart_required = false;
            (void)HAL_UART_AbortReceive(&huart2);
            flow_dma_running = false;
        }

        if (!flow_dma_running)
        {
            if (Flow_StartDmaReception() != HAL_OK)
            {
                osDelay(1000U);
                continue;
            }
        }

        received_byte = false;
        while (Flow_RingPop(&byte))
        {
            received_byte = true;
            if (FlowCxof_ProcessByte(&flow_device, byte, &frame))
            {
                Flow_OnFrame(&frame);
            }
        }

        /* 统计量转抄进快照（打印读的是快照）。 */
        flow_snapshot.frame_count = flow_device.frame_count;
        flow_snapshot.checksum_error_count = flow_device.checksum_error_count;
        flow_snapshot.format_error_count = flow_device.format_error_count;
        flow_snapshot.quality_reject_count = flow_device.quality_reject_count;
        flow_snapshot.resync_count = flow_device.resync_count;
        flow_snapshot.byte_count = flow_device.byte_count;
        flow_snapshot.uart_error_count = flow_uart_error_count;
        flow_snapshot.dma_drop_count = flow_dma_drop_count;

        Flow_UpdateValidity();

        if (!received_byte) osDelay(FLOW_POLL_MS);
        else taskYIELD();
    }
#else
    /* USART2 归 GPS 时本任务不该被创建（app_freertos.c 已经用同一个开关挡住了）。
       万一真被创建，这里空转，**绝不碰 USART2** —— 免得把 GPS 的 DMA 抢掉。
       空转而不是 return：任务返回会触发 FreeRTOS 的断言。 */
    for (;;)
    {
        osDelay(1000U);
    }
#endif
}

void Flow_GetState(FlowState_t *state)
{
    if (state == NULL) return;
    /* 一写多读：本任务在写、tofTask 在读，两者都是 BelowNormal，
       所以临界区里拷贝一份就够（与 gps.c 同一约定）。 */
    taskENTER_CRITICAL();
    *state = flow_snapshot;
    taskEXIT_CRITICAL();
}

#if APEX_UART2_ROLE_FLOW

/* ── USART2 的两个 HAL 回调（与 gps.c 里的同名函数互斥编译）──────────
   逻辑与 gps.c 逐行一致，只是把缓冲换成光流的。改一边要同时看另一边。 */

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    HAL_UART_RxEventTypeTypeDef event_type;

    if ((huart == NULL) || (huart->Instance != USART2) || (huart->hdmarx == NULL)) return;
    if (size > FLOW_DMA_BUFFER_SIZE) size = FLOW_DMA_BUFFER_SIZE;

    event_type = HAL_UARTEx_GetRxEventType(huart);
    if (huart->hdmarx->Init.Mode == DMA_CIRCULAR)
    {
        /* 循环模式：只搬"从上次位置到本次位置"这一段。 */
        if (size > flow_dma_last_position)
        {
            Flow_RingPushFromISR(&flow_dma_buffer[flow_dma_last_position], (uint16_t)(size - flow_dma_last_position));
        }
        else if (size < flow_dma_last_position)
        {
            /* 绕过了缓冲末尾：先搬尾巴，再搬开头。 */
            Flow_RingPushFromISR(&flow_dma_buffer[flow_dma_last_position], (uint16_t)(FLOW_DMA_BUFFER_SIZE - flow_dma_last_position));
            if (size > 0U) Flow_RingPushFromISR(flow_dma_buffer, size);
        }
        flow_dma_last_position = (size == FLOW_DMA_BUFFER_SIZE) ? 0U : size;
    }
    else if (event_type != HAL_UART_RXEVENT_HT)
    {
        /* 非循环模式：一次搬完，然后重开（ReceiveToIdle 是单次的）。 */
        if (size > 0U) Flow_RingPushFromISR(flow_dma_buffer, size);
        flow_dma_restart_required = true;
        flow_dma_running = false;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if ((huart == NULL) || (huart->Instance != USART2)) return;
    flow_uart_error_count++;
    flow_dma_restart_required = true;
    flow_dma_running = false;
}

#endif /* APEX_UART2_ROLE_FLOW */
