/**
 * @file serial_echo.c
 * @brief 【临时】USART3 透明回显实现：发什么就原样返回什么。
 *
 * 数据流：
 *      PC 发字节 → 接收中断(每字节一次) → 环形缓冲 → 回显任务排空 → Debug_Send 原样发回
 *
 * ── 三个必须守住的约束 ───────────────────────────────────────
 * 1. **ISR 里只做两件事**：把字节塞进环形缓冲、重新武装接收。绝不在这里发送——
 *    发送是阻塞的，放在中断里会拖垮整个系统的实时性。
 * 2. **先重新武装，再给信号量**：HAL 的接收是"一次一个字节"，回调返回后必须已经
 *    重新使能了 RXNE 中断；而给完信号量本函数返回时可能立刻切到回显任务，
 *    那一瞬间没在收就会丢字节。
 * 3. **NVIC 优先级必须 ≥ 5**（`configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY`），
 *    否则在中断里调用 FreeRTOS 的 `xSemaphoreGiveFromISR` 会破坏内核。
 *    实际取值见 `usart.c` 的 `USER CODE BEGIN USART3_MspInit 1`。
 *
 * ── 溢出（ORE）必须自己兜住 ─────────────────────────────────
 * USART3 一出溢出，HAL 会把正在进行的接收中止掉、不再产生接收完成中断，
 * 表现就是"回显突然装死"。所以必须有 `HAL_UART_ErrorCallback` 清标志并重新武装，
 * 而不是指望它自己恢复。这一条是回显测试能"断开再插回来照样能用"的关键。
 */

#include "serial_echo.h"

#include "debug.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include <stdint.h>

/* 环形缓冲大小，必须是 2 的幂：索引用掩码回绕，避免取模除法。 */
#define SERIAL_ECHO_RING_SIZE 256U
#define SERIAL_ECHO_RING_MASK (SERIAL_ECHO_RING_SIZE - 1U)

/* 回显一次最多搬出来的字节数，取满整个缓冲：一次排空，避免任务反复占用串口。 */
#define SERIAL_ECHO_CHUNK_MAX SERIAL_ECHO_RING_SIZE

/* 等接收通知的超时，单位毫秒。只是兜底：万一通知丢了也不会永远不干活。 */
#define SERIAL_ECHO_WAIT_MS 100U

/* 环形缓冲。中断写 head、任务读 tail，单生产者单消费者，不需要临界区。
   注意留一个空槽判满，所以实际可用容量是 SIZE-1。 */
static volatile uint8_t serial_echo_ring[SERIAL_ECHO_RING_SIZE];
static volatile uint16_t serial_echo_head;
static volatile uint16_t serial_echo_tail;

/* 缓冲满时丢掉的字节数，只用于诊断（回显任务跟不上才会增长）。 */
static volatile uint32_t serial_echo_dropped_bytes;

/* HAL_UART_Receive_IT 的落点，一次只收 1 个字节。 */
static uint8_t serial_echo_rx_byte;

/* 接收通知。静态分配，不占 FreeRTOS 堆；二值信号量足够——任务每次都把缓冲排空。 */
static StaticSemaphore_t serial_echo_sem_storage;
static SemaphoreHandle_t serial_echo_sem;

/* 发回用的中转缓冲。不能直接发环形缓冲里的内容：Debug_Send 是阻塞的，
   发送期间中断仍在往环形缓冲写，head 会动、tail 也会被本次排空推进。 */
static uint8_t serial_echo_tx_chunk[SERIAL_ECHO_CHUNK_MAX];

/**
 * @brief 把 1 个字节放进环形缓冲。只在中断上下文调用。
 *
 * 缓冲满就丢掉这个字节并计数。回显任务正常情况下跟得上（收多快就发多快），
 * 只有上位机连续猛发、且串口发送被别的日志抢占时才会发生。
 */
static void SerialEcho_PushFromIsr(uint8_t byte)
{
    uint16_t next = (uint16_t)((serial_echo_head + 1U) & SERIAL_ECHO_RING_MASK);

    if (next == serial_echo_tail)
    {
        serial_echo_dropped_bytes++;
        return;
    }
    serial_echo_ring[serial_echo_head] = byte;
    serial_echo_head = next;
}

/**
 * @brief 把环形缓冲里现有的字节搬到 out，返回搬了多少个。
 * @param out 输出缓冲。
 * @param max_length 最多搬多少个。
 */
static uint16_t SerialEcho_Drain(uint8_t *out, uint16_t max_length)
{
    uint16_t length = 0U;

    while ((length < max_length) && (serial_echo_tail != serial_echo_head))
    {
        out[length] = serial_echo_ring[serial_echo_tail];
        serial_echo_tail = (uint16_t)((serial_echo_tail + 1U) & SERIAL_ECHO_RING_MASK);
        length++;
    }
    return length;
}

/**
 * @brief 接收完成回调，每次收到 1 个字节调用一次。运行在 USART3 中断上下文。
 *
 * 只处理 USART3：GPS 用的是 USART2 的另一条回调链，不能误碰。
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    BaseType_t higher_priority_task_woken = pdFALSE;

    if ((huart == NULL) || (huart->Instance != USART3))
    {
        return;
    }

    SerialEcho_PushFromIsr(serial_echo_rx_byte);
    /* 先重新武装，再通知任务，理由见文件头第 2 条。 */
    (void)HAL_UART_Receive_IT(&huart3, &serial_echo_rx_byte, 1U);

    if (serial_echo_sem != NULL)
    {
        xSemaphoreGiveFromISR(serial_echo_sem, &higher_priority_task_woken);
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
}

/**
 * @brief 接收出错回调（溢出 ORE / 帧错误等）。运行在 USART3 中断上下文。
 *
 * ⚠️ 这段是必需的：HAL 在出错时会中止接收，不重新武装就再也不会有接收完成回调，
 * 表现为"回显装死"。清掉接收错误标志后重新武装即可继续。
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if ((huart == NULL) || (huart->Instance != USART3))
    {
        return;
    }

    /* 读 SR 再读 DR，把 PE/FE/NE/ORE 一起清掉。 */
    __HAL_UART_CLEAR_PEFLAG(&huart3);
    (void)HAL_UART_Receive_IT(&huart3, &serial_echo_rx_byte, 1U);
}

/**
 * @brief USART3 全局中断。【临时】由本模块实现，覆盖启动文件里的 WEAK 默认版本。
 */
void USART3_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart3);
}

void SerialEcho_Task(void *argument)
{
    uint16_t length;

    (void)argument;

    serial_echo_head = 0U;
    serial_echo_tail = 0U;
    serial_echo_dropped_bytes = 0U;
    serial_echo_sem = xSemaphoreCreateBinaryStatic(&serial_echo_sem_storage);

    /* 武装第一次接收；之后每次接收完成都在回调里重新武装。 */
    if (HAL_UART_Receive_IT(&huart3, &serial_echo_rx_byte, 1U) != HAL_OK)
    {
        /* 收不起来这个任务就没有意义了。不卡死系统，原地待命。 */
        for (;;)
        {
            vTaskDelay(pdMS_TO_TICKS(1000U));
        }
    }

    for (;;)
    {
        if (serial_echo_sem != NULL)
        {
            (void)xSemaphoreTake(serial_echo_sem, pdMS_TO_TICKS(SERIAL_ECHO_WAIT_MS));
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(10U));
        }

        /* 排空后再看一眼：搬满一整块说明后面可能还有，继续搬。 */
        do
        {
            length = SerialEcho_Drain(serial_echo_tx_chunk, SERIAL_ECHO_CHUNK_MAX);
            if (length > 0U)
            {
                (void)Debug_Send(serial_echo_tx_chunk, length);
            }
        } while (length >= SERIAL_ECHO_CHUNK_MAX);
    }
}
