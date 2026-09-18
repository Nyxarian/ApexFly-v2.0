/**
 * @file gps.c
 * @brief ATGM336H 任务：USART2 DMA 收 NMEA，任务里解析，对外提供 UTC 时间。
 *
 * DMA 中断只把字节搬进环形缓冲。字符串校验和换算仍在任务中做。
 * 定位结果只给观察和以后用，当前不进控制环。
 *
 * ── ⚠️ USART2 现在归谁用是可切的（2026-09-18）────────────────
 * 本板没留多余串口，而光流模块只需要一个 RX 脚，所以在把定点/定高做稳的这段时间里
 * 把 GPS 摘下来、光流接到同一个座上（开关见 `Core/Inc/apex_config.h` 的
 * `APEX_UART2_ROLE_FLOW`）。开关为 1 时，本文件里**碰 USART2 的东西全部编掉**：
 * 三个静态收发辅助函数、`GPS_Task` 的任务体、以及下面那两个 HAL 回调。
 *
 * ⚠️ 那两个回调（`HAL_UARTEx_RxEventCallback` / `HAL_UART_ErrorCallback`）与
 * `Modules/Flow/flow.c` 里的同名函数**必须互斥编译**，两边用同一个开关控制 ——
 * 同时定义会报 `L6200E: Symbol HAL_UART_ErrorCallback multiply defined`
 * （2026-09 做串口回显测试时同一个符号已经踩过一次）。
 *
 * ⚠️ 本文件**其余部分照常编译**：`GPS_GetData` / `GPS_GetUtcTime` /
 * `GPS_UnixSecondsToCalendar` 仍被 radio.c 与 blackbox.c 调用。
 * 摘掉 GPS 之后它们一律返回"无效"，于是黑匣子场次的 UTC 时间戳会写 0
 * （用户已定：照原样显示、不做兜底；找日志按场次序号找）。
 */

#include "gps.h"

#include "apex_config.h"
#include "cmsis_os.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define GPS_DMA_BUFFER_SIZE 256U
#define GPS_RING_BUFFER_SIZE 1024U
#define GPS_RING_BUFFER_MASK (GPS_RING_BUFFER_SIZE - 1U)

static GPS_Device_t gps_device;
static GPS_Data_t gps_snapshot;

static uint8_t gps_dma_buffer[GPS_DMA_BUFFER_SIZE];
static uint8_t gps_ring_buffer[GPS_RING_BUFFER_SIZE];

static volatile uint16_t gps_ring_head;
static volatile uint16_t gps_ring_tail;
static volatile uint16_t gps_dma_last_position;
static volatile bool gps_dma_running;
static volatile bool gps_dma_restart_required;
static volatile uint32_t gps_dma_drop_count;
static volatile uint32_t gps_uart_error_count;

#if !APEX_UART2_ROLE_FLOW
/* ⚠️ 下面这三个静态辅助函数只在"GPS 用 USART2"时才有意义。
   USART2 归光流时必须一起编掉，否则它们会变成未使用的静态函数（-Wunused-function）。 */
static void GPS_RingPushFromISR(const uint8_t *data, uint16_t length)
{
    uint16_t index;

    if (data == NULL) return;
    for (index = 0U; index < length; index++)
    {
        uint16_t next_head = (uint16_t)((gps_ring_head + 1U) & GPS_RING_BUFFER_MASK);
        if (next_head == gps_ring_tail)
        {
            gps_dma_drop_count++;
            continue;
        }
        gps_ring_buffer[gps_ring_head] = data[index];
        gps_ring_head = next_head;
    }
}

static bool GPS_RingPop(uint8_t *byte)
{
    if ((byte == NULL) || (gps_ring_tail == gps_ring_head)) return false;
    *byte = gps_ring_buffer[gps_ring_tail];
    gps_ring_tail = (uint16_t)((gps_ring_tail + 1U) & GPS_RING_BUFFER_MASK);
    return true;
}

static HAL_StatusTypeDef GPS_StartDmaReception(void)
{
    HAL_StatusTypeDef status;

    if (huart2.hdmarx == NULL) return HAL_ERROR;

    gps_dma_last_position = 0U;
    status = HAL_UARTEx_ReceiveToIdle_DMA(&huart2, gps_dma_buffer, GPS_DMA_BUFFER_SIZE);
    if (status == HAL_OK)
    {
        __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
        gps_dma_running = true;
    }
    return status;
}

#endif /* !APEX_UART2_ROLE_FLOW */

void GPS_Task(void *argument)
{
#if APEX_UART2_ROLE_FLOW
    /* USART2 归光流时本任务不该被创建（app_freertos.c 用同一个开关挡住了创建）。
       万一真被创建，这里空转，**绝不碰 USART2** —— 否则会把光流的 DMA 抢掉。
       空转而不是直接 return：任务返回会触发 FreeRTOS 的断言。 */
    (void)argument;
    for (;;)
    {
        osDelay(1000U);
    }
#else
    bool received_byte;
    uint8_t byte;
    GPS_Data_t data;

    (void)argument;
    GPS_ATGM336H_Init(&gps_device, &huart2);

    for (;;)
    {
        if (gps_dma_restart_required)
        {
            gps_dma_restart_required = false;
            (void)HAL_UART_AbortReceive(&huart2);
            gps_dma_running = false;
        }

        if (!gps_dma_running)
        {
            if (GPS_StartDmaReception() != HAL_OK)
            {
                osDelay(1000U);
                continue;
            }
        }

        received_byte = false;
        while (GPS_RingPop(&byte))
        {
            received_byte = true;
            (void)GPS_ATGM336H_ProcessByte(&gps_device, byte);
        }

        GPS_ATGM336H_GetData(&gps_device, &data);
        taskENTER_CRITICAL();
        gps_snapshot = data;
        taskEXIT_CRITICAL();

        if (!received_byte) osDelay(2U);
        else taskYIELD();
    }
#endif /* APEX_UART2_ROLE_FLOW */
}

#if !APEX_UART2_ROLE_FLOW
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    HAL_UART_RxEventTypeTypeDef event_type;

    if ((huart == NULL) || (huart->Instance != USART2) || (huart->hdmarx == NULL)) return;
    if (size > GPS_DMA_BUFFER_SIZE) size = GPS_DMA_BUFFER_SIZE;

    event_type = HAL_UARTEx_GetRxEventType(huart);
    if (huart->hdmarx->Init.Mode == DMA_CIRCULAR)
    {
        if (size > gps_dma_last_position)
        {
            GPS_RingPushFromISR(&gps_dma_buffer[gps_dma_last_position], (uint16_t)(size - gps_dma_last_position));
        }
        else if (size < gps_dma_last_position)
        {
            GPS_RingPushFromISR(&gps_dma_buffer[gps_dma_last_position], (uint16_t)(GPS_DMA_BUFFER_SIZE - gps_dma_last_position));
            if (size > 0U) GPS_RingPushFromISR(gps_dma_buffer, size);
        }
        gps_dma_last_position = (size == GPS_DMA_BUFFER_SIZE) ? 0U : size;
    }
    else if (event_type != HAL_UART_RXEVENT_HT)
    {
        if (size > 0U) GPS_RingPushFromISR(gps_dma_buffer, size);
        gps_dma_restart_required = true;
        gps_dma_running = false;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if ((huart == NULL) || (huart->Instance != USART2)) return;
    gps_uart_error_count++;
    gps_dma_restart_required = true;
    gps_dma_running = false;
}
#endif /* !APEX_UART2_ROLE_FLOW */

void GPS_GetData(GPS_Data_t *data)
{
    if (data == NULL) return;
    taskENTER_CRITICAL();
    *data = gps_snapshot;
    taskEXIT_CRITICAL();
}

bool GPS_GetUtcTime(uint32_t *utc_unix_seconds)
{
    GPS_Data_t data;
    uint32_t elapsed_ms;

    if (utc_unix_seconds == NULL) return false;
    GPS_GetData(&data);
    if (!data.date_time_valid) return false;
    elapsed_ms = HAL_GetTick() - data.utc_update_tick;
    *utc_unix_seconds = data.utc_unix_seconds + elapsed_ms / 1000U;
    return true;
}

static bool GPS_IsLeapYear(uint16_t year)
{
    return ((year % 4U) == 0U) && (((year % 100U) != 0U) || ((year % 400U) == 0U));
}

static uint8_t GPS_DaysInMonth(uint16_t year, uint8_t month)
{
    static const uint8_t days_per_month[12] = {31U, 28U, 31U, 30U, 31U, 30U, 31U, 31U, 30U, 31U, 30U, 31U};
    if ((month == 0U) || (month > 12U)) return 0U;
    if ((month == 2U) && GPS_IsLeapYear(year)) return 29U;
    return days_per_month[month - 1U];
}

void GPS_UnixSecondsToCalendar(uint32_t unix_seconds, uint16_t *year, uint8_t *month, uint8_t *day, uint8_t *hour, uint8_t *minute, uint8_t *second)
{
    uint32_t days = unix_seconds / 86400UL;
    uint32_t seconds_of_day = unix_seconds % 86400UL;
    uint16_t calculated_year = 1970U;
    uint8_t calculated_month = 1U;
    uint16_t days_in_year;
    uint8_t days_in_month;

    if ((year == NULL) || (month == NULL) || (day == NULL) || (hour == NULL) || (minute == NULL) || (second == NULL)) return;
    for (;;)
    {
        days_in_year = GPS_IsLeapYear(calculated_year) ? 366U : 365U;
        if (days < days_in_year) break;
        days -= days_in_year;
        calculated_year++;
    }
    for (;;)
    {
        days_in_month = GPS_DaysInMonth(calculated_year, calculated_month);
        if (days < days_in_month) break;
        days -= days_in_month;
        calculated_month++;
    }
    *year = calculated_year;
    *month = calculated_month;
    *day = (uint8_t)(days + 1U);
    *hour = (uint8_t)(seconds_of_day / 3600UL);
    *minute = (uint8_t)((seconds_of_day % 3600UL) / 60UL);
    *second = (uint8_t)(seconds_of_day % 60UL);
}

uint32_t GPS_GetDmaDropCount(void)
{
    return gps_dma_drop_count;
}

uint32_t GPS_GetUartErrorCount(void)
{
    return gps_uart_error_count;
}
