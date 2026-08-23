/**
 * @file debug.c
 * @brief USART3 调试输出实现：文本日志和 VOFA+ 波形。
 *
 * 多个 FreeRTOS 任务可能同时打印日志，因此发送前用静态互斥锁保护
 * USART3 和格式化缓冲区，避免一行日志被拆开，或 JustFloat 数据帧被插入其他内容。
 */

#include "debug.h"

#include "usart.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

/*
 * 调试串口与老工程保持一致：USART3。
 * PC10：发送
 * PC11：接收
 * 波特率：115200。
 */
#define DEBUG_UART       (&huart3)

/* 阻塞发送和等待互斥锁的超时时间，单位毫秒。 */
#define DEBUG_TIMEOUT_MS 100U

/* 文本格式化缓冲区。发送期间由互斥锁保护，因此可以被多个任务调用。 */
static char debug_printf_buffer[DEBUG_PRINTF_BUFFER_SIZE];

/* 静态互斥锁不占用 FreeRTOS 堆内存。 */
static StaticSemaphore_t debug_mutex_storage;
static SemaphoreHandle_t debug_mutex;

/*
 * VOFA+ JustFloat 协议的帧尾。
 * 这 4 个字节对应浮点数正无穷 0x7F800000。
 * 本芯片是小端，所以串口上的发送顺序是 00 00 80 7F。
 */
static const uint8_t vofa_justfloat_tail[4] = {0x00U, 0x00U, 0x80U, 0x7FU};

/**
 * @brief 创建互斥锁。可以重复调用，只会创建一次。
 */
void Debug_Init(void)
{
    taskENTER_CRITICAL();
    if (debug_mutex == NULL)
    {
        debug_mutex = xSemaphoreCreateMutexStatic(&debug_mutex_storage);
    }
    taskEXIT_CRITICAL();
}

/**
 * @brief 占用调试串口。
 *
 * 调度器尚未启动时直接放行，方便在 main() 里提前打印。
 * @return true 表示已经获得发送权。
 */
static bool Debug_Lock(void)
{
    Debug_Init();
    if (debug_mutex == NULL)
    {
        return false;
    }
    if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED)
    {
        return true;
    }
    return xSemaphoreTake(debug_mutex, pdMS_TO_TICKS(DEBUG_TIMEOUT_MS)) == pdTRUE;
}

/**
 * @brief 释放调试串口。
 *
 * 调度器尚未启动时不需要释放互斥锁。
 */
static void Debug_Unlock(void)
{
    if ((debug_mutex != NULL) && (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED))
    {
        (void)xSemaphoreGive(debug_mutex);
    }
}

/**
 * @brief 通过 USART3 阻塞发送原始字节。
 */
HAL_StatusTypeDef Debug_Send(const uint8_t *data, uint16_t length)
{
    HAL_StatusTypeDef status;

    if ((data == NULL) || (length == 0U))
    {
        return HAL_ERROR;
    }

    if (!Debug_Lock())
    {
        return HAL_BUSY;
    }

    status = HAL_UART_Transmit(DEBUG_UART, (uint8_t *)data, length, DEBUG_TIMEOUT_MS);
    Debug_Unlock();
    return status;
}

/**
 * @brief 格式化一行文本并发送。超长日志会被截断到缓冲区大小。
 */
int Debug_Printf(const char *format, ...)
{
    va_list argument_list;
    int length;
    HAL_StatusTypeDef status;

    if (format == NULL)
    {
        return -1;
    }

    if (!Debug_Lock())
    {
        return -1;
    }

    va_start(argument_list, format);
    length = vsnprintf(debug_printf_buffer, sizeof(debug_printf_buffer), format, argument_list);
    va_end(argument_list);

    if (length < 0)
    {
        Debug_Unlock();
        return -1;
    }

    if (length >= (int)sizeof(debug_printf_buffer))
    {
        length = (int)sizeof(debug_printf_buffer) - 1;
    }

    status = HAL_UART_Transmit(DEBUG_UART, (uint8_t *)debug_printf_buffer, (uint16_t)length, DEBUG_TIMEOUT_MS);
    Debug_Unlock();

    if (status != HAL_OK)
    {
        return -1;
    }

    return length;
}

/**
 * @brief 发送 JustFloat 二进制帧：若干浮点通道，再加 4 字节帧尾。
 */
HAL_StatusTypeDef Debug_VofaJustFloat(const float *channels, uint8_t channel_count)
{
    uint8_t transmit_buffer[DEBUG_VOFA_MAX_CHANNELS * sizeof(float) + sizeof(vofa_justfloat_tail)];
    uint16_t data_length;

    if ((channels == NULL) || (channel_count == 0U) || (channel_count > DEBUG_VOFA_MAX_CHANNELS))
    {
        return HAL_ERROR;
    }

    data_length = (uint16_t)channel_count * (uint16_t)sizeof(float);
    memcpy(transmit_buffer, channels, data_length);
    memcpy(&transmit_buffer[data_length], vofa_justfloat_tail, sizeof(vofa_justfloat_tail));

    return Debug_Send(transmit_buffer, data_length + (uint16_t)sizeof(vofa_justfloat_tail));
}

/**
 * @brief 发送 FireWater 文本帧：逗号分隔的浮点数，以回车换行结束。
 */
HAL_StatusTypeDef Debug_VofaFireWater(const float *channels, uint8_t channel_count)
{
    char line[DEBUG_PRINTF_BUFFER_SIZE];
    int offset = 0;
    uint8_t i;

    if ((channels == NULL) || (channel_count == 0U) || (channel_count > DEBUG_VOFA_MAX_CHANNELS))
    {
        return HAL_ERROR;
    }

    for (i = 0U; i < channel_count; i++)
    {
        int written;

        if (i == 0U)
        {
            written = snprintf(&line[offset], sizeof(line) - (size_t)offset, "%.6f", channels[i]);
        }
        else
        {
            written = snprintf(&line[offset], sizeof(line) - (size_t)offset, ",%.6f", channels[i]);
        }

        if (written < 0)
        {
            return HAL_ERROR;
        }

        offset += written;
        if (offset >= (int)(sizeof(line) - 3))
        {
            return HAL_ERROR;
        }
    }

    if ((sizeof(line) - (size_t)offset) < 3U)
    {
        return HAL_ERROR;
    }

    line[offset++] = '\r';
    line[offset++] = '\n';
    line[offset] = '\0';

    return Debug_Send((const uint8_t *)line, (uint16_t)offset);
}

/**
 * @brief 把六轴惯性数据打包成 JustFloat 的 6 通道波形。
 */
HAL_StatusTypeDef Debug_VofaImu6Axis(float accel_x_g, float accel_y_g, float accel_z_g, float gyro_x_dps, float gyro_y_dps, float gyro_z_dps)
{
    float channels[6];

    channels[0] = accel_x_g;
    channels[1] = accel_y_g;
    channels[2] = accel_z_g;
    channels[3] = gyro_x_dps;
    channels[4] = gyro_y_dps;
    channels[5] = gyro_z_dps;

    return Debug_VofaJustFloat(channels, 6U);
}

#if defined(__CC_ARM)

/* Keil ARMCC5 关闭半主机模式，避免没有调试器时 printf 卡死。 */
#pragma import(__use_no_semihosting)

struct __FILE
{
    int handle;
};

FILE __stdout;

void _sys_exit(int status)
{
    (void)status;
    while (1)
    {
    }
}

#endif

/**
 * @brief 标准库 printf 的底层输出，逐个字符通过 USART3 发送。
 *
 * 高频日志请使用 Debug_Printf()，不要使用 printf()，以免逐字节发送拖慢任务。
 */
int fputc(int character, FILE *stream)
{
    uint8_t data = (uint8_t)character;

    (void)stream;
    (void)HAL_UART_Transmit(DEBUG_UART, &data, 1U, DEBUG_TIMEOUT_MS);
    return character;
}
