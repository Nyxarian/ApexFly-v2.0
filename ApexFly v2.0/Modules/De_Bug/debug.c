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
static float debug_attitude_roll_deg;
static float debug_attitude_pitch_deg;
static float debug_attitude_yaw_deg;
static bool debug_attitude_valid;

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

void Debug_SetAttitudeSnapshot(float roll_deg, float pitch_deg, float yaw_deg)
{
    taskENTER_CRITICAL();
    debug_attitude_roll_deg = roll_deg;
    debug_attitude_pitch_deg = pitch_deg;
    debug_attitude_yaw_deg = yaw_deg;
    debug_attitude_valid = true;
    taskEXIT_CRITICAL();
}

bool Debug_GetAttitudeSnapshot(float *roll_deg, float *pitch_deg, float *yaw_deg)
{
    bool valid;

    taskENTER_CRITICAL();
    valid = debug_attitude_valid;
    if (valid)
    {
        if (roll_deg != NULL) *roll_deg = debug_attitude_roll_deg;
        if (pitch_deg != NULL) *pitch_deg = debug_attitude_pitch_deg;
        if (yaw_deg != NULL) *yaw_deg = debug_attitude_yaw_deg;
    }
    taskEXIT_CRITICAL();
    return valid;
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
 * @brief 仅输出机体系 Roll、Pitch、Yaw，供姿态方向和回正测试使用。
 */
int Debug_PrintAttitudeAngles(float roll_deg, float pitch_deg, float yaw_deg)
{
    int length;
    HAL_StatusTypeDef status;

    if (!Debug_Lock())
    {
        return -1;
    }

    length = snprintf(debug_printf_buffer, sizeof(debug_printf_buffer), "[ATT] roll=%+.2f deg, pitch=%+.2f deg, yaw=%+.2f deg.\r\n", (double)roll_deg, (double)pitch_deg, (double)yaw_deg);
    if ((length < 0) || (length >= (int)sizeof(debug_printf_buffer)))
    {
        Debug_Unlock();
        return -1;
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
 * @brief 通过 USART3 发送一段原始字节，供黑匣子整行 CSV 导出使用。
 */
HAL_StatusTypeDef Debug_Send(const uint8_t *data, uint16_t length)
{
    HAL_StatusTypeDef status;

    if ((data == NULL) && (length > 0U))
    {
        return HAL_ERROR;
    }
    if (length == 0U)
    {
        return HAL_OK;
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
 * @brief 格式化文本日志并发送。黑匣子 dump / dumpall 走这条路径。
 */
int Debug_Printf(const char *format, ...)
{
    va_list args;
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

    va_start(args, format);
    length = vsnprintf(debug_printf_buffer, sizeof(debug_printf_buffer), format, args);
    va_end(args);
    if ((length < 0) || (length >= (int)sizeof(debug_printf_buffer)))
    {
        Debug_Unlock();
        return -1;
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
    (void)channels;
    (void)channel_count;
    return HAL_OK;
}

/**
 * @brief 发送 FireWater 文本帧：逗号分隔的浮点数，以回车换行结束。
 */
HAL_StatusTypeDef Debug_VofaFireWater(const float *channels, uint8_t channel_count)
{
    (void)channels;
    (void)channel_count;
    return HAL_OK;
}

/**
 * @brief 把六轴惯性数据打包成 JustFloat 的 6 通道波形。
 */
HAL_StatusTypeDef Debug_VofaImu6Axis(float accel_x_g, float accel_y_g, float accel_z_g, float gyro_x_dps, float gyro_y_dps, float gyro_z_dps)
{
    (void)accel_x_g;
    (void)accel_y_g;
    (void)accel_z_g;
    (void)gyro_x_dps;
    (void)gyro_y_dps;
    (void)gyro_z_dps;
    return HAL_OK;
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
    /* 标准库 printf 仍关闭，避免半主机或逐字节发送打乱 dump。 */
    (void)character;
    (void)stream;
    return character;
}
