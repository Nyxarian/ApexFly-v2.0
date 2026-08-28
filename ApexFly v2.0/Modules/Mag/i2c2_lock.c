/**
 * @file i2c2_lock.c
 * @brief I2C2 静态互斥量实现。调度起来之前就要创建，避免气压和磁力计同时 Init。
 */

#include "i2c2_lock.h"

#include "FreeRTOS.h"
#include "i2c.h"
#include "semphr.h"

static SemaphoreHandle_t i2c2_mutex;
static StaticSemaphore_t i2c2_mutex_storage;

void I2C2_Lock_Init(void)
{
    if (i2c2_mutex != NULL) return;
    i2c2_mutex = xSemaphoreCreateMutexStatic(&i2c2_mutex_storage);
}

bool I2C2_Lock(uint32_t timeout_ms)
{
    if (i2c2_mutex == NULL) I2C2_Lock_Init();
    if (i2c2_mutex == NULL) return false;
    return xSemaphoreTake(i2c2_mutex, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}

void I2C2_Unlock(void)
{
    if (i2c2_mutex == NULL) return;
    (void)xSemaphoreGive(i2c2_mutex);
}

static void I2C2_ClockDelay(void)
{
    volatile uint32_t i;
    for (i = 0U; i < 200U; i++)
    {
        __NOP();
    }
}

void I2C2_BusRecover(void)
{
    GPIO_InitTypeDef gpio = {0};
    uint8_t pulse;

    (void)HAL_I2C_DeInit(&hi2c2);

    __HAL_RCC_GPIOA_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    gpio.Mode = GPIO_MODE_OUTPUT_OD;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &gpio);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
    I2C2_ClockDelay();

    /* 从机若把 SDA 拉死，主控补 9 个 SCL 脉冲把它踢出当前字节。 */
    for (pulse = 0U; pulse < 9U; pulse++)
    {
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == GPIO_PIN_SET) break;
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
        I2C2_ClockDelay();
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
        I2C2_ClockDelay();
    }

    /* 补一个 STOP：SDA 在 SCL 高时拉高。 */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
    I2C2_ClockDelay();
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
    I2C2_ClockDelay();
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
    I2C2_ClockDelay();

    MX_I2C2_Init();
}
