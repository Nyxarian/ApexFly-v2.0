/**
 * @file led.c
 * @brief 面板 LED 心跳实现。
 *
 * 行为对齐老工程 app_freertos.c 里的 LED_Task：
 * 先翻转一次红灯，然后每 1000ms 同时翻转蓝灯和红灯。
 * 只要调度器在跑，面板就会持续闪烁。
 */

#include "led.h"

#include "cmsis_os.h"
#include "main.h"

/* 心跳周期，与老工程一致，单位毫秒。 */
#define LED_BLINK_PERIOD_MS 1000U

void Led_Task(void *argument)
{
    (void)argument;

    /* 上电后先翻转一次红灯，避免两灯相位完全相同不好辨认。 */
    HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);

    for (;;)
    {
        HAL_GPIO_TogglePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin);
        HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
        osDelay(LED_BLINK_PERIOD_MS);
    }
}
