/**
 * @file led.h
 * @brief 面板 LED 心跳指示。
 *
 * 硬件与老工程一致：
 * LED_BLUE：PF9
 * LED_RED ：PF10
 *
 * 上电后由 FreeRTOS 任务交替翻转蓝灯和红灯，用来确认程序正在运行。
 */

#ifndef LED_H
#define LED_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief LED 心跳任务入口。
 * @param argument 未使用。
 */
void Led_Task(void *argument);

#ifdef __cplusplus
}
#endif

#endif
