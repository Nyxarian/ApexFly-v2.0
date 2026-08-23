/**
 * @file battery.h
 * @brief 飞机端电池电压：ADC1_IN1 / PA0 经 10k:1k 分压换成毫伏。
 *
 * 电路：BAT_12V -- 10k(R95) -- VBAT_SENSE -- 1k(R94) -- GND，抽头进 PA0。
 * 分压比 1/11。本模块只读 ADC，不写 TIM8，也不碰 LoRa。
 * 不单独开 FreeRTOS 任务；电机任务 1Hz 调用 Update，电台组遥测时取毫伏。
 */

#ifndef BATTERY_H
#define BATTERY_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 校准 ADC1。可以重复调用，只校准一次。
 */
void Battery_Init(void);

/**
 * @brief 采 8 次平均并换算成电池毫伏。
 */
void Battery_Update(void);

/**
 * @brief 最近一次换算出的电池电压。
 * @return 单位毫伏；尚未采样成功时返回 0。
 */
uint16_t Battery_GetMillivolts(void);

#ifdef __cplusplus
}
#endif

#endif
