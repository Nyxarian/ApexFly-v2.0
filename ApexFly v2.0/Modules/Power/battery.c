/**
 * @file battery.c
 * @brief 电池电压采样实现。
 *
 * 硬件：PA0 / ADC1_IN1，10k 上、1k 下，分压 1/11。
 * 3S 满电约 12.6V，脚上约 1.15V，低于 3.3V。
 * 没有滤波电容，软件平均 16 次后再做 IIR，避免遥控一位小数在 11.7/11.8 之间跳。
 * 分压比保持 11。万用表 12.11V 时未校准读数约 11.75V，用 103/100 把电阻公差和 VDDA 偏差一次补上。
 */

#include "battery.h"

#include "adc.h"
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#define BATTERY_ADC_RESOLUTION 4095U
#define BATTERY_VDDA_MV 3300U
#define BATTERY_DIVIDER_RATIO 11U
#define BATTERY_SCALE_NUM 103U
#define BATTERY_SCALE_DEN 100U
#define BATTERY_AVERAGE_COUNT 16U
#define BATTERY_FILTER_SHIFT 3U
#define BATTERY_SAMPLE_TIMEOUT_MS 10U

static uint16_t battery_millivolts;
static uint32_t battery_filter_mv;
static bool battery_filter_ready;
static bool battery_calibrated;

/**
 * @brief 软件触发一次 ADC1，超时或失败返回 false。
 */
static bool Battery_ReadRaw(uint32_t *raw)
{
    if (raw == NULL) return false;
    if (HAL_ADC_Start(&hadc1) != HAL_OK) return false;
    if (HAL_ADC_PollForConversion(&hadc1, BATTERY_SAMPLE_TIMEOUT_MS) != HAL_OK)
    {
        (void)HAL_ADC_Stop(&hadc1);
        return false;
    }
    *raw = HAL_ADC_GetValue(&hadc1);
    (void)HAL_ADC_Stop(&hadc1);
    return true;
}

void Battery_Init(void)
{
    ADC_ChannelConfTypeDef config;

    if (battery_calibrated) return;
    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK) return;

    /* 分压源阻抗约 11k，把采样时间从 CubeMX 的 2.5 周期加长。 */
    memset(&config, 0, sizeof(config));
    config.Channel = ADC_CHANNEL_1;
    config.Rank = ADC_REGULAR_RANK_1;
    config.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
    config.SingleDiff = ADC_SINGLE_ENDED;
    config.OffsetNumber = ADC_OFFSET_NONE;
    config.Offset = 0;
    if (HAL_ADC_ConfigChannel(&hadc1, &config) != HAL_OK) return;
    battery_calibrated = true;
}

void Battery_Update(void)
{
    uint32_t raw;
    uint32_t sum;
    uint32_t measured;
    uint8_t i;
    uint8_t good;

    if (!battery_calibrated) Battery_Init();
    if (!battery_calibrated) return;

    sum = 0U;
    good = 0U;
    for (i = 0U; i < BATTERY_AVERAGE_COUNT; i++)
    {
        if (!Battery_ReadRaw(&raw)) continue;
        sum += raw;
        good++;
    }
    if (good == 0U) return;
    /* 先算脚上毫伏再 ×11，避免 16 次累加后乘 103 溢出 32 位。 */
    measured = (sum / good) * BATTERY_VDDA_MV / BATTERY_ADC_RESOLUTION;
    measured = measured * BATTERY_DIVIDER_RATIO * BATTERY_SCALE_NUM / BATTERY_SCALE_DEN;
    if (!battery_filter_ready)
    {
        battery_filter_mv = measured;
        battery_filter_ready = true;
    }
    else
    {
        battery_filter_mv = (battery_filter_mv * ((1UL << BATTERY_FILTER_SHIFT) - 1UL) + measured) >> BATTERY_FILTER_SHIFT;
    }
    battery_millivolts = (uint16_t)battery_filter_mv;
}

uint16_t Battery_GetMillivolts(void)
{
    return battery_millivolts;
}
