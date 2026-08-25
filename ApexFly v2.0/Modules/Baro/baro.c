/**
 * @file baro.c
 * @brief SPL06 任务：预热后采地面压力，以压力比计算相对高度并发布新样本。
 *
 * 地面压力只在未解锁、允许跟踪时缓慢适应环境变化；解锁后冻结，避免把真实升高吃掉。
 */

#include "baro.h"

#include "cmsis_os.h"
#include "i2c.h"
#include "spl06.h"
#include "task.h"
#include <math.h>
#include <stddef.h>

#define BARO_POLL_PERIOD_MS 20U
#define BARO_RETRY_PERIOD_MS 1000U
#define BARO_WARMUP_SAMPLES 40U
#define BARO_GROUND_SAMPLES 32U
#define BARO_GROUND_TRIM_SAMPLES 4U
#define BARO_MISS_REINIT_COUNT 150U
#define BARO_GROUND_TRACK_ALPHA 0.001f
#define BARO_VALID_PRESSURE_MIN_PA 30000.0f
#define BARO_VALID_PRESSURE_MAX_PA 120000.0f

static SPL06_Device_t baro_device;
static BaroSnapshot_t baro_snapshot;
static float baro_ground_pressure_pa;
static volatile bool baro_ground_tracking_allowed = true;
static volatile bool baro_has_new;
static float baro_pending_altitude_m;

static float Baro_RelativeAltitude(float pressure_pa, float ground_pressure_pa)
{
    if ((pressure_pa < BARO_VALID_PRESSURE_MIN_PA) || (pressure_pa > BARO_VALID_PRESSURE_MAX_PA) || (ground_pressure_pa < BARO_VALID_PRESSURE_MIN_PA)) return 0.0f;
    return 44330.0f * (1.0f - powf(pressure_pa / ground_pressure_pa, 0.19029495f));
}

static float Baro_TrimmedMean(float *samples, uint8_t count, uint8_t trim)
{
    float sum;
    float temporary;
    uint8_t i;
    uint8_t j;

    for (i = 1U; i < count; i++)
    {
        temporary = samples[i];
        j = i;
        while ((j > 0U) && (samples[j - 1U] > temporary))
        {
            samples[j] = samples[j - 1U];
            j--;
        }
        samples[j] = temporary;
    }

    sum = 0.0f;
    for (i = trim; i < (uint8_t)(count - trim); i++) sum += samples[i];
    return sum / (float)(count - 2U * trim);
}

void Baro_Task(void *argument)
{
    SPL06_Data_t data;
    float ground_samples[BARO_GROUND_SAMPLES];
    uint8_t warmup_count;
    uint8_t ground_count;
    uint32_t last_sample_tick;
    uint32_t current_tick;
    uint16_t miss_count;

    (void)argument;
    for (;;)
    {
        if (!SPL06_Init(&baro_device, &hi2c2))
        {
            taskENTER_CRITICAL();
            baro_snapshot.valid = false;
            baro_has_new = false;
            taskEXIT_CRITICAL();
            osDelay(BARO_RETRY_PERIOD_MS);
            continue;
        }

        warmup_count = 0U;
        ground_count = 0U;
        last_sample_tick = 0U;
        miss_count = 0U;
        taskENTER_CRITICAL();
        baro_snapshot.valid = false;
        baro_has_new = false;
        taskEXIT_CRITICAL();

        for (;;)
        {
            if (!SPL06_ReadData(&baro_device, &data))
            {
                miss_count++;
                if (miss_count >= BARO_MISS_REINIT_COUNT)
                {
                    break;
                }
                osDelay(BARO_POLL_PERIOD_MS);
                continue;
            }
            miss_count = 0U;
            if ((data.pressure_pa < BARO_VALID_PRESSURE_MIN_PA) || (data.pressure_pa > BARO_VALID_PRESSURE_MAX_PA))
            {
                osDelay(BARO_POLL_PERIOD_MS);
                continue;
            }

            current_tick = osKernelGetTickCount();
            if (warmup_count < BARO_WARMUP_SAMPLES)
            {
                warmup_count++;
                osDelay(BARO_POLL_PERIOD_MS);
                continue;
            }
            if (ground_count < BARO_GROUND_SAMPLES)
            {
                ground_samples[ground_count++] = data.pressure_pa;
                if (ground_count < BARO_GROUND_SAMPLES)
                {
                    osDelay(BARO_POLL_PERIOD_MS);
                    continue;
                }
                baro_ground_pressure_pa = Baro_TrimmedMean(ground_samples, BARO_GROUND_SAMPLES, BARO_GROUND_TRIM_SAMPLES);
                last_sample_tick = current_tick;
            }

            if (baro_ground_tracking_allowed)
            {
                baro_ground_pressure_pa += BARO_GROUND_TRACK_ALPHA * (data.pressure_pa - baro_ground_pressure_pa);
            }

            taskENTER_CRITICAL();
            baro_snapshot.sample_interval_ms = (last_sample_tick == 0U) ? 0U : (current_tick - last_sample_tick);
            baro_snapshot.altitude_m = Baro_RelativeAltitude(data.pressure_pa, baro_ground_pressure_pa);
            baro_snapshot.pressure_pa = data.pressure_pa;
            baro_snapshot.ground_pressure_pa = baro_ground_pressure_pa;
            baro_snapshot.raw_pressure = data.raw_pressure;
            baro_snapshot.sample_count = data.sample_count;
            baro_snapshot.valid = true;
            baro_pending_altitude_m = baro_snapshot.altitude_m;
            baro_has_new = true;
            taskEXIT_CRITICAL();
            last_sample_tick = current_tick;

            osDelay(BARO_POLL_PERIOD_MS);
        }
    }
}

void Baro_SetGroundTrackingAllowed(bool allowed)
{
    baro_ground_tracking_allowed = allowed;
}

bool Baro_GetSnapshot(BaroSnapshot_t *snapshot)
{
    bool valid;

    if (snapshot == NULL) return false;
    taskENTER_CRITICAL();
    *snapshot = baro_snapshot;
    valid = baro_snapshot.valid;
    taskEXIT_CRITICAL();
    return valid;
}

bool Baro_TakeNewAltitude(float *altitude_m)
{
    bool has_new;

    if (altitude_m == NULL) return false;
    taskENTER_CRITICAL();
    has_new = baro_has_new && baro_snapshot.valid;
    if (has_new)
    {
        *altitude_m = baro_pending_altitude_m;
        baro_has_new = false;
    }
    taskEXIT_CRITICAL();
    return has_new;
}
