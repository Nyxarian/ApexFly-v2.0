/**
 * @file imu_level.c
 * @brief 平放静置水平零点实现。
 *
 * 只对机体系横滚、俯仰求平均并减掉，不改互补滤波，也不动陀螺零偏。
 */

#include "imu_level.h"

#include <stddef.h>
#include <string.h>

/**
 * @brief 清空累加器并开始一轮静置水平采集。
 */
void IMU_Level_Start(IMU_LevelCollector_t *collector, uint32_t target_sample_count)
{
    if (collector == NULL)
    {
        return;
    }

    memset(collector, 0, sizeof(*collector));
    if (target_sample_count == 0U)
    {
        target_sample_count = 1U;
    }
    collector->target_sample_count = target_sample_count;
}

/**
 * @brief 累加一拍机体系角度，收齐后计算平均值作为水平零点。
 */
bool IMU_Level_Update(IMU_LevelCollector_t *collector, const IMU_BodyAttitude_t *body)
{
    if ((collector == NULL) || (body == NULL) || collector->finished)
    {
        return false;
    }

    collector->roll_sum_deg += body->roll_deg;
    collector->pitch_sum_deg += body->pitch_deg;
    collector->sample_count++;

    if (collector->sample_count < collector->target_sample_count)
    {
        return false;
    }

    collector->offset.roll_deg = collector->roll_sum_deg / (float)collector->sample_count;
    collector->offset.pitch_deg = collector->pitch_sum_deg / (float)collector->sample_count;
    collector->offset.valid = true;
    collector->finished = true;
    return true;
}

/**
 * @brief 横滚、俯仰减去静置平均，角速度和偏航不改。
 */
void IMU_Level_Apply(const IMU_LevelOffset_t *offset, IMU_BodyAttitude_t *body)
{
    if ((offset == NULL) || (body == NULL) || (!offset->valid))
    {
        return;
    }

    body->roll_deg -= offset->roll_deg;
    body->pitch_deg -= offset->pitch_deg;
}
