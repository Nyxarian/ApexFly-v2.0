/**
 * @file imu_bias.c
 * @brief 上电静置陀螺仪零偏估计实现。
 */

#include "imu_bias.h"

#include <stddef.h>
#include <string.h>

/**
 * @brief 清空累加器并开始一轮静置零偏采集。
 */
void IMU_GyroBias_Start(IMU_GyroBiasCollector_t *collector, uint32_t target_sample_count)
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
 * @brief 累加一组陀螺仪样本，收齐后计算平均值作为零偏。
 */
bool IMU_GyroBias_Update(IMU_GyroBiasCollector_t *collector, const MPU6050_Data_t *data)
{
    if ((collector == NULL) || (data == NULL) || collector->finished)
    {
        return false;
    }

    collector->gyro_x_sum += data->gyro_x_dps;
    collector->gyro_y_sum += data->gyro_y_dps;
    collector->gyro_z_sum += data->gyro_z_dps;
    collector->sample_count++;

    if (collector->sample_count < collector->target_sample_count)
    {
        return false;
    }

    collector->bias.gyro_x_dps = collector->gyro_x_sum / (float)collector->sample_count;
    collector->bias.gyro_y_dps = collector->gyro_y_sum / (float)collector->sample_count;
    collector->bias.gyro_z_dps = collector->gyro_z_sum / (float)collector->sample_count;
    collector->bias.valid = true;
    collector->finished = true;
    return true;
}

/**
 * @brief 从六轴数据中扣除已完成的陀螺仪零偏，加速度保持不变。
 */
void IMU_GyroBias_Apply(const IMU_GyroBias_t *bias, MPU6050_Data_t *data)
{
    if ((bias == NULL) || (data == NULL) || (!bias->valid))
    {
        return;
    }

    data->gyro_x_dps -= bias->gyro_x_dps;
    data->gyro_y_dps -= bias->gyro_y_dps;
    data->gyro_z_dps -= bias->gyro_z_dps;
}
