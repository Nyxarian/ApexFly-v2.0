/**
 * @file imu_bias.c
 * @brief 上电静置陀螺仪零偏估计实现。
 */

#include "imu_bias.h"

#include <stddef.h>
#include <string.h>

/* 加计模长门禁（用平方值，避免开方）：0.95g ~ 1.10g。
   上限放宽到 1.10g 是因为加计尚未做零偏/尺度标定，静置实测约 1.055g。 */
#define IMU_BIAS_ACCEL_NORM_SQ_MIN 0.9025f
#define IMU_BIAS_ACCEL_NORM_SQ_MAX 1.2100f
/* 静止门禁：相邻两拍角速度变化上限，单位度每秒。静置噪声实测小于 0.3dps。 */
#define IMU_BIAS_GYRO_JUMP_MAX_DPS 1.0f

static float IMU_GyroBias_Absf(float value)
{
    return (value < 0.0f) ? -value : value;
}

/* 门禁触发时清空累加器与静止检测历史，从头重新采集。 */
static void IMU_GyroBias_ClearAccumulator(IMU_GyroBiasCollector_t *collector)
{
    collector->gyro_x_sum = 0.0f;
    collector->gyro_y_sum = 0.0f;
    collector->gyro_z_sum = 0.0f;
    collector->sample_count = 0U;
    collector->last_gyro_x_dps = 0.0f;
    collector->last_gyro_y_dps = 0.0f;
    collector->last_gyro_z_dps = 0.0f;
}

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
    float accel_norm_sq;
    float jump_x;
    float jump_y;
    float jump_z;

    if ((collector == NULL) || (data == NULL) || collector->finished)
    {
        return false;
    }

    /* 门禁 1：加计模长必须接近 1g（机体水平静置），越界说明没放平或正在移动。 */
    accel_norm_sq = (data->accel_x_g * data->accel_x_g) + (data->accel_y_g * data->accel_y_g) + (data->accel_z_g * data->accel_z_g);
    if ((accel_norm_sq < IMU_BIAS_ACCEL_NORM_SQ_MIN) || (accel_norm_sq > IMU_BIAS_ACCEL_NORM_SQ_MAX))
    {
        IMU_GyroBias_ClearAccumulator(collector);
        collector->last_restart_reason = (uint8_t)IMU_BIAS_RESTART_ACCEL;
        return false;
    }

    /* 门禁 2：相邻两拍角速度变化必须小（静止检测），否则清零重来。 */
    if (collector->sample_count > 0U)
    {
        jump_x = IMU_GyroBias_Absf(data->gyro_x_dps - collector->last_gyro_x_dps);
        jump_y = IMU_GyroBias_Absf(data->gyro_y_dps - collector->last_gyro_y_dps);
        jump_z = IMU_GyroBias_Absf(data->gyro_z_dps - collector->last_gyro_z_dps);
        if ((jump_x > IMU_BIAS_GYRO_JUMP_MAX_DPS) || (jump_y > IMU_BIAS_GYRO_JUMP_MAX_DPS) || (jump_z > IMU_BIAS_GYRO_JUMP_MAX_DPS))
        {
            collector->last_jump_dps = (jump_x > jump_y) ? ((jump_x > jump_z) ? jump_x : jump_z) : ((jump_y > jump_z) ? jump_y : jump_z);
            IMU_GyroBias_ClearAccumulator(collector);
            collector->last_restart_reason = (uint8_t)IMU_BIAS_RESTART_GYRO;
            return false;
        }
    }
    collector->last_gyro_x_dps = data->gyro_x_dps;
    collector->last_gyro_y_dps = data->gyro_y_dps;
    collector->last_gyro_z_dps = data->gyro_z_dps;
    collector->last_restart_reason = (uint8_t)IMU_BIAS_RESTART_NONE;

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
