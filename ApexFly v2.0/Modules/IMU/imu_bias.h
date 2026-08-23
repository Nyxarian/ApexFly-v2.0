/**
 * @file imu_bias.h
 * @brief 上电静置陀螺仪零偏估计。
 *
 * 飞机平放不动时采集一段时间的陀螺仪平均值，作为后续角速度的零偏。
 * 本模块不处理加速度六面校准，也不做姿态解算。
 */

#ifndef IMU_BIAS_H
#define IMU_BIAS_H

#include "mpu6050.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    /* X 轴陀螺仪零偏，单位度每秒。 */
    float gyro_x_dps;
    /* Y 轴陀螺仪零偏，单位度每秒。 */
    float gyro_y_dps;
    /* Z 轴陀螺仪零偏，单位度每秒。 */
    float gyro_z_dps;
    /* true 表示已经采完并算出有效零偏。 */
    bool valid;
} IMU_GyroBias_t;

typedef struct
{
    /* 采集期间 X 轴角速度累加，单位度每秒。 */
    float gyro_x_sum;
    /* 采集期间 Y 轴角速度累加，单位度每秒。 */
    float gyro_y_sum;
    /* 采集期间 Z 轴角速度累加，单位度每秒。 */
    float gyro_z_sum;
    /* 已经累加的样本数。 */
    uint32_t sample_count;
    /* 本轮计划采集的样本数。 */
    uint32_t target_sample_count;
    /* true 表示本轮已经结束，不再继续累加。 */
    bool finished;
    /* 采集完成后的零偏结果。 */
    IMU_GyroBias_t bias;
} IMU_GyroBiasCollector_t;

/**
 * @brief 清空累加器并开始一轮零偏采集。
 * @param collector 采集对象。
 * @param target_sample_count 需要累计的样本数，100Hz 下 200 个点约 2 秒。
 */
void IMU_GyroBias_Start(IMU_GyroBiasCollector_t *collector, uint32_t target_sample_count);

/**
 * @brief 累加一组原始六轴数据中的陀螺仪。
 * @return 收齐目标样本并算出零偏后返回 true。
 */
bool IMU_GyroBias_Update(IMU_GyroBiasCollector_t *collector, const MPU6050_Data_t *data);

/**
 * @brief 从原始六轴数据中扣除已完成的陀螺仪零偏。
 * @param data 输入为原始数据，输出角速度已减零偏；加速度保持不变。
 */
void IMU_GyroBias_Apply(const IMU_GyroBias_t *bias, MPU6050_Data_t *data);

#ifdef __cplusplus
}
#endif

#endif
