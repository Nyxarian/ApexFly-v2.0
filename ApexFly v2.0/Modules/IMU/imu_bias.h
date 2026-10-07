/**
 * @file imu_bias.h
 * @brief 上电静置陀螺仪零偏估计。
 *
 * 飞机平放不动时采集一段时间的陀螺仪平均值，作为后续角速度的零偏。
 * 本模块不做加速度计校准，也不做姿态解算。
 */

#ifndef IMU_BIAS_H
#define IMU_BIAS_H

#include "mpu6050.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 零偏采集被门禁清零的原因，仅诊断用。 */
typedef enum
{
    IMU_BIAS_RESTART_NONE = 0, /* 未触发门禁。 */
    IMU_BIAS_RESTART_ACCEL,    /* 加计模长越界：机体未放平或正在移动。 */
    IMU_BIAS_RESTART_GYRO      /* 相邻两拍角速度跳变过大：机体在动。 */
} IMU_BiasRestartReason_t;

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
    /* 上一拍角速度，用于静止检测（逐拍变化门禁）。 */
    float last_gyro_x_dps;
    float last_gyro_y_dps;
    float last_gyro_z_dps;
    /* 最近一次门禁触发时的三轴最大跳变量，单位度每秒，仅诊断用。 */
    float last_jump_dps;
    /* 最近一次被门禁清零的原因。 */
    uint8_t last_restart_reason;
    /* 采集完成后的零偏结果。 */
    IMU_GyroBias_t bias;
} IMU_GyroBiasCollector_t;

typedef struct
{
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
    bool learning_allowed;
} IMU_GyroOnlineBias_t;

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

/** @brief 清零运行时陀螺仪 Bias，结果只保存在 RAM。 */
void IMU_GyroOnlineBias_Init(IMU_GyroOnlineBias_t *online_bias);

/**
 * @brief 在满足静止门禁时慢速学习残余角速度。
 * @param disarmed true 表示未解锁。
 * @param stationary 姿态模块静止判定。
 * @param accel_health_ok 加速度健康状态是否为正常。
 * @param accel_weight 当前重力修正权重。
 * @param dt_s 本拍周期。
 * @return 本拍是否允许学习。
 */
bool IMU_GyroOnlineBias_Update(IMU_GyroOnlineBias_t *online_bias, bool disarmed, bool stationary, bool accel_health_ok, float accel_weight, float gyro_x_dps, float gyro_y_dps, float gyro_z_dps, float dt_s);

/** @brief 把运行时 Bias 从角速度中扣除。 */
void IMU_GyroOnlineBias_Apply(const IMU_GyroOnlineBias_t *online_bias, MPU6050_Data_t *data);

/** @brief 读取运行时 Bias 和本拍学习许可。 */
void IMU_GyroOnlineBias_Get(const IMU_GyroOnlineBias_t *online_bias, float *x_dps, float *y_dps, float *z_dps, bool *learning_allowed);

#ifdef __cplusplus
}
#endif

#endif
