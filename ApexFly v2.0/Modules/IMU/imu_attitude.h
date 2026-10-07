/**
 * @file imu_attitude.h
 * @brief MPU6050 六轴 Mahony4 风格四元数姿态解算接口。
 *
 * 内部姿态状态保存在 MPU6050 芯片坐标系四元数中，输入陀螺仪必须已经扣除零偏，
 * 加速度单位为 g，陀螺仪单位为度每秒。对外仍输出 ApexFly 原有的机体系角度和角速度，
 * 因此姿态串级 PID、混控和水平零点接口不需要改变。
 *
 * 当前芯片系到机体系的实测映射保持不变，输出再叠加固定安装角 Trim：
 * - 机体系 Roll  = 芯片系 pitch_deg，角速度 = gyro_y
 * - 机体系 Pitch = 芯片系 roll_deg，角速度 = gyro_x
 * - 机体系 Yaw   = -芯片系 yaw_deg，角速度 = -gyro_z
 *
 * MPU6050 没有绝对航向参考，Yaw 仍然是相对角度，长时间会因陀螺仪残余零偏漂移。
 */

#ifndef IMU_ATTITUDE_H
#define IMU_ATTITUDE_H

#include "mpu6050.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    /* 芯片系四元数标量部分。 */
    float q0;
    /* 芯片系四元数 X 虚部。 */
    float q1;
    /* 芯片系四元数 Y 虚部。 */
    float q2;
    /* 芯片系四元数 Z 虚部。 */
    float q3;
    /* Mahony 积分反馈的 X 轴角速度修正，单位弧度每秒。 */
    float integral_x;
    /* Mahony 积分反馈的 Y 轴角速度修正，单位弧度每秒。 */
    float integral_y;
    /* Mahony 积分反馈的 Z 轴角速度修正，单位弧度每秒。 */
    float integral_z;
    /* 最近一拍加速度模长，单位 g。 */
    float accel_norm_g;
    /* 加计三轴软件低通状态，单位 g。 */
    float accel_x_lpf;
    float accel_y_lpf;
    float accel_z_lpf;
    /* true 表示低通已用首拍数据初始化。 */
    bool accel_lpf_ready;
    /* 上一拍修正后的角速度（含 Mahony 修正），单位弧度每秒，用于中值积分。 */
    float last_gyro_x_rad;
    float last_gyro_y_rad;
    float last_gyro_z_rad;
    /* true 表示已有一拍历史，可以开始中值积分。 */
    bool gyro_history_valid;
    /* 最近一拍重力方向误差，单位无量纲。 */
    float fusion_error_x;
    /* 最近一拍重力方向误差，单位无量纲。 */
    float fusion_error_y;
    /* 最近一拍重力方向误差，单位无量纲。 */
    float fusion_error_z;
    /* true 表示已经由有效加速度完成初始化。 */
    bool initialized;
    /* true 表示本拍使用了加速度重力修正。 */
    bool accel_correction_valid;
    /* 低频诊断：加速度健康状态，1=正常，2=可疑，3=动态/不可用。 */
    uint8_t accel_health_state;
    /* 低频诊断：持续满足静止门限的样本数。 */
    uint16_t stationary_sample_count;
    /* true 表示已持续满足静止门限。 */
    bool stationary;
    /* 静止检测使用的上一拍陀螺仪数据。 */
    float health_last_gyro_x_dps;
    float health_last_gyro_y_dps;
    float health_last_gyro_z_dps;
    bool health_gyro_history_valid;
    /* 最近一拍重力融合误差的模长。 */
    float fusion_error_norm;
    /* 当前加速度重力修正权重，0~1。阶段 4C 动态调整。 */
    float accel_correction_weight;
} IMU_Attitude_t;

typedef struct
{
    /* 映射后横滚角，单位度。右倾为正，左倾为负。 */
    float roll_deg;
    /* 映射后俯仰角，单位度。抬头为正，低头为负。 */
    float pitch_deg;
    /* 映射后偏航角，单位度。顺时针为正，逆时针为负。 */
    float yaw_deg;
    /* 机体系横滚角速度，单位度每秒。 */
    float roll_rate_dps;
    /* 机体系俯仰角速度，单位度每秒。 */
    float pitch_rate_dps;
    /* 机体系偏航角速度，单位度每秒。 */
    float yaw_rate_dps;
} IMU_BodyAttitude_t;

/**
 * @brief 清零姿态状态并设置单位四元数。
 * @param attitude 姿态状态对象地址。
 */
void IMU_Attitude_Init(IMU_Attitude_t *attitude);

/**
 * @brief 用减过陀螺仪零偏的六轴数据更新一次四元数姿态。
 * @param attitude 姿态状态对象地址。
 * @param data MPU6050 数据，加速度单位 g，陀螺仪单位度每秒。
 * @param delta_time_s 与上一拍的实际时间间隔，单位秒。
 * @return 本拍更新成功返回 true，参数、dt 或四元数状态异常返回 false。
 */
bool IMU_Attitude_Update(IMU_Attitude_t *attitude, const MPU6050_Data_t *data, float delta_time_s);

/**
 * @brief 把芯片系四元数姿态和减过零偏的陀螺仪映射成机体系输出。
 * @param attitude 当前四元数姿态。
 * @param data 当前已扣除陀螺仪零偏的数据。
 * @param body 输出的机体系角度和角速度。
 */
void IMU_Attitude_MapToBody(const IMU_Attitude_t *attitude, const MPU6050_Data_t *data, IMU_BodyAttitude_t *body);

/**
 * @brief 用当前四元数把加速度投影到向上方向并扣除重力。
 * @param attitude 当前四元数姿态。
 * @param data 当前加速度数据，单位 g。
 * @return 垂直线性加速度，单位米每二次方秒，向上为正。
 */
float IMU_Attitude_VerticalAccelUp(const IMU_Attitude_t *attitude, const MPU6050_Data_t *data);

/** @brief 获取最近一拍的加速度模长，单位 g。 */
float IMU_Attitude_GetAccelNormG(const IMU_Attitude_t *attitude);

/** @brief 获取最近一拍是否使用了加速度重力修正。 */
bool IMU_Attitude_IsAccelCorrectionValid(const IMU_Attitude_t *attitude);

/**
 * @brief 获取最近一拍的 Mahony 重力方向误差。
 * @param x 输出 X 误差地址，可为空。
 * @param y 输出 Y 误差地址，可为空。
 * @param z 输出 Z 误差地址，可为空。
 */
void IMU_Attitude_GetFusionError(const IMU_Attitude_t *attitude, float *x, float *y, float *z);

/** @brief 获取姿态健康诊断：加速度模长、修正状态、静止状态和融合误差。 */
void IMU_Attitude_GetHealth(const IMU_Attitude_t *attitude, float *accel_norm_g, bool *accel_correction_valid, uint8_t *accel_health_state, bool *stationary, float *fusion_error_norm, float *accel_correction_weight);

/**
 * @brief 获取当前四元数。
 * @param q 输出四元数数组，顺序为 q0、q1、q2、q3。
 */
void IMU_Attitude_GetQuaternion(const IMU_Attitude_t *attitude, float q[4]);

#ifdef __cplusplus
}
#endif

#endif
