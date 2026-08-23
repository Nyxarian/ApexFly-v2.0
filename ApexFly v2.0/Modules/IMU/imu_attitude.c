/**
 * @file imu_attitude.c
 * @brief 互补滤波姿态解算实现。
 *
 * 短时间相信陀螺仪，长时间用加速度计的重力方向把横滚和俯仰拉回来。
 * 偏航只做陀螺仪积分。
 *
 * IMU_Attitude_Update() 只维护芯片系角度，atan2 / 互补滤波公式不要改。
 *
 * 映射前（芯片系，实机测得）：
 * - roll_deg  ：前倾为负（往前飞），后倾为正（往后飞，抬头）
 * - pitch_deg ：左倾为负（往左飞），右倾为正（往右飞）
 * - yaw_deg   ：逆时针为正，顺时针为负
 *
 * 映射公式（IMU_Attitude_MapToBody）：
 * - 机体系 Roll  =  芯片系 pitch_deg，角速度 = gyro_y
 * - 机体系 Pitch =  芯片系 roll_deg，角速度 = gyro_x
 * - 机体系 Yaw   = -芯片系 yaw_deg，角速度 = -gyro_z
 *
 * 映射后（机体系，飞控习惯）：
 * - Roll  ：右倾为正（往右飞），左倾为负（往左飞）
 * - Pitch ：抬头为正（往后飞），低头为负（往前飞）
 * - Yaw   ：顺时针为正（机头右转），逆时针为负
 */

#include "imu_attitude.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define IMU_ATTITUDE_RAD_TO_DEG 57.2957795f
#define IMU_ATTITUDE_DEG_TO_RAD 0.0174532925f
#define IMU_ATTITUDE_COMPLEMENTARY_TIME_CONSTANT_S 0.50f
#define IMU_ATTITUDE_ACCEL_NORM_MIN_G 0.80f
#define IMU_ATTITUDE_ACCEL_NORM_MAX_G 1.20f
#define IMU_ATTITUDE_DELTA_TIME_MIN_S 0.0005f
#define IMU_ATTITUDE_DELTA_TIME_MAX_S 0.10f
#define IMU_ATTITUDE_MIN_COS_PITCH 0.01f

static float IMU_Attitude_NormalizeAngle(float angle_deg)
{
    while (angle_deg > 180.0f)
    {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f)
    {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

static bool IMU_Attitude_CalculateAccelAngles(const MPU6050_Data_t *data, float *roll_deg, float *pitch_deg)
{
    float accel_norm_square;
    float yz_norm;

    if ((data == NULL) || (roll_deg == NULL) || (pitch_deg == NULL))
    {
        return false;
    }

    accel_norm_square = data->accel_x_g * data->accel_x_g + data->accel_y_g * data->accel_y_g + data->accel_z_g * data->accel_z_g;

    if ((accel_norm_square < (IMU_ATTITUDE_ACCEL_NORM_MIN_G * IMU_ATTITUDE_ACCEL_NORM_MIN_G)) || (accel_norm_square > (IMU_ATTITUDE_ACCEL_NORM_MAX_G * IMU_ATTITUDE_ACCEL_NORM_MAX_G)))
    {
        return false;
    }

    yz_norm = sqrtf(data->accel_y_g * data->accel_y_g + data->accel_z_g * data->accel_z_g);
    *roll_deg = atan2f(data->accel_y_g, data->accel_z_g) * IMU_ATTITUDE_RAD_TO_DEG;
    *pitch_deg = atan2f(-data->accel_x_g, yz_norm) * IMU_ATTITUDE_RAD_TO_DEG;
    return true;
}

void IMU_Attitude_Init(IMU_Attitude_t *attitude)
{
    if (attitude == NULL)
    {
        return;
    }
    memset(attitude, 0, sizeof(*attitude));
}

bool IMU_Attitude_Update(IMU_Attitude_t *attitude, const MPU6050_Data_t *data, float delta_time_s)
{
    float accel_roll_deg;
    float accel_pitch_deg;
    bool accel_angles_valid;
    float roll_rad;
    float pitch_rad;
    float sin_roll;
    float cos_roll;
    float sin_pitch;
    float cos_pitch;
    float tan_pitch;
    float roll_rate_dps;
    float pitch_rate_dps;
    float yaw_rate_dps;
    float predicted_roll_deg;
    float predicted_pitch_deg;
    float predicted_yaw_deg;
    float gyro_weight;
    float roll_correction_deg;
    float pitch_correction_deg;

    if ((attitude == NULL) || (data == NULL))
    {
        return false;
    }
    if ((delta_time_s < IMU_ATTITUDE_DELTA_TIME_MIN_S) || (delta_time_s > IMU_ATTITUDE_DELTA_TIME_MAX_S))
    {
        return false;
    }

    accel_angles_valid = IMU_Attitude_CalculateAccelAngles(data, &accel_roll_deg, &accel_pitch_deg);

    if (attitude->initialized == false)
    {
        if (accel_angles_valid == false)
        {
            return false;
        }
        attitude->roll_deg = IMU_Attitude_NormalizeAngle(accel_roll_deg);
        attitude->pitch_deg = IMU_Attitude_NormalizeAngle(accel_pitch_deg);
        attitude->yaw_deg = 0.0f;
        attitude->initialized = true;
        return true;
    }

    roll_rad = attitude->roll_deg * IMU_ATTITUDE_DEG_TO_RAD;
    pitch_rad = attitude->pitch_deg * IMU_ATTITUDE_DEG_TO_RAD;
    sin_roll = sinf(roll_rad);
    cos_roll = cosf(roll_rad);
    sin_pitch = sinf(pitch_rad);
    cos_pitch = cosf(pitch_rad);

    if ((cos_pitch >= 0.0f) && (cos_pitch < IMU_ATTITUDE_MIN_COS_PITCH))
    {
        cos_pitch = IMU_ATTITUDE_MIN_COS_PITCH;
    }
    if ((cos_pitch < 0.0f) && (cos_pitch > -IMU_ATTITUDE_MIN_COS_PITCH))
    {
        cos_pitch = -IMU_ATTITUDE_MIN_COS_PITCH;
    }
    tan_pitch = sin_pitch / cos_pitch;

    roll_rate_dps = data->gyro_x_dps + sin_roll * tan_pitch * data->gyro_y_dps + cos_roll * tan_pitch * data->gyro_z_dps;
    pitch_rate_dps = cos_roll * data->gyro_y_dps - sin_roll * data->gyro_z_dps;
    yaw_rate_dps = (sin_roll / cos_pitch) * data->gyro_y_dps + (cos_roll / cos_pitch) * data->gyro_z_dps;

    predicted_roll_deg = IMU_Attitude_NormalizeAngle(attitude->roll_deg + roll_rate_dps * delta_time_s);
    predicted_pitch_deg = IMU_Attitude_NormalizeAngle(attitude->pitch_deg + pitch_rate_dps * delta_time_s);
    predicted_yaw_deg = IMU_Attitude_NormalizeAngle(attitude->yaw_deg + yaw_rate_dps * delta_time_s);

    gyro_weight = IMU_ATTITUDE_COMPLEMENTARY_TIME_CONSTANT_S / (IMU_ATTITUDE_COMPLEMENTARY_TIME_CONSTANT_S + delta_time_s);

    if (accel_angles_valid)
    {
        roll_correction_deg = IMU_Attitude_NormalizeAngle(accel_roll_deg - predicted_roll_deg);
        pitch_correction_deg = IMU_Attitude_NormalizeAngle(accel_pitch_deg - predicted_pitch_deg);
        attitude->roll_deg = IMU_Attitude_NormalizeAngle(predicted_roll_deg + (1.0f - gyro_weight) * roll_correction_deg);
        attitude->pitch_deg = IMU_Attitude_NormalizeAngle(predicted_pitch_deg + (1.0f - gyro_weight) * pitch_correction_deg);
    }
    else
    {
        attitude->roll_deg = predicted_roll_deg;
        attitude->pitch_deg = predicted_pitch_deg;
    }

    attitude->yaw_deg = predicted_yaw_deg;
    return true;
}

void IMU_Attitude_MapToBody(const IMU_Attitude_t *attitude, const MPU6050_Data_t *data, IMU_BodyAttitude_t *body)
{
    if ((attitude == NULL) || (data == NULL) || (body == NULL))
    {
        return;
    }

    /* 芯片系 pitch 实际是机体左右倾斜，映射为飞控横滚：右倾为正。 */
    body->roll_deg = attitude->pitch_deg;
    /* 芯片系 roll 实际是机体前后倾斜，映射为飞控俯仰：抬头/后倾为正。 */
    body->pitch_deg = attitude->roll_deg;
    /* 芯片系 yaw 逆时针为正，取反后变为飞控常用的顺时针为正。 */
    body->yaw_deg = -attitude->yaw_deg;
    /* 角速度与角度同一套轴映射，内环直接用减过零偏的陀螺，不用欧拉角速率。 */
    body->roll_rate_dps = data->gyro_y_dps;
    body->pitch_rate_dps = data->gyro_x_dps;
    body->yaw_rate_dps = -data->gyro_z_dps;
}
