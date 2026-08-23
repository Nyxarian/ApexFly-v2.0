/**
 * @file imu_attitude.h
 * @brief 互补滤波姿态解算。
 *
 * 输入已经扣除陀螺仪零偏的六轴数据。
 * 横滚和俯仰由陀螺仪积分加加速度重力方向修正得到。
 * 偏航没有磁力计，只能做相对积分，长时间会缓慢漂移。
 *
 * IMU_Attitude_t 里保存的是芯片坐标系角度，互补滤波必须继续用这组值，
 * 不要把映射后的机体系角度写回去。对外给 VOFA+ / 控制用时，调用
 * IMU_Attitude_MapToBody()。装机水平零点在映射之后减，见 imu_level。
 *
 * ---------- 映射前（芯片系，实机测得）----------
 * roll_deg  对应旧 VOFA+ I0，管前后：
 *   前倾（机头低、往前飞）为负；后倾（机头高、往后飞/抬头）为正。
 * pitch_deg 对应旧 VOFA+ I1，管左右：
 *   左倾（往左飞）为负；右倾（往右飞）为正。
 * yaw_deg   对应旧 VOFA+ I2，管自旋：
 *   逆时针为正；顺时针为负。
 *
 * ---------- 映射公式 ----------
 * 机体系横滚 Roll  =  芯片系 pitch_deg，角速度用芯片系 gyro_y
 * 机体系俯仰 Pitch =  芯片系 roll_deg，角速度用芯片系 gyro_x
 * 机体系偏航 Yaw   = -芯片系 yaw_deg，角速度用 -芯片系 gyro_z
 *
 * ---------- 映射后（机体系，飞控习惯）----------
 * I0 = Roll ：右倾为正，对应往右飞；左倾为负，对应往左飞。
 * I1 = Pitch：抬头为正，对应后倾、往后飞；低头为负，对应前倾、往前飞。
 * I2 = Yaw  ：顺时针为正，机头右转；逆时针为负，机头左转。
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
    /* 芯片系。映射前对应旧 I0：前倾为负（往前飞），后倾为正（往后飞）。 */
    float roll_deg;
    /* 芯片系。映射前对应旧 I1：左倾为负（往左飞），右倾为正（往右飞）。 */
    float pitch_deg;
    /* 芯片系。映射前对应旧 I2：逆时针为正，顺时针为负。 */
    float yaw_deg;
    bool initialized;
} IMU_Attitude_t;

typedef struct
{
    /* 映射后横滚，单位度。右倾为正（往右飞），左倾为负（往左飞）。 */
    float roll_deg;
    /* 映射后俯仰，单位度。抬头为正（往后飞），低头为负（往前飞）。 */
    float pitch_deg;
    /* 映射后偏航，单位度。顺时针为正（机头右转），逆时针为负。 */
    float yaw_deg;
    /* 机体系横滚角速度，单位度每秒。右倾加快为正。 */
    float roll_rate_dps;
    /* 机体系俯仰角速度，单位度每秒。抬头加快为正。 */
    float pitch_rate_dps;
    /* 机体系偏航角速度，单位度每秒。顺时针加快为正。 */
    float yaw_rate_dps;
} IMU_BodyAttitude_t;

void IMU_Attitude_Init(IMU_Attitude_t *attitude);

/**
 * @brief 用一组减过零偏的六轴数据更新芯片系姿态角。
 * @param delta_time_s 与上一拍的时间间隔，单位秒，100Hz 时应接近 0.01。
 * @return 本拍更新成功返回 true。
 */
bool IMU_Attitude_Update(IMU_Attitude_t *attitude, const MPU6050_Data_t *data, float delta_time_s);

/**
 * @brief 把芯片系姿态和减过零偏的陀螺映射成机体系角度、角速度。
 *
 * Roll  = 芯片系 pitch_deg，roll_rate  = gyro_y
 * Pitch = 芯片系 roll_deg，pitch_rate = gyro_x
 * Yaw   = -芯片系 yaw_deg，yaw_rate   = -gyro_z
 */
void IMU_Attitude_MapToBody(const IMU_Attitude_t *attitude, const MPU6050_Data_t *data, IMU_BodyAttitude_t *body);

#ifdef __cplusplus
}
#endif

#endif
