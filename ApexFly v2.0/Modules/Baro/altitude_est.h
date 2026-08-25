/**
 * @file altitude_est.h
 * @brief 三状态卡尔曼：离地高度 z、垂直速度 vz、垂直加速度零偏 ba。
 *
 * 100 Hz 用垂直加速度预测，气压新样本到来时更新。气压连续异常或超时后，
 * healthy 会清零，高度控制冻结最近安全油门，等待估计恢复或人工上锁。
 */

#ifndef ALTITUDE_EST_H
#define ALTITUDE_EST_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    /* 离地高度，单位米，向上为正。 */
    float z_m;
    /* 垂直速度，单位米每秒，向上为正。 */
    float vz_mps;
    /* 估出的垂直加速度零偏，单位米每二次方秒，从 az 里扣掉。 */
    float accel_bias_mps2;
    /* 最近一次气压创新量 baro_z - kf_z，单位米。 */
    float baro_innovation_m;
    /* true 表示至少已经用气压初始化。 */
    bool initialized;
    /* true 表示气压连续有效且估计量正常，高度环可以使用。 */
    bool healthy;
} AltitudeEstimate_t;

/**
 * @brief 重置高度估计器并写入静置采集的垂直加速度初始零偏。
 * @param initial_accel_bias_mps2 静止时垂直线性加速度平均值，单位米每二次方秒。
 */
void AltitudeEst_Init(float initial_accel_bias_mps2);

/**
 * @brief 用垂直加速度预测一拍，若有新气压则更新。
 * @param accel_up_mps2 地球垂直线性加速度，单位米每二次方秒，已扣重力，向上为正。
 * @param dt_s 与上一拍间隔，单位秒。
 * @param baro_altitude_m 相对地面气压高度，单位米。
 * @param baro_fresh 本拍有新气压样本。
 */
void AltitudeEst_Update(float accel_up_mps2, float dt_s, float baro_altitude_m, bool baro_fresh);

void AltitudeEst_Get(AltitudeEstimate_t *estimate);

#ifdef __cplusplus
}
#endif

#endif
