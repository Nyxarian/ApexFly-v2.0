/**
 * @file baro.h
 * @brief SPL06 气压任务：读 I2C2，以启动地面压力计算相对高度。
 *
 * 卡尔曼在 altitude_est，只用于高度显示；油门由电机任务直通映射。本模块不写 PWM。
 * 当前气压计任务停用，I2C2 只给磁力计。本文件和实现保留，不创建 Baro_Task。
 * 未解锁静置时允许地面压力慢跟踪环境变化；解锁后必须冻结地面压力。
 */

#ifndef BARO_H
#define BARO_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    /* 相对地面高度，单位米，向上为正。 */
    float altitude_m;
    /* 补偿后气压，单位帕。 */
    float pressure_pa;
    /* 当前地面基准压力，单位帕。 */
    float ground_pressure_pa;
    /* 相邻有效气压样本间隔，单位毫秒。 */
    uint32_t sample_interval_ms;
    /* 芯片原始气压计数，仅供串口观察。 */
    int32_t raw_pressure;
    uint32_t sample_count;
    /* true 表示预热和地面压力采集完成。 */
    bool valid;
} BaroSnapshot_t;

/**
 * @brief 气压任务入口。独占 I2C2 上的 SPL06。
 * @param argument 未使用。
 */
void Baro_Task(void *argument);

/**
 * @brief 设置是否允许地面压力慢跟踪。
 * @param allowed 未解锁且电机停止时传 true；解锁后必须传 false。
 */
void Baro_SetGroundTrackingAllowed(bool allowed);

/**
 * @brief 复制最新气压快照。
 * @param snapshot 输出。
 * @return 地面压力已采完且最近一次读取成功返回 true。
 */
bool Baro_GetSnapshot(BaroSnapshot_t *snapshot);

/**
 * @brief 若有尚未被估计环取走的新气压样本，写入 altitude_m 并清标志。
 * @param altitude_m 相对地面高度，单位米。
 * @return 有新样本返回 true。
 */
bool Baro_TakeNewAltitude(float *altitude_m);

#ifdef __cplusplus
}
#endif

#endif
