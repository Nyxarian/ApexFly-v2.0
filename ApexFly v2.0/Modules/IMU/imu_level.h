/**
 * @file imu_level.h
 * @brief 平放静置水平零点：横滚、俯仰减安装倾角。
 *
 * 互补滤波内部仍是芯片系，不要改 imu_attitude 公式。
 * 本模块只处理映射后的机体系角度：静置采平均，之后每拍减去该平均。
 * 不减角速度，也不减偏航。结果只放 RAM，不上电保存。
 *
 * 上电时飞机必须按平时停放姿态放平，采集期间不要碰。
 * 这不是加速度六面校准。
 */

#ifndef IMU_LEVEL_H
#define IMU_LEVEL_H

#include "imu_attitude.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    /* 静置机体系横滚平均，单位度。后续从测量里减掉。 */
    float roll_deg;
    /* 静置机体系俯仰平均，单位度。后续从测量里减掉。 */
    float pitch_deg;
    /* true 表示已经采完，可以对机体系角度使用。 */
    bool valid;
} IMU_LevelOffset_t;

typedef struct
{
    /* 采集期间横滚累加，单位度。 */
    float roll_sum_deg;
    /* 采集期间俯仰累加，单位度。 */
    float pitch_sum_deg;
    /* 已经累加的样本数。 */
    uint32_t sample_count;
    /* 本轮计划采集的样本数。 */
    uint32_t target_sample_count;
    /* true 表示本轮已经结束，不再继续累加。 */
    bool finished;
    /* 采集完成后的水平零点。 */
    IMU_LevelOffset_t offset;
} IMU_LevelCollector_t;

/**
 * @brief 清空累加器并开始一轮水平零点采集。
 * @param collector 采集对象。
 * @param target_sample_count 需要累计的样本数，100Hz 下 200 个点约 2 秒。
 */
void IMU_Level_Start(IMU_LevelCollector_t *collector, uint32_t target_sample_count);

/**
 * @brief 累加一拍已经映射好的机体系横滚、俯仰。
 * @param body 映射后的机体系姿态，单位度。
 * @return 收齐目标样本并算出平均后返回 true。
 */
bool IMU_Level_Update(IMU_LevelCollector_t *collector, const IMU_BodyAttitude_t *body);

/**
 * @brief 从机体系角度中减去已完成的水平零点。角速度和偏航保持不变。
 * @param body 输入为映射后的姿态，输出横滚、俯仰已减零点。
 */
void IMU_Level_Apply(const IMU_LevelOffset_t *offset, IMU_BodyAttitude_t *body);

#ifdef __cplusplus
}
#endif

#endif
