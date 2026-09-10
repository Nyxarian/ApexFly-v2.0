/**
 * @file mag.h
 * @brief 磁力计任务接口。I2C2 读 GY-273，对外提供原始磁场快照。
 *
 * 当前把芯片 XYZ 转到机体系 mx/my/mz 并做一阶低通，只供串口观察。
 * 支持串口 CALMAG 和遥控 MAG_CAL 操作码采集硬铁偏移；成功后由黑匣子写入 0x002000，上电再加载。
 * 不进姿态融合，不改偏航 PID。
 * 安装丝印：Y 朝机头、X 朝左、Z 朝下。实测朝北/朝西后按 mx=-x、my=y、mz=z 转到机体系。
 */

#ifndef MAG_H
#define MAG_H

#include "hmc5883l.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    /* true 表示已经完成一次可用硬铁校准。 */
    bool calibration_valid;
    /* 校准后的机体系前向磁场，单位高斯。 */
    float cal_mx_gauss;
    /* 校准后的机体系右向磁场，单位高斯。 */
    float cal_my_gauss;
    /* 校准后的机体系下向磁场，单位高斯。 */
    float cal_mz_gauss;
    /* 校准后三轴合成模长，单位高斯。 */
    float cal_magnitude_gauss;
    /* 校准后的水平航向，单位度，0～360。未做倾角补偿。 */
    float cal_heading_deg;
    MagChip_t chip;
    /* 7 位 I2C 地址。 */
    uint8_t address_7bit;
    int16_t raw_x;
    int16_t raw_y;
    int16_t raw_z;
    float x_gauss;
    float y_gauss;
    float z_gauss;
    float magnitude_gauss;
    /* 机体系前向磁场，单位高斯。实测 = -芯片 y。 */
    float mx_gauss;
    /* 机体系右向磁场，单位高斯。实测 = +芯片 x。 */
    float my_gauss;
    /* 机体系下向磁场，单位高斯。实测芯片 Z 朝下，所以 mz = +芯片 z。 */
    float mz_gauss;
    /* 机体系三轴合成模长，单位高斯。 */
    float body_magnitude_gauss;
    /* 芯片系水平粗航向，单位度，0～360。未做倾角补偿。 */
    float raw_heading_deg;
    /* 机体系水平粗航向，单位度，0～360。atan2(-my, mx)，顺时针为正。未做倾角补偿。 */
    float body_heading_deg;
    uint32_t sample_count;
    uint32_t overflow_count;
    uint32_t error_count;
    /* 最近一次总线扫描结果，init 失败时用来判断接线还是 ID。 */
    MagProbe_t probe;
    bool valid;
} MagSnapshot_t;

/**
 * @brief 磁力计任务入口。探测芯片后按约 20 ms 读一次。
 * @param argument 未使用。
 */
void Mag_Task(void *argument);

/**
 * @brief 复制最近一次磁力计快照。
 * @param snapshot 输出。
 * @return 当前有有效样本返回 true。
 */
bool Mag_GetSnapshot(MagSnapshot_t *snapshot);

typedef struct
{
    /* true 表示正在采集极值。 */
    bool active;
    /* true 表示已有一次校准偏移正在应用。 */
    bool valid;
    /* 三轴覆盖范围是否达到最低要求。 */
    bool quality_ok;
    /* 已采集的有效样本数。 */
    uint32_t sample_count;
    /* 芯片转换到机体系后的最小值，单位高斯。 */
    float min_mx_gauss;
    float min_my_gauss;
    float min_mz_gauss;
    /* 芯片转换到机体系后的最大值，单位高斯。 */
    float max_mx_gauss;
    float max_my_gauss;
    float max_mz_gauss;
    /* 由 min/max 中点得到的硬铁偏移，单位高斯。 */
    float offset_mx_gauss;
    float offset_my_gauss;
    float offset_mz_gauss;
    /* 三轴覆盖跨度，单位高斯。 */
    float span_mx_gauss;
    float span_my_gauss;
    float span_mz_gauss;
} MagCalibrationStatus_t;

/**
 * @brief 开始一次磁力计硬铁校准，清空本次三轴极值。
 * @return 成功开始或已经在采集返回 true；磁力计未就绪返回 false。
 */
bool Mag_CalibrationStart(void);

/**
 * @brief 停止校准并计算三轴硬铁偏移，先写入 RAM。Flash 由黑匣子任务另存。
 * @param status 输出校准状态，可为 NULL。
 * @return 三轴覆盖充分并成功应用偏移返回 true。
 */
bool Mag_CalibrationStop(MagCalibrationStatus_t *status);

/**
 * @brief 读取当前校准采集状态和偏移。
 * @param status 输出状态。
 */
void Mag_CalibrationGetStatus(MagCalibrationStatus_t *status);

/**
 * @brief 取消本次采集。不更新硬铁偏移，也不擦 Flash。
 */
void Mag_CalibrationCancel(void);

/**
 * @brief 把 Flash 里读出的硬铁偏移灌进 RAM。采集进行中拒绝，避免冲掉正在转的圈。
 * @param status 至少要带 valid 和三轴 offset；span 与样本数只给状态显示。
 * @return 已应用返回 true。
 */
bool Mag_CalibrationApplySaved(const MagCalibrationStatus_t *status);

#ifdef __cplusplus
}
#endif

#endif
