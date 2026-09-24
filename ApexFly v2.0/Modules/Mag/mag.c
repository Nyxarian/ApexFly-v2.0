/**
 * @file mag.c
 * @brief 磁力计任务：初始化 GY-273，定时读三轴，发布快照给串口观察。
 *
 * 芯片系到机体系：mx = -y，my = x，mz = z。一阶低通后再发布。
 * 实测（2026-09-10，换位置后）芯片电气轴是 X 右、Y 后、Z 下，与丝印标注差 180°。
 * 换安装位置或转朝向都必须重新推导这个映射，并重做硬铁校准，旧 Flash 偏移不能再用。
 * 控制环不要在这里读 I2C。航向融合和偏航外环以后再接。
 * 硬铁校准可由串口或遥控 MAG_CAL 操作码触发；成功后黑匣子写入 0x002000，上电再加载。
 */

#include "mag.h"

#include "cmsis_os.h"
#include "debug.h"
#include "blackbox.h"
#include "i2c.h"
#include "i2c2_lock.h"
#include "task.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

#define MAG_POLL_PERIOD_MS 20U
#define MAG_RETRY_PERIOD_MS 1000U
#define MAG_MISS_REINIT_COUNT 150U
#define MAG_RAD_TO_DEG 57.2957795f
/* 50 Hz 采样时约 2.5 Hz 带宽，只磨噪声，不把航向拖慢。 */
#define MAG_BODY_LPF_ALPHA 0.25f
#define MAG_CAL_MIN_SAMPLES 100U
#define MAG_CAL_MIN_SPAN_GAUSS 0.25f

static MagDevice_t mag_device;
static MagSnapshot_t mag_snapshot;
static MagCalibrationStatus_t mag_calibration;
static float mag_mx_filtered_gauss;
static float mag_my_filtered_gauss;
static float mag_mz_filtered_gauss;

static bool mag_body_filter_ready;

static float Mag_WrapHeadingDeg(float heading_deg)
{
    if (heading_deg < 0.0f) heading_deg += 360.0f;
    if (heading_deg >= 360.0f) heading_deg -= 360.0f;
    return heading_deg;
}

static float Mag_RawHeadingDeg(float x_gauss, float y_gauss)
{
    if ((x_gauss == 0.0f) && (y_gauss == 0.0f)) return 0.0f;
    return Mag_WrapHeadingDeg(atan2f(y_gauss, x_gauss) * MAG_RAD_TO_DEG);
}

static float Mag_BodyHeadingDeg(float mx_gauss, float my_gauss)
{
    /* 机体系 Y 朝右、偏航顺时针为正：机头相对磁北 = atan2(-my, mx)。 */
    if ((mx_gauss == 0.0f) && (my_gauss == 0.0f)) return 0.0f;
    return Mag_WrapHeadingDeg(atan2f(-my_gauss, mx_gauss) * MAG_RAD_TO_DEG);
}

static float Mag_Lpf(float previous, float sample, bool ready)
{
    if (!ready) return sample;
    return previous + (MAG_BODY_LPF_ALPHA * (sample - previous));
}

static void Mag_ChipToBody(float x_gauss, float y_gauss, float z_gauss, float *mx_gauss, float *my_gauss, float *mz_gauss)
{
    /* 换安装位置后实测（机头朝北、机身水平、|B|=0.51G）：
       水平磁场完全落在芯片 -Y 上，Z 读数为正的完整垂直分量。
       结论：芯片实际电气轴是 X -> 机体右、Y -> 机体后、Z -> 机体下，
       与模块丝印标注差 180°。不要在航向上另加 90° 偏移。
       注意：换位置或转朝向都必须重新推导本映射，并重做硬铁校准。 */
    *mx_gauss = -y_gauss;
    *my_gauss = x_gauss;
    *mz_gauss = z_gauss;
}

static void Mag_CalibrationUpdate(float mx_gauss, float my_gauss, float mz_gauss)
{
    taskENTER_CRITICAL();
    if (mag_calibration.active)
    {
        if (mag_calibration.sample_count == 0U)
        {
            mag_calibration.min_mx_gauss = mx_gauss;
            mag_calibration.min_my_gauss = my_gauss;
            mag_calibration.min_mz_gauss = mz_gauss;
            mag_calibration.max_mx_gauss = mx_gauss;
            mag_calibration.max_my_gauss = my_gauss;
            mag_calibration.max_mz_gauss = mz_gauss;
        }
        else
        {
            if (mx_gauss < mag_calibration.min_mx_gauss) mag_calibration.min_mx_gauss = mx_gauss;
            if (my_gauss < mag_calibration.min_my_gauss) mag_calibration.min_my_gauss = my_gauss;
            if (mz_gauss < mag_calibration.min_mz_gauss) mag_calibration.min_mz_gauss = mz_gauss;
            if (mx_gauss > mag_calibration.max_mx_gauss) mag_calibration.max_mx_gauss = mx_gauss;
            if (my_gauss > mag_calibration.max_my_gauss) mag_calibration.max_my_gauss = my_gauss;
            if (mz_gauss > mag_calibration.max_mz_gauss) mag_calibration.max_mz_gauss = mz_gauss;
        }
        mag_calibration.sample_count++;
    }
    taskEXIT_CRITICAL();
}

static void Mag_CalibrationRefreshQuality(void)
{
    mag_calibration.span_mx_gauss = mag_calibration.max_mx_gauss - mag_calibration.min_mx_gauss;
    mag_calibration.span_my_gauss = mag_calibration.max_my_gauss - mag_calibration.min_my_gauss;
    mag_calibration.span_mz_gauss = mag_calibration.max_mz_gauss - mag_calibration.min_mz_gauss;
    mag_calibration.quality_ok = (mag_calibration.sample_count >= MAG_CAL_MIN_SAMPLES) && (mag_calibration.span_mx_gauss >= MAG_CAL_MIN_SPAN_GAUSS) && (mag_calibration.span_my_gauss >= MAG_CAL_MIN_SPAN_GAUSS) && (mag_calibration.span_mz_gauss >= MAG_CAL_MIN_SPAN_GAUSS);
}

static void Mag_CalibrationCopy(MagCalibrationStatus_t *status)
{
    if (status == NULL) return;
    taskENTER_CRITICAL();
    *status = mag_calibration;
    taskEXIT_CRITICAL();
}

static void Mag_ClearSnapshot(bool keep_chip)
{
    MagChip_t chip = mag_snapshot.chip;
    uint8_t address = mag_snapshot.address_7bit;

    taskENTER_CRITICAL();
    memset(&mag_snapshot, 0, sizeof(mag_snapshot));
    if (keep_chip)
    {
        mag_snapshot.chip = chip;
        mag_snapshot.address_7bit = address;
    }
    taskEXIT_CRITICAL();
}

/**
 * @brief 打印一次 I2C2 总线扫描结果：应答的设备个数与全部 7 位地址。
 *
 * 用途是排查"新挂的设备地址撞不撞车、接线通不通"。当前 I2C2 上应有两个设备：
 *   0x2C = QMC5883P 磁力计
 *   0x29 = VL53L1X 激光测距（接入后）
 * 缺了 0x2C 说明磁力计没通；缺了 0x29 说明激光没接好或 XSHUT 没被拉高。
 *
 * 只在每次初始化尝试时调一次：成功时上电打一行，失败时随 1 秒重试节奏重打，便于插拔排查。
 * 本函数不做总线访问，打印不会占用 I2C2。
 */
static void Mag_PrintBusScan(const MagProbe_t *probe)
{
    if (probe == NULL) return;

    if (probe->found_count == 0U)
    {
        Debug_Printf("[I2C2] scan: no device responded (fail=%u).\r\n", (unsigned)probe->fail_code);
        return;
    }
}

void Mag_Task(void *argument)
{
    MagRawData_t data;
    MagProbe_t probe;
    uint16_t miss_count;
    float body_mx_gauss;
    float body_my_gauss;
    float body_mz_gauss;

    (void)argument;
    I2C2_Lock_Init();
    for (;;)
    {
        (void)Mag_ScanBus(&hi2c2, &probe);
        Mag_PrintBusScan(&probe);
        if (!Mag_DeviceInit(&mag_device, &hi2c2))
        {
            if ((probe.fail_code == 0U) && (probe.found_count > 0U)) probe.fail_code = 2U;
            if (probe.found_count == 0U)
            {
                /* I2C2 现在只挂磁力计。扫不到通常是卡死或杜邦线接触不良。 */
                I2C2_BusRecover();
                if (probe.fail_code == 1U) probe.fail_code = 4U;
            }
            Mag_ClearSnapshot(false);
            taskENTER_CRITICAL();
            mag_snapshot.probe = probe;
            taskEXIT_CRITICAL();
            osDelay(MAG_RETRY_PERIOD_MS);
            continue;
        }

        miss_count = 0U;
        mag_body_filter_ready = false;
        Mag_ClearSnapshot(true);
        taskENTER_CRITICAL();
        mag_snapshot.chip = Mag_DeviceChip(&mag_device);
        mag_snapshot.address_7bit = Mag_DeviceAddress7Bit(&mag_device);
        mag_snapshot.probe = probe;
        mag_snapshot.probe.fail_code = 0U;
        taskEXIT_CRITICAL();
        /* 黑匣子可能比本任务更早读完 Flash；芯片就绪后再灌一次，避免上电窗口丢偏移。 */
        (void)Blackbox_ApplySavedMagCalibration();

        for (;;)
        {
            if (!Mag_DeviceRead(&mag_device, &data))
            {
                taskENTER_CRITICAL();
                mag_snapshot.error_count++;
                if (data.overflow) mag_snapshot.overflow_count++;
                taskEXIT_CRITICAL();
                /* 饱和只丢这一拍；连续通信失败才重新探测芯片。 */
                if (!data.overflow) miss_count++;
                if (miss_count >= MAG_MISS_REINIT_COUNT) break;
                osDelay(MAG_POLL_PERIOD_MS);
                continue;
            }

            miss_count = 0U;
            Mag_ChipToBody(data.x_gauss, data.y_gauss, data.z_gauss, &body_mx_gauss, &body_my_gauss, &body_mz_gauss);
            Mag_CalibrationUpdate(body_mx_gauss, body_my_gauss, body_mz_gauss);
            mag_mx_filtered_gauss = Mag_Lpf(mag_mx_filtered_gauss, body_mx_gauss, mag_body_filter_ready);
            mag_my_filtered_gauss = Mag_Lpf(mag_my_filtered_gauss, body_my_gauss, mag_body_filter_ready);
            mag_mz_filtered_gauss = Mag_Lpf(mag_mz_filtered_gauss, body_mz_gauss, mag_body_filter_ready);
            mag_body_filter_ready = true;
            taskENTER_CRITICAL();
            mag_snapshot.chip = Mag_DeviceChip(&mag_device);
            mag_snapshot.address_7bit = Mag_DeviceAddress7Bit(&mag_device);
            mag_snapshot.raw_x = data.raw_x;
            mag_snapshot.raw_y = data.raw_y;
            mag_snapshot.raw_z = data.raw_z;
            mag_snapshot.x_gauss = data.x_gauss;
            mag_snapshot.y_gauss = data.y_gauss;
            mag_snapshot.z_gauss = data.z_gauss;
            mag_snapshot.magnitude_gauss = data.magnitude_gauss;
            mag_snapshot.mx_gauss = mag_mx_filtered_gauss;
            mag_snapshot.my_gauss = mag_my_filtered_gauss;
            mag_snapshot.mz_gauss = mag_mz_filtered_gauss;
            mag_snapshot.body_magnitude_gauss = sqrtf((mag_mx_filtered_gauss * mag_mx_filtered_gauss) + (mag_my_filtered_gauss * mag_my_filtered_gauss) + (mag_mz_filtered_gauss * mag_mz_filtered_gauss));
            mag_snapshot.calibration_valid = mag_calibration.valid;
            mag_snapshot.cal_mx_gauss = mag_mx_filtered_gauss - mag_calibration.offset_mx_gauss;
            mag_snapshot.cal_my_gauss = mag_my_filtered_gauss - mag_calibration.offset_my_gauss;
            mag_snapshot.cal_mz_gauss = mag_mz_filtered_gauss - mag_calibration.offset_mz_gauss;
            mag_snapshot.cal_magnitude_gauss = sqrtf((mag_snapshot.cal_mx_gauss * mag_snapshot.cal_mx_gauss) + (mag_snapshot.cal_my_gauss * mag_snapshot.cal_my_gauss) + (mag_snapshot.cal_mz_gauss * mag_snapshot.cal_mz_gauss));
            mag_snapshot.cal_heading_deg = Mag_BodyHeadingDeg(mag_snapshot.cal_mx_gauss, mag_snapshot.cal_my_gauss);
            mag_snapshot.raw_heading_deg = Mag_RawHeadingDeg(data.x_gauss, data.y_gauss);
            mag_snapshot.body_heading_deg = Mag_BodyHeadingDeg(mag_mx_filtered_gauss, mag_my_filtered_gauss);
            mag_snapshot.sample_count = data.sample_count;
            mag_snapshot.valid = true;
            taskEXIT_CRITICAL();
            osDelay(MAG_POLL_PERIOD_MS);
        }
    }
}

bool Mag_GetSnapshot(MagSnapshot_t *snapshot)
{
    bool valid;

    if (snapshot == NULL) return false;
    taskENTER_CRITICAL();
    *snapshot = mag_snapshot;
    valid = mag_snapshot.valid;
    taskEXIT_CRITICAL();
    return valid;
}

bool Mag_CalibrationStart(void)
{
    bool started = false;

    taskENTER_CRITICAL();
    if (mag_calibration.active)
    {
        started = true;
    }
    else if (mag_snapshot.valid || (mag_snapshot.chip != MAG_CHIP_NONE))
    {
        MagCalibrationStatus_t previous = mag_calibration;

        memset(&mag_calibration, 0, sizeof(mag_calibration));
        mag_calibration.active = true;
        /* 新一次没算出来之前继续用旧偏移，失败也不把已保存的北冲掉。 */
        if (previous.valid)
        {
            mag_calibration.valid = true;
            mag_calibration.offset_mx_gauss = previous.offset_mx_gauss;
            mag_calibration.offset_my_gauss = previous.offset_my_gauss;
            mag_calibration.offset_mz_gauss = previous.offset_mz_gauss;
        }
        started = true;
    }
    taskEXIT_CRITICAL();
    if (started) Debug_Printf("[MAG_CAL] collecting; rotate all 3 axes, then stop.\r\n");
    else Debug_Printf("[MAG_CAL] start fail; magnetometer is not ready.\r\n");
    return started;
}

bool Mag_CalibrationStop(MagCalibrationStatus_t *status)
{
    bool ok;
    bool was_active;
    MagCalibrationStatus_t local;

    taskENTER_CRITICAL();
    was_active = mag_calibration.active;
    if (!was_active)
    {
        ok = mag_calibration.valid;
    }
    else
    {
        mag_calibration.active = false;
        Mag_CalibrationRefreshQuality();
        ok = mag_calibration.quality_ok;
        if (ok)
        {
            mag_calibration.offset_mx_gauss = (mag_calibration.max_mx_gauss + mag_calibration.min_mx_gauss) * 0.5f;
            mag_calibration.offset_my_gauss = (mag_calibration.max_my_gauss + mag_calibration.min_my_gauss) * 0.5f;
            mag_calibration.offset_mz_gauss = (mag_calibration.max_mz_gauss + mag_calibration.min_mz_gauss) * 0.5f;
            mag_calibration.valid = true;
        }
    }
    local = mag_calibration;
    if (status != NULL) *status = local;
    taskEXIT_CRITICAL();
    if (was_active)
    {
        if (ok) Debug_Printf("[MAG_CAL] done samples=%lu span=%.3f,%.3f,%.3f offset=%.3f,%.3f,%.3f.\r\n", (unsigned long)local.sample_count, (double)local.span_mx_gauss, (double)local.span_my_gauss, (double)local.span_mz_gauss, (double)local.offset_mx_gauss, (double)local.offset_my_gauss, (double)local.offset_mz_gauss);
        else Debug_Printf("[MAG_CAL] rejected samples=%lu span=%.3f,%.3f,%.3f.\r\n", (unsigned long)local.sample_count, (double)local.span_mx_gauss, (double)local.span_my_gauss, (double)local.span_mz_gauss);
    }
    return ok;
}

void Mag_CalibrationGetStatus(MagCalibrationStatus_t *status)
{
    Mag_CalibrationCopy(status);
}

void Mag_CalibrationCancel(void)
{
    taskENTER_CRITICAL();
    mag_calibration.active = false;
    mag_calibration.quality_ok = false;
    taskEXIT_CRITICAL();
    Debug_Printf("[MAG_CAL] cancelled.\r\n");
}

bool Mag_CalibrationApplySaved(const MagCalibrationStatus_t *status)
{
    if ((status == NULL) || !status->valid) return false;

    taskENTER_CRITICAL();
    if (mag_calibration.active)
    {
        taskEXIT_CRITICAL();
        return false;
    }
    /* 已经有 RAM 偏移时不要用 Flash 旧值盖掉，避免 STOP 后、写扇区前被任务重初始化冲掉。 */
    if (mag_calibration.valid)
    {
        taskEXIT_CRITICAL();
        return true;
    }
    mag_calibration.valid = true;
    mag_calibration.quality_ok = true;
    mag_calibration.sample_count = status->sample_count;
    mag_calibration.offset_mx_gauss = status->offset_mx_gauss;
    mag_calibration.offset_my_gauss = status->offset_my_gauss;
    mag_calibration.offset_mz_gauss = status->offset_mz_gauss;
    mag_calibration.span_mx_gauss = status->span_mx_gauss;
    mag_calibration.span_my_gauss = status->span_my_gauss;
    mag_calibration.span_mz_gauss = status->span_mz_gauss;
    taskEXIT_CRITICAL();
    return true;
}
