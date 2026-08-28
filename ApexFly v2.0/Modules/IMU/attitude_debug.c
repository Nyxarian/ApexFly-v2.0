/**
 * @file attitude_debug.c
 * @brief 观察串口任务实现。当前只打印磁力计，方便接线判型和原始值检查。
 *
 * 姿态和 GPS 文本先注释，测完磁力计后再打开。
 * 黑匣子正在 dump 时暂停打印，避免文本插进 CSV。
 */

#include "attitude_debug.h"

#include "blackbox.h"
#include "cmsis_os.h"
#include "debug.h"
#include "mag.h"
#if 0
#include "gps.h"
#include "radio.h"
#endif
#include <stdint.h>

#define ATTITUDE_DEBUG_PERIOD_MS 100U

#if 0
static void AttitudeDebug_SplitSigned(int32_t value, uint32_t scale, const char **sign, uint32_t *whole, uint32_t *fraction)
{
    uint32_t magnitude;

    if (value < 0)
    {
        *sign = "-";
        magnitude = (uint32_t)(-(int64_t)value);
    }
    else
    {
        *sign = "";
        magnitude = (uint32_t)value;
    }
    *whole = magnitude / scale;
    *fraction = magnitude % scale;
}
#endif

void AttitudeDebug_Task(void *argument)
{
#if 0
    RadioFlightStateSnapshot_t state;
    GPS_Data_t gps;
    uint32_t utc_unix_seconds;
    uint16_t utc_year;
    uint16_t beijing_year;
    uint8_t utc_month;
    uint8_t utc_day;
    uint8_t utc_hour;
    uint8_t utc_minute;
    uint8_t utc_second;
    uint8_t beijing_month;
    uint8_t beijing_day;
    uint8_t beijing_hour;
    uint8_t beijing_minute;
    uint8_t beijing_second;
    const char *latitude_sign;
    const char *longitude_sign;
    uint32_t latitude_whole;
    uint32_t latitude_fraction;
    uint32_t longitude_whole;
    uint32_t longitude_fraction;
#endif
    MagSnapshot_t mag;
    const char *chip_name;

    (void)argument;
    for (;;)
    {
        if (!Blackbox_IsDumping())
        {
#if 0
            if (Radio_GetFlightStateSnapshot(&state))
            {
                (void)Debug_PrintAttitudeAngles(state.roll_deg, state.pitch_deg, state.yaw_deg);
            }

            GPS_GetData(&gps);
            AttitudeDebug_SplitSigned(gps.latitude_e7, 10000000U, &latitude_sign, &latitude_whole, &latitude_fraction);
            AttitudeDebug_SplitSigned(gps.longitude_e7, 10000000U, &longitude_sign, &longitude_whole, &longitude_fraction);
            Debug_Printf("[GPS] fix=%u quality=%u sat=%u lat=%s%lu.%07lu lon=%s%lu.%07lu alt_cm=%ld speed_cms=%lu course=%.2f hdop=%.2f ok=%lu crc_err=%lu fmt_err=%lu dma_drop=%lu uart_err=%lu.\r\n", gps.fix_valid ? 1U : 0U, gps.fix_quality, gps.satellites, latitude_sign, (unsigned long)latitude_whole, (unsigned long)latitude_fraction, longitude_sign, (unsigned long)longitude_whole, (unsigned long)longitude_fraction, (long)gps.altitude_cm, (unsigned long)gps.ground_speed_cms, (double)gps.course_cdeg / 100.0, (double)gps.hdop_x100 / 100.0, (unsigned long)gps.valid_sentence_count, (unsigned long)gps.checksum_error_count, (unsigned long)gps.format_error_count, (unsigned long)GPS_GetDmaDropCount(), (unsigned long)GPS_GetUartErrorCount());

            if (GPS_GetUtcTime(&utc_unix_seconds))
            {
                GPS_UnixSecondsToCalendar(utc_unix_seconds, &utc_year, &utc_month, &utc_day, &utc_hour, &utc_minute, &utc_second);
                GPS_UnixSecondsToCalendar(utc_unix_seconds + 8UL * 3600UL, &beijing_year, &beijing_month, &beijing_day, &beijing_hour, &beijing_minute, &beijing_second);
                Debug_Printf("[GPS_TIME] time_valid=1 utc=%04u-%02u-%02uT%02u:%02u:%02u bjt=%04u-%02u-%02uT%02u:%02u:%02u unix=%lu.\r\n", utc_year, utc_month, utc_day, utc_hour, utc_minute, utc_second, beijing_year, beijing_month, beijing_day, beijing_hour, beijing_minute, beijing_second, (unsigned long)utc_unix_seconds);
            }
            else
            {
                Debug_Printf("[GPS_TIME] time_valid=0 utc=NA bjt=NA unix=0.\r\n");
            }
#endif
            chip_name = "NONE";
            (void)Mag_GetSnapshot(&mag);
            if (mag.chip == MAG_CHIP_HMC5883L) chip_name = "HMC5883L";
            else if (mag.chip == MAG_CHIP_QMC5883L) chip_name = "QMC5883L";
            else if (mag.chip == MAG_CHIP_QMC5883P) chip_name = "QMC5883P";
            else if (mag.chip == MAG_CHIP_QMC6308) chip_name = "QMC6308";
            Debug_Printf("[MAG] valid=%u chip=%s raw=%d,%d,%d body=%.3f,%.3f,%.3f mag=%.3f hdg=%.1f bhdg=%.1f cal=%u,%.3f,%.3f,%.3f calmag=%.3f calhdg=%.1f ok=%lu ovf=%lu err=%lu fail=%u found=%u.\r\n", mag.valid ? 1U : 0U, chip_name, (int)mag.raw_x, (int)mag.raw_y, (int)mag.raw_z, (double)mag.mx_gauss, (double)mag.my_gauss, (double)mag.mz_gauss, (double)mag.body_magnitude_gauss, (double)mag.raw_heading_deg, (double)mag.body_heading_deg, mag.calibration_valid ? 1U : 0U, (double)mag.cal_mx_gauss, (double)mag.cal_my_gauss, (double)mag.cal_mz_gauss, (double)mag.cal_magnitude_gauss, (double)mag.cal_heading_deg, (unsigned long)mag.sample_count, (unsigned long)mag.overflow_count, (unsigned long)mag.error_count, mag.probe.fail_code, mag.probe.found_count);
        }
        osDelay(ATTITUDE_DEBUG_PERIOD_MS);
    }
}
