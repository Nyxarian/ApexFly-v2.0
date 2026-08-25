/**
 * @file gps_atgm336h.h
 * @brief ATGM336H NMEA 解析。只把 USART2 收到的文本变成定位和时间快照。
 *
 * 本模块不启动 DMA、不创建任务、不写黑匣子。GPS 任务负责收字节并调用 ProcessByte。
 * 当前飞控只用时间；经纬度和高度会解析出来，但不参与控制。
 */

#ifndef GPS_ATGM336H_H
#define GPS_ATGM336H_H

#include "stm32g4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 常见 GGA/RMC 不到 90 字节，留 128 覆盖异常长句。 */
#define GPS_NMEA_SENTENCE_MAX_LENGTH 128U

typedef struct
{
    /* true 表示 GGA 质量大于 0，或 RMC 状态为 A。 */
    bool fix_valid;
    /* true 表示 RMC 同时给出了合法日期和 UTC 时分秒。 */
    bool date_time_valid;
    /* GGA 定位质量，0 表示无效。 */
    uint8_t fix_quality;
    /* 当前用于定位的卫星数。 */
    uint8_t satellites;
    /* UTC 时，0～23。 */
    uint8_t hour;
    /* UTC 分，0～59。 */
    uint8_t minute;
    /* UTC 秒，0～59。 */
    uint8_t second;
    /* UTC 日，1～31。 */
    uint8_t day;
    /* UTC 月，1～12。 */
    uint8_t month;
    /* UTC 年，两位，按 2000～2099 解释。 */
    uint8_t year;
    /* UTC Unix 秒，供黑匣子保存绝对时间。 */
    uint32_t utc_unix_seconds;
    /* 最近一次刷新 UTC 时的 HAL tick。 */
    uint32_t utc_update_tick;
    /* 纬度，单位 1e-7 度，北纬为正。 */
    int32_t latitude_e7;
    /* 经度，单位 1e-7 度，东经为正。 */
    int32_t longitude_e7;
    /* GPS 海拔，单位厘米，不是气压离地高。 */
    int32_t altitude_cm;
    /* 地速，单位厘米每秒。 */
    uint32_t ground_speed_cms;
    /* 航向，单位百分之一度。 */
    uint16_t course_cdeg;
    /* 水平精度因子，放大 100 倍。 */
    uint16_t hdop_x100;
    /* 校验通过并解析成功的 GGA/RMC 条数。 */
    uint32_t valid_sentence_count;
    /* NMEA 校验失败次数。 */
    uint32_t checksum_error_count;
    /* 语句格式错误次数。 */
    uint32_t format_error_count;
    /* 最近一次成功解析时的 HAL tick。 */
    uint32_t last_update_tick;
} GPS_Data_t;

typedef struct
{
    UART_HandleTypeDef *uart;
    char sentence[GPS_NMEA_SENTENCE_MAX_LENGTH];
    uint16_t sentence_length;
    bool collecting;
    GPS_Data_t data;
} GPS_Device_t;

/**
 * @brief 绑定 GPS 使用的 UART，并清空解析状态。
 * @param device GPS 设备对象。
 * @param uart 本板传入 &huart2。
 * @return 成功返回 true。
 */
bool GPS_ATGM336H_Init(GPS_Device_t *device, UART_HandleTypeDef *uart);

/**
 * @brief 向 NMEA 状态机送入一个已由 DMA 收到的字节。
 * @param device GPS 设备对象。
 * @param byte 一个 NMEA 字符。
 * @return true 表示刚刚成功解析了一条 GGA 或 RMC。
 */
bool GPS_ATGM336H_ProcessByte(GPS_Device_t *device, uint8_t byte);

/**
 * @brief 复制最近一次解析结果。
 * @param device 已初始化的设备。
 * @param data 输出快照。
 */
void GPS_ATGM336H_GetData(const GPS_Device_t *device, GPS_Data_t *data);

#ifdef __cplusplus
}
#endif

#endif
