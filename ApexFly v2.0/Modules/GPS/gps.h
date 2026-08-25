/**
 * @file gps.h
 * @brief ATGM336H 任务接口。USART2 DMA 收 NMEA，对外提供时间和定位快照。
 *
 * 本模块独占 USART2。当前飞控只用 UTC 时间给黑匣子打时间戳。
 * 经纬度、海拔和卫星数会解析并可供串口观察，但不参与姿态、油门或混控。
 */

#ifndef GPS_H
#define GPS_H

#include "gps_atgm336h.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief GPS 任务入口。启动 USART2 DMA 接收并持续解析 NMEA。
 * @param argument 未使用。
 */
void GPS_Task(void *argument);

/**
 * @brief 复制最近一次 GPS 快照。
 * @param data 输出。
 */
void GPS_GetData(GPS_Data_t *data);

/**
 * @brief 读取由 RMC 校准后的 UTC Unix 秒。
 * @param utc_unix_seconds 输出 UTC 秒。即使暂时没有定位，合法日期时间仍可用。
 * @return 有合法 RMC 时间返回 true。
 */
bool GPS_GetUtcTime(uint32_t *utc_unix_seconds);

/**
 * @brief 把 Unix 秒拆成日历。传入 UTC 秒得到 UTC，传入 UTC+8 秒得到北京时间。
 * @param unix_seconds Unix 秒。
 * @param year 输出年。
 * @param month 输出月，1～12。
 * @param day 输出日，1～31。
 * @param hour 输出时，0～23。
 * @param minute 输出分，0～59。
 * @param second 输出秒，0～59。
 */
void GPS_UnixSecondsToCalendar(uint32_t unix_seconds, uint16_t *year, uint8_t *month, uint8_t *day, uint8_t *hour, uint8_t *minute, uint8_t *second);

/**
 * @brief DMA 环形缓冲溢出丢掉的字节数。
 */
uint32_t GPS_GetDmaDropCount(void);

/**
 * @brief USART2 接收错误次数。
 */
uint32_t GPS_GetUartErrorCount(void);

#ifdef __cplusplus
}
#endif

#endif
