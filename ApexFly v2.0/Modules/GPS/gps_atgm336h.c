/**
 * @file gps_atgm336h.c
 * @brief ATGM336H NMEA 解析实现。只处理 GGA 和 RMC。
 *
 * 校验失败的语句直接丢掉。时间只认 RMC 的完整日期和时间，GGA 只补充位置和卫星数。
 */

#include "gps_atgm336h.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define GPS_NMEA_MAX_FIELDS 20U

static int8_t GPS_HexValue(char character)
{
    if ((character >= '0') && (character <= '9')) return (int8_t)(character - '0');
    if ((character >= 'A') && (character <= 'F')) return (int8_t)(character - 'A' + 10);
    if ((character >= 'a') && (character <= 'f')) return (int8_t)(character - 'a' + 10);
    return -1;
}

static bool GPS_CheckChecksum(char *sentence, char **checksum_mark)
{
    uint8_t checksum = 0U;
    char *cursor;
    char *asterisk;
    int8_t high;
    int8_t low;

    if ((sentence == NULL) || (sentence[0] != '$')) return false;
    asterisk = strchr(sentence, '*');
    if ((asterisk == NULL) || (asterisk[1] == '\0') || (asterisk[2] == '\0')) return false;

    for (cursor = &sentence[1]; cursor < asterisk; cursor++) checksum ^= (uint8_t)(*cursor);
    high = GPS_HexValue(asterisk[1]);
    low = GPS_HexValue(asterisk[2]);
    if ((high < 0) || (low < 0)) return false;
    if (checksum != (uint8_t)(((uint8_t)high << 4) | (uint8_t)low)) return false;

    *checksum_mark = asterisk;
    return true;
}

static uint8_t GPS_SplitFields(char *payload, char *fields[], uint8_t maximum_fields)
{
    uint8_t count = 0U;
    char *cursor;

    if ((payload == NULL) || (fields == NULL) || (maximum_fields == 0U)) return 0U;
    fields[count++] = payload;
    for (cursor = payload; (*cursor != '\0') && (count < maximum_fields); cursor++)
    {
        if (*cursor == ',')
        {
            *cursor = '\0';
            fields[count++] = cursor + 1;
        }
    }
    return count;
}

static bool GPS_IsSentenceType(const char *field, const char *type)
{
    size_t length;

    if ((field == NULL) || (type == NULL)) return false;
    length = strlen(field);
    if (length < 3U) return false;
    return strcmp(&field[length - 3U], type) == 0;
}

static bool GPS_FieldHasDigits(const char *field, uint8_t count)
{
    uint8_t index;
    if ((field == NULL) || (strlen(field) < count)) return false;
    for (index = 0U; index < count; index++)
    {
        if ((field[index] < '0') || (field[index] > '9')) return false;
    }
    return true;
}

static bool GPS_ParseTime(const char *field, uint8_t *hour, uint8_t *minute, uint8_t *second)
{
    uint8_t parsed_hour;
    uint8_t parsed_minute;
    uint8_t parsed_second;
    if ((hour == NULL) || (minute == NULL) || (second == NULL) || (!GPS_FieldHasDigits(field, 6U))) return false;
    parsed_hour = (uint8_t)((field[0] - '0') * 10 + (field[1] - '0'));
    parsed_minute = (uint8_t)((field[2] - '0') * 10 + (field[3] - '0'));
    parsed_second = (uint8_t)((field[4] - '0') * 10 + (field[5] - '0'));
    if ((parsed_hour > 23U) || (parsed_minute > 59U) || (parsed_second > 59U)) return false;
    *hour = parsed_hour;
    *minute = parsed_minute;
    *second = parsed_second;
    return true;
}

static bool GPS_IsLeapYear(uint16_t year)
{
    return ((year % 4U) == 0U) && (((year % 100U) != 0U) || ((year % 400U) == 0U));
}

static uint8_t GPS_DaysInMonth(uint16_t year, uint8_t month)
{
    static const uint8_t days_per_month[12] = {31U, 28U, 31U, 30U, 31U, 30U, 31U, 31U, 30U, 31U, 30U, 31U};
    if ((month == 0U) || (month > 12U)) return 0U;
    if ((month == 2U) && GPS_IsLeapYear(year)) return 29U;
    return days_per_month[month - 1U];
}

static bool GPS_ParseDate(const char *field, uint8_t *day, uint8_t *month, uint8_t *year)
{
    uint8_t parsed_day;
    uint8_t parsed_month;
    uint8_t parsed_year;
    if ((day == NULL) || (month == NULL) || (year == NULL) || (!GPS_FieldHasDigits(field, 6U))) return false;
    parsed_day = (uint8_t)((field[0] - '0') * 10 + (field[1] - '0'));
    parsed_month = (uint8_t)((field[2] - '0') * 10 + (field[3] - '0'));
    parsed_year = (uint8_t)((field[4] - '0') * 10 + (field[5] - '0'));
    if ((parsed_month == 0U) || (parsed_month > 12U) || (parsed_day == 0U) || (parsed_day > GPS_DaysInMonth((uint16_t)(2000U + parsed_year), parsed_month))) return false;
    *day = parsed_day;
    *month = parsed_month;
    *year = parsed_year;
    return true;
}

static uint32_t GPS_DateTimeToUnixSeconds(uint8_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second)
{
    uint16_t full_year;
    uint16_t current_year;
    uint8_t current_month;
    uint32_t days = 0U;
    full_year = (uint16_t)(2000U + year);
    for (current_year = 1970U; current_year < full_year; current_year++) days += GPS_IsLeapYear(current_year) ? 366U : 365U;
    for (current_month = 1U; current_month < month; current_month++) days += GPS_DaysInMonth(full_year, current_month);
    days += (uint32_t)day - 1U;
    return days * 86400UL + (uint32_t)hour * 3600UL + (uint32_t)minute * 60UL + second;
}

static int32_t GPS_ParseCoordinateE7(const char *field, char hemisphere)
{
    double raw;
    double decimal_degree;
    int32_t degrees;
    int32_t scaled;

    if ((field == NULL) || (field[0] == '\0')) return 0;
    raw = strtod(field, NULL);
    degrees = (int32_t)(raw / 100.0);
    decimal_degree = (double)degrees + (raw - ((double)degrees * 100.0)) / 60.0;
    scaled = (int32_t)(decimal_degree * 10000000.0 + 0.5);
    if ((hemisphere == 'S') || (hemisphere == 'W')) scaled = -scaled;
    return scaled;
}

static bool GPS_ParseGGA(char *fields[], uint8_t count, GPS_Data_t *data)
{
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    if ((count < 10U) || (data == NULL)) return false;

    if (GPS_ParseTime(fields[1], &hour, &minute, &second))
    {
        data->hour = hour;
        data->minute = minute;
        data->second = second;
    }
    data->latitude_e7 = GPS_ParseCoordinateE7(fields[2], fields[3][0]);
    data->longitude_e7 = GPS_ParseCoordinateE7(fields[4], fields[5][0]);
    data->fix_quality = (uint8_t)strtoul(fields[6], NULL, 10);
    data->fix_valid = data->fix_quality > 0U;
    data->satellites = (uint8_t)strtoul(fields[7], NULL, 10);
    data->hdop_x100 = (uint16_t)(strtod(fields[8], NULL) * 100.0 + 0.5);
    data->altitude_cm = (int32_t)(strtod(fields[9], NULL) * 100.0);
    return true;
}

static bool GPS_ParseRMC(char *fields[], uint8_t count, GPS_Data_t *data)
{
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint8_t day;
    uint8_t month;
    uint8_t year;
    if ((count < 10U) || (data == NULL)) return false;
    if ((!GPS_ParseTime(fields[1], &hour, &minute, &second)) || (!GPS_ParseDate(fields[9], &day, &month, &year))) return false;

    data->hour = hour;
    data->minute = minute;
    data->second = second;
    data->day = day;
    data->month = month;
    data->year = year;
    data->utc_unix_seconds = GPS_DateTimeToUnixSeconds(year, month, day, hour, minute, second);
    data->utc_update_tick = HAL_GetTick();
    data->date_time_valid = true;
    data->fix_valid = fields[2][0] == 'A';
    data->latitude_e7 = GPS_ParseCoordinateE7(fields[3], fields[4][0]);
    data->longitude_e7 = GPS_ParseCoordinateE7(fields[5], fields[6][0]);
    data->ground_speed_cms = (uint32_t)(strtod(fields[7], NULL) * 51.444444 + 0.5);
    data->course_cdeg = (uint16_t)(strtod(fields[8], NULL) * 100.0 + 0.5);
    return true;
}

static bool GPS_ParseSentence(GPS_Device_t *device)
{
    char *checksum_mark;
    char *fields[GPS_NMEA_MAX_FIELDS];
    uint8_t field_count;
    bool parsed = false;

    if (!GPS_CheckChecksum(device->sentence, &checksum_mark))
    {
        device->data.checksum_error_count++;
        return false;
    }

    *checksum_mark = '\0';
    field_count = GPS_SplitFields(&device->sentence[1], fields, GPS_NMEA_MAX_FIELDS);
    if (field_count == 0U)
    {
        device->data.format_error_count++;
        return false;
    }

    if (GPS_IsSentenceType(fields[0], "GGA")) parsed = GPS_ParseGGA(fields, field_count, &device->data);
    else if (GPS_IsSentenceType(fields[0], "RMC")) parsed = GPS_ParseRMC(fields, field_count, &device->data);
    else return false;

    if (!parsed)
    {
        device->data.format_error_count++;
        return false;
    }

    device->data.valid_sentence_count++;
    device->data.last_update_tick = HAL_GetTick();
    return true;
}

bool GPS_ATGM336H_Init(GPS_Device_t *device, UART_HandleTypeDef *uart)
{
    if ((device == NULL) || (uart == NULL)) return false;
    memset(device, 0, sizeof(*device));
    device->uart = uart;
    return true;
}

bool GPS_ATGM336H_ProcessByte(GPS_Device_t *device, uint8_t byte)
{
    if (device == NULL) return false;

    if (byte == '$')
    {
        device->collecting = true;
        device->sentence_length = 0U;
        device->sentence[device->sentence_length++] = (char)byte;
        return false;
    }

    if (!device->collecting) return false;
    if (byte == '\n')
    {
        device->sentence[device->sentence_length] = '\0';
        device->collecting = false;
        return GPS_ParseSentence(device);
    }

    if (byte != '\r')
    {
        if (device->sentence_length >= (GPS_NMEA_SENTENCE_MAX_LENGTH - 1U))
        {
            device->collecting = false;
            device->sentence_length = 0U;
            device->data.format_error_count++;
            return false;
        }
        device->sentence[device->sentence_length++] = (char)byte;
    }
    return false;
}

void GPS_ATGM336H_GetData(const GPS_Device_t *device, GPS_Data_t *data)
{
    if ((device == NULL) || (data == NULL)) return;
    *data = device->data;
}
