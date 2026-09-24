/**
 * @file blackbox.c
 * @brief W25Q64 黑匣子实现：PID 双备份、场次日志、串口 CSV 导出。
 *
 * 分区：
 * 0x000000 参数槽 A，4KB
 * 0x001000 参数槽 B，4KB
 * 0x002000 磁力计硬铁校准，扇区内 A/B 各 64 字节
 * 0x003000 日志写指针索引，4KB
 * 0x004000 日志起点
 * 最后 4KB 自检扇区，不拿来记飞
 *
 * 日志记录固定 64 字节。写满后回到 0x004000 覆盖最老的数据，写指针保存在 0x003000。
 * dump [N] 从当前写指针往回取最近 N 条，不是从芯片物理末尾取。
 * 场次头里的 PID 用 q100（百分之一）钉死当时那套参，方便导出后对上波形。
 * 解锁 UTC 写在高度参数记录的保留区，上锁 UTC 写在场次尾；样本仍是本场相对毫秒。
 * USART3 轮询收 dump / dumpall / help；SAVE / 日志写入只在本任务做。
 */

#include "blackbox.h"

#include "apex_protocol.h"
#include "debug.h"
#include "gps.h"
#include "main.h"
#include "mag.h"
#include "pid_table.h"
#include "spi.h"
#include "usart.h"
#include "vertical_control.h"
#include "w25q64.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define BLACKBOX_PARAMETER_SECTOR_A 0x000000UL
#define BLACKBOX_PARAMETER_SECTOR_B 0x001000UL
#define BLACKBOX_MAG_CAL_SECTOR 0x002000UL
#define BLACKBOX_MAG_CAL_SLOT_A_OFFSET 0U
#define BLACKBOX_MAG_CAL_SLOT_B_OFFSET 64U
#define BLACKBOX_MAG_CAL_RECORD_SIZE 64U
#define BLACKBOX_MAG_CAL_MAGIC 0x4D414734UL /* "MAG4"，朝北/朝西实测后改为 -X 前、Y 右，与 MAG3 不兼容。 */
#define BLACKBOX_MAG_CAL_SCHEMA 3U
#define BLACKBOX_LOG_INDEX_ADDRESS 0x003000UL
#define BLACKBOX_LOG_START_ADDRESS 0x004000UL
#define BLACKBOX_TEST_SECTOR_ADDRESS (W25Q64_TOTAL_SIZE_BYTES - W25Q64_SECTOR_SIZE_BYTES)
#define BLACKBOX_LOG_END_ADDRESS BLACKBOX_TEST_SECTOR_ADDRESS
#define BLACKBOX_LOG_INDEX_MAGIC 0x424C5849UL /* "BLXI" */
#define BLACKBOX_LOG_INDEX_RECORD_SIZE 32U

#define BLACKBOX_PARAMETER_MAGIC 0x50445032UL /* "PDP2"，与老工程 magic 区分。 */
/* Flash 参数表记录的格式版本。⚠️ 2026-09-15 由 2 抬到 3：
   参数个数从 30 变成 33（加了高度环 3 项），而记录里带 parameter_count 校验，
   不抬版本的话**旧记录会被判成无效**（LOAD_SAVED 直接失败）。
   抬版本 + 在 Blackbox_ReadSlot 里加一条 30 项的迁移分支，
   旧记录会被读出并按当前默认值补齐新增的三项。 */
#define BLACKBOX_PARAMETER_SCHEMA 3U
/* 旧格式（26 项，v6）与上一版（30 项，v7）。数值不能改，它们是历史存档的标记。 */
#define BLACKBOX_PARAMETER_LEGACY_SCHEMA 1U
#define BLACKBOX_PARAMETER_V7_SCHEMA 2U
#define BLACKBOX_SESSION_PID_PARAMETER_COUNT APEX_PID_V6_PARAMETER_COUNT
#define BLACKBOX_RECORD_MAGIC 0x424C5832UL /* "BLX2" */
#define BLACKBOX_RECORD_SCHEMA 3U
#define BLACKBOX_RECORD_TYPE_SESSION_BEGIN 1U
#define BLACKBOX_RECORD_TYPE_SAMPLE 2U
#define BLACKBOX_RECORD_TYPE_SESSION_END 3U
#define BLACKBOX_RECORD_TYPE_ALTITUDE_PARAMS 4U
#define BLACKBOX_RECORD_TYPE_POSITION_SAMPLE 5U
#define BLACKBOX_RECORD_SIZE 64U
#define BLACKBOX_RECORD_CRC_OFFSET 60U
#define BLACKBOX_SAMPLE_QUEUE_LENGTH 32U
#define BLACKBOX_POSITION_SAMPLE_PERIOD_MS 80U

#define BLACKBOX_UART (&huart3)
#define BLACKBOX_COMMAND_BUFFER_SIZE 40U
#define BLACKBOX_UART_POLL_MS 5U
#define BLACKBOX_DUMP_DEFAULT_RECORDS 20000U

typedef struct
{
    uint32_t magic;
    uint16_t schema;
    uint16_t parameter_count;
    uint32_t generation;
    int32_t values_q10000[APEX_PID_PARAMETER_COUNT];
    uint16_t crc16;
    uint16_t reserved;
} BlackboxParameterRecord_t;

typedef struct
{
    uint32_t magic;
    uint16_t schema;
    uint16_t parameter_count;
    uint32_t generation;
    int32_t values_q10000[APEX_PID_V6_PARAMETER_COUNT];
    uint16_t crc16;
    uint16_t reserved;
} BlackboxLegacyParameterRecord_t;

/* 上一版（30 项）的参数记录。字段顺序必须与 BlackboxParameterRecord_t 完全一致，
   迁移时才能用 memcpy 把它的 payload 直接搬过来。 */
typedef struct
{
    uint32_t magic;
    uint16_t schema;
    uint16_t parameter_count;
    uint32_t generation;
    int32_t values_q10000[APEX_PID_V7_PARAMETER_COUNT];
    uint16_t crc16;
    uint16_t reserved;
} BlackboxLegacyV7ParameterRecord_t;

typedef struct
{
    uint32_t magic;
    uint32_t write_address;
    uint16_t next_session_id;
    uint16_t wrapped;
    uint32_t generation;
    uint16_t crc16;
    uint16_t reserved;
} BlackboxLogIndexRecord_t;

#pragma pack(push, 1)
typedef struct
{
    uint32_t magic;
    uint8_t type;
    uint8_t schema;
    uint16_t session_id;
    int16_t pid_q100[BLACKBOX_SESSION_PID_PARAMETER_COUNT];
    uint16_t crc16;
    uint16_t reserved;
} BlackboxSessionBeginRecord_t;

typedef struct
{
    uint32_t magic;
    uint8_t type;
    uint8_t schema;
    uint16_t session_id;
    /* 垂速环（内环）三增益 + 满杆油门。**这四项的偏移与长度一个字节都不能动**。 */
    int32_t altitude_q10000[4];
    /* 解锁瞬间的 UTC Unix 秒。没有合法 RMC 时为 0。老记录该处是 0。 */
    uint32_t utc_seconds;
    /* 高度环（外环）三增益。⚠️ 2026-09-15 新增，**刻意放在 utc 之后**，
       占用的是原来 reserved0 的前 12 字节，于是：
         · altitude_q10000 与 utc_seconds 的偏移**完全没变** ⇒ 旧记录读出来照样正确；
         · 旧记录这 12 字节是全 0（memset 过） ⇒ 外环三项读成 0 = "外环关闭"，
           而那时外环确实还不存在，语义也对；
         · CRC 覆盖的仍是前 60 字节 ⇒ 旧记录的 CRC 依然校验通过。
       ⇒ 因此**不需要**动 BLACKBOX_RECORD_SCHEMA，新旧记录用同一个结构都能读。 */
    int32_t height_q10000[3];
    uint8_t reserved0[20];
    uint16_t crc16;
    uint16_t reserved1;
} BlackboxAltitudeParamsRecord_t;

typedef struct
{
    uint32_t magic;
    uint8_t type;
    uint8_t schema;
    uint16_t session_id;
    uint32_t t_ms;
    int16_t roll_cdeg;
    int16_t pitch_cdeg;
    int16_t yaw_cdeg;
    int16_t roll_rate_cdps;
    int16_t pitch_rate_cdps;
    int16_t yaw_rate_cdps;
    int16_t roll_corr_us;
    int16_t pitch_corr_us;
    int16_t yaw_corr_us;
    uint16_t throttle_us;
    uint16_t motor_us[4];
    int16_t altitude_cm;
    int16_t vz_cms;
    /* 保持 19 列 schema 2 的 armed/connected 字节位置；更早 17 列布局由导出器单独兼容。 */
    uint8_t armed;
    uint8_t connected;
    /* 磁航向，单位百分之一度。无效 -1。这一格不是"目标垂速"，旧名是历史遗留。 */
    int16_t mag_heading_cdeg;
    int16_t vz_correction_us;
    /* 目标垂速，单位厘米每秒，向上为正。原来是"本拍最高油门"，偏移与宽度不变。 */
    int16_t vz_target_cms;
    uint8_t altitude_state;
    uint8_t pad[7];
    uint16_t crc16;
    uint16_t reserved1;
} BlackboxSampleRecord_t;

typedef struct
{
    uint32_t magic;
    uint8_t type;
    uint8_t schema;
    uint16_t session_id;
    uint32_t t_ms;
    int16_t flow_dx;
    int16_t flow_dy;
    uint16_t status;
    int32_t flow_sum_x_q10;
    int32_t flow_sum_y_q10;
    int32_t flow_zero_x_q1000;
    int32_t flow_zero_y_q1000;
    int32_t velocity_forward_q10;
    int32_t velocity_right_q10;
    int32_t velocity_forward_sp_q10;
    int32_t velocity_right_sp_q10;
    int16_t raw_roll_cdeg;
    int16_t raw_pitch_cdeg;
    int16_t roll_cdeg;
    int16_t pitch_cdeg;
    uint8_t flow_quality;
    uint8_t reserved0;
    uint16_t crc16;
    uint16_t reserved1;
} BlackboxPositionRecord_t;

typedef struct
{
    uint32_t magic;
    uint8_t type;
    uint8_t schema;
    uint16_t session_id;
    uint32_t t_ms;
    uint16_t sample_count;
    uint16_t reserved0;
    uint32_t utc_seconds;
    uint8_t reserved1[40];
    uint16_t crc16;
    uint16_t reserved2;
} BlackboxSessionEndRecord_t;
#pragma pack(pop)

typedef union
{
    uint8_t bytes[BLACKBOX_RECORD_SIZE];
    BlackboxSessionBeginRecord_t begin;
    BlackboxAltitudeParamsRecord_t altitude;
    BlackboxSampleRecord_t sample;
    BlackboxPositionRecord_t position;
    BlackboxSessionEndRecord_t end;
} BlackboxFlashRecord_t;

typedef struct
{
    bool valid;
    uint32_t generation;
    int32_t values_q10000[APEX_PID_PARAMETER_COUNT];
} BlackboxParameterCandidate_t;

#pragma pack(push, 1)
typedef struct
{
    uint32_t magic;
    uint16_t schema;
    uint16_t reserved0;
    uint32_t generation;
    int32_t offset_mx_q10000;
    int32_t offset_my_q10000;
    int32_t offset_mz_q10000;
    int32_t span_mx_q10000;
    int32_t span_my_q10000;
    int32_t span_mz_q10000;
    uint32_t sample_count;
    uint8_t reserved1[20];
    uint16_t crc16;
    uint16_t reserved2;
} BlackboxMagCalRecord_t;
#pragma pack(pop)

typedef struct
{
    bool valid;
    uint32_t generation;
    MagCalibrationStatus_t status;
} BlackboxMagCalCandidate_t;

typedef enum
{
    BLACKBOX_JOB_NONE = 0,
    BLACKBOX_JOB_SAMPLE,
    BLACKBOX_JOB_POSITION_SAMPLE,
    BLACKBOX_JOB_SAVE,
    BLACKBOX_JOB_SAVE_MAG_CAL,
    BLACKBOX_JOB_ARM,
    BLACKBOX_JOB_DISARM
} BlackboxJobType_t;

typedef struct
{
    BlackboxJobType_t type;
    BlackboxSample_t sample;
    BlackboxPositionSample_t position;
} BlackboxJob_t;

static W25Q64_Device_t blackbox_flash;
static QueueHandle_t blackbox_queue;
static StaticQueue_t blackbox_queue_storage;
static uint8_t blackbox_queue_buffer[BLACKBOX_SAMPLE_QUEUE_LENGTH * sizeof(BlackboxJob_t)];
static volatile bool blackbox_ready;
static volatile bool blackbox_recording;
static volatile bool blackbox_save_pending;
static volatile bool blackbox_mag_cal_save_pending;
static volatile bool blackbox_has_saved;
static bool blackbox_has_saved_mag_cal;
static uint32_t blackbox_mag_cal_generation;
static MagCalibrationStatus_t blackbox_saved_mag_cal;
static volatile uint32_t blackbox_dropped_samples;
static uint32_t blackbox_log_write_address;
static uint32_t blackbox_parameter_generation;
static uint32_t blackbox_log_index_generation;
static bool blackbox_log_wrapped;
static uint16_t blackbox_next_session_id;
static uint16_t blackbox_active_session_id;
static uint32_t blackbox_session_start_tick;
static uint16_t blackbox_session_sample_count;
static uint32_t blackbox_session_begin_utc_seconds;
static int32_t blackbox_saved_values[APEX_PID_PARAMETER_COUNT];
static char blackbox_command[BLACKBOX_COMMAND_BUFFER_SIZE];
static uint8_t blackbox_command_length;

typedef char BlackboxParameterRecordMustFit[(sizeof(BlackboxParameterRecord_t) <= W25Q64_SECTOR_SIZE_BYTES) ? 1 : -1];
typedef char BlackboxSessionBeginMustBe64[(sizeof(BlackboxSessionBeginRecord_t) == BLACKBOX_RECORD_SIZE) ? 1 : -1];
typedef char BlackboxAltitudeParamsMustBe64[(sizeof(BlackboxAltitudeParamsRecord_t) == BLACKBOX_RECORD_SIZE) ? 1 : -1];
typedef char BlackboxSampleMustBe64[(sizeof(BlackboxSampleRecord_t) == BLACKBOX_RECORD_SIZE) ? 1 : -1];
typedef char BlackboxPositionMustBe64[(sizeof(BlackboxPositionRecord_t) == BLACKBOX_RECORD_SIZE) ? 1 : -1];
typedef char BlackboxSessionEndMustBe64[(sizeof(BlackboxSessionEndRecord_t) == BLACKBOX_RECORD_SIZE) ? 1 : -1];
typedef char BlackboxFlashRecordMustBe64[(sizeof(BlackboxFlashRecord_t) == BLACKBOX_RECORD_SIZE) ? 1 : -1];
typedef char BlackboxLogIndexMustFit[(sizeof(BlackboxLogIndexRecord_t) <= BLACKBOX_LOG_INDEX_RECORD_SIZE) ? 1 : -1];
typedef char BlackboxMagCalMustBe64[(sizeof(BlackboxMagCalRecord_t) == BLACKBOX_MAG_CAL_RECORD_SIZE) ? 1 : -1];

static volatile bool blackbox_abort_dump;
static bool blackbox_dumping;

static void Blackbox_PollUartForStop(void);
static bool Blackbox_WriteLogIndex(void);

static int16_t Blackbox_ClampI16(int32_t value)
{
    if (value > 32767) return 32767;
    if (value < -32768) return (int16_t)-32768;
    return (int16_t)value;
}

static int16_t Blackbox_Q10000ToQ100(int32_t value_q10000)
{
    if (value_q10000 >= 0) return Blackbox_ClampI16((value_q10000 + 50) / 100);
    return Blackbox_ClampI16((value_q10000 - 50) / 100);
}

static uint16_t Blackbox_ReadU16(const uint8_t *bytes, uint8_t offset)
{
    return (uint16_t)bytes[offset] | ((uint16_t)bytes[offset + 1U] << 8U);
}

static uint32_t Blackbox_ReadU32(const uint8_t *bytes, uint8_t offset)
{
    uint32_t value = (uint32_t)bytes[offset];
    value |= (uint32_t)bytes[offset + 1U] << 8U;
    value |= (uint32_t)bytes[offset + 2U] << 16U;
    value |= (uint32_t)bytes[offset + 3U] << 24U;
    return value;
}

static int16_t Blackbox_ReadI16(const uint8_t *bytes, uint8_t offset)
{
    return (int16_t)Blackbox_ReadU16(bytes, offset);
}

static void Blackbox_FormatBeijingTime(uint32_t utc_seconds, char *text, uint16_t size)
{
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;

    if ((text == NULL) || (size == 0U)) return;
    if (utc_seconds == 0U)
    {
        (void)snprintf(text, size, "NA");
        return;
    }
    GPS_UnixSecondsToCalendar(utc_seconds + 8UL * 3600UL, &year, &month, &day, &hour, &minute, &second);
    (void)snprintf(text, size, "%04u-%02u-%02uT%02u:%02u:%02u", year, month, day, hour, minute, second);
}

static void Blackbox_FinalizeRecord(uint8_t *bytes)
{
    uint16_t crc = ApexProtocol_Crc16(bytes, BLACKBOX_RECORD_CRC_OFFSET);
    bytes[BLACKBOX_RECORD_CRC_OFFSET] = (uint8_t)crc;
    bytes[BLACKBOX_RECORD_CRC_OFFSET + 1U] = (uint8_t)(crc >> 8U);
}

static bool Blackbox_RecordValid(const uint8_t *bytes)
{
    uint16_t crc;
    uint16_t stored;

    if (Blackbox_ReadU32(bytes, 0U) != BLACKBOX_RECORD_MAGIC) return false;
    crc = ApexProtocol_Crc16(bytes, BLACKBOX_RECORD_CRC_OFFSET);
    stored = Blackbox_ReadU16(bytes, BLACKBOX_RECORD_CRC_OFFSET);
    return crc == stored;
}

static bool Blackbox_QueueJob(BlackboxJobType_t type, const BlackboxSample_t *sample)
{
    BlackboxJob_t job;

    if (blackbox_queue == NULL) return false;
    memset(&job, 0, sizeof(job));
    job.type = type;
    if (sample != NULL) job.sample = *sample;
    if (xQueueSend(blackbox_queue, &job, 0) != pdTRUE)
    {
        if (type == BLACKBOX_JOB_SAMPLE) blackbox_dropped_samples++;
        return false;
    }
    return true;
}

static bool Blackbox_QueuePositionJob(const BlackboxPositionSample_t *sample)
{
    BlackboxJob_t job;

    if ((blackbox_queue == NULL) || (sample == NULL)) return false;
    memset(&job, 0, sizeof(job));
    job.type = BLACKBOX_JOB_POSITION_SAMPLE;
    job.position = *sample;
    if (xQueueSend(blackbox_queue, &job, 0) != pdTRUE)
    {
        blackbox_dropped_samples++;
        return false;
    }
    return true;
}

static bool Blackbox_SaveSlot(uint32_t address, uint32_t generation, const int32_t *values_q10000)
{
    BlackboxParameterRecord_t record;

    memset(&record, 0, sizeof(record));
    record.magic = BLACKBOX_PARAMETER_MAGIC;
    record.schema = BLACKBOX_PARAMETER_SCHEMA;
    record.parameter_count = APEX_PID_PARAMETER_COUNT;
    record.generation = generation;
    memcpy(record.values_q10000, values_q10000, sizeof(record.values_q10000));
    record.crc16 = ApexProtocol_Crc16((const uint8_t *)&record, (uint16_t)(sizeof(record) - 4U));
    if (W25Q64_EraseSector(&blackbox_flash, address) != W25Q64_STATUS_OK) return false;
    return W25Q64_Write(&blackbox_flash, address, (const uint8_t *)&record, sizeof(record)) == W25Q64_STATUS_OK;
}

static bool Blackbox_ReadSlot(uint32_t address, BlackboxParameterCandidate_t *candidate)
{
    BlackboxParameterRecord_t record;
    BlackboxLegacyV7ParameterRecord_t legacy_v7;
    BlackboxLegacyParameterRecord_t legacy;
    uint16_t crc;

    memset(candidate, 0, sizeof(*candidate));
    memset(&record, 0, sizeof(record));
    if (W25Q64_Read(&blackbox_flash, address, (uint8_t *)&record, sizeof(record)) != W25Q64_STATUS_OK) return false;
    if (record.magic != BLACKBOX_PARAMETER_MAGIC) return false;

    if ((record.schema == BLACKBOX_PARAMETER_SCHEMA) && (record.parameter_count == APEX_PID_PARAMETER_COUNT))
    {
        crc = ApexProtocol_Crc16((const uint8_t *)&record, (uint16_t)(sizeof(record) - 4U));
        if (crc != record.crc16) return false;
        candidate->generation = record.generation;
        memcpy(candidate->values_q10000, record.values_q10000, sizeof(candidate->values_q10000));
    }
    else if ((record.schema == BLACKBOX_PARAMETER_V7_SCHEMA) && (record.parameter_count == APEX_PID_V7_PARAMETER_COUNT))
    {
        /* 上一版 30 项记录（加高度环之前存的）。CRC 在原结构尾部，按原长度校验后
           把 30 项搬回来，新增的高度环三项保留 PidTable_ExportSnapshot 出来的当前默认值。
           ⚠️ 顺序不能颠倒：先 ExportSnapshot 填默认，再 memcpy 覆盖前 30 项。 */
        memcpy(&legacy_v7, &record, sizeof(legacy_v7));
        crc = ApexProtocol_Crc16((const uint8_t *)&legacy_v7, (uint16_t)(sizeof(legacy_v7) - 4U));
        if (crc != legacy_v7.crc16) return false;
        PidTable_ExportSnapshot(candidate->values_q10000);
        memcpy(candidate->values_q10000, legacy_v7.values_q10000, sizeof(legacy_v7.values_q10000));
        candidate->generation = legacy_v7.generation;
    }
    else if ((record.schema == BLACKBOX_PARAMETER_LEGACY_SCHEMA) && (record.parameter_count == APEX_PID_V6_PARAMETER_COUNT))
    {
        /* 旧 26 项记录的 CRC 在旧结构尾部，按旧长度校验后再补入四项高度默认值。 */
        memcpy(&legacy, &record, sizeof(legacy));
        crc = ApexProtocol_Crc16((const uint8_t *)&legacy, (uint16_t)(sizeof(legacy) - 4U));
        if (crc != legacy.crc16) return false;
        PidTable_ExportSnapshot(candidate->values_q10000);
        memcpy(candidate->values_q10000, legacy.values_q10000, sizeof(legacy.values_q10000));
        candidate->generation = legacy.generation;
    }
    else return false;

    candidate->valid = true;
    return true;
}

static int32_t Blackbox_FloatToQ10000(float value)
{
    if (value >= 0.0f) return (int32_t)(value * (float)APEX_PID_VALUE_SCALE + 0.5f);
    return (int32_t)(value * (float)APEX_PID_VALUE_SCALE - 0.5f);
}

static float Blackbox_Q10000ToFloat(int32_t value_q10000)
{
    return (float)value_q10000 / (float)APEX_PID_VALUE_SCALE;
}

static bool Blackbox_WriteMagCalSlot(uint32_t offset, uint32_t generation, const MagCalibrationStatus_t *status)
{
    BlackboxMagCalRecord_t record;

    memset(&record, 0, sizeof(record));
    record.magic = BLACKBOX_MAG_CAL_MAGIC;
    record.schema = BLACKBOX_MAG_CAL_SCHEMA;
    record.generation = generation;
    record.offset_mx_q10000 = Blackbox_FloatToQ10000(status->offset_mx_gauss);
    record.offset_my_q10000 = Blackbox_FloatToQ10000(status->offset_my_gauss);
    record.offset_mz_q10000 = Blackbox_FloatToQ10000(status->offset_mz_gauss);
    record.span_mx_q10000 = Blackbox_FloatToQ10000(status->span_mx_gauss);
    record.span_my_q10000 = Blackbox_FloatToQ10000(status->span_my_gauss);
    record.span_mz_q10000 = Blackbox_FloatToQ10000(status->span_mz_gauss);
    record.sample_count = status->sample_count;
    record.crc16 = ApexProtocol_Crc16((const uint8_t *)&record, (uint16_t)(sizeof(record) - 4U));
    return W25Q64_Write(&blackbox_flash, BLACKBOX_MAG_CAL_SECTOR + offset, (const uint8_t *)&record, sizeof(record)) == W25Q64_STATUS_OK;
}

static bool Blackbox_ReadMagCalSlot(uint32_t offset, BlackboxMagCalCandidate_t *candidate)
{
    BlackboxMagCalRecord_t record;
    uint16_t crc;

    memset(candidate, 0, sizeof(*candidate));
    if (W25Q64_Read(&blackbox_flash, BLACKBOX_MAG_CAL_SECTOR + offset, (uint8_t *)&record, sizeof(record)) != W25Q64_STATUS_OK) return false;
    if ((record.magic != BLACKBOX_MAG_CAL_MAGIC) || (record.schema != BLACKBOX_MAG_CAL_SCHEMA)) return false;
    crc = ApexProtocol_Crc16((const uint8_t *)&record, (uint16_t)(sizeof(record) - 4U));
    if (crc != record.crc16) return false;
    candidate->generation = record.generation;
    candidate->status.valid = true;
    candidate->status.quality_ok = true;
    candidate->status.sample_count = record.sample_count;
    candidate->status.offset_mx_gauss = Blackbox_Q10000ToFloat(record.offset_mx_q10000);
    candidate->status.offset_my_gauss = Blackbox_Q10000ToFloat(record.offset_my_q10000);
    candidate->status.offset_mz_gauss = Blackbox_Q10000ToFloat(record.offset_mz_q10000);
    candidate->status.span_mx_gauss = Blackbox_Q10000ToFloat(record.span_mx_q10000);
    candidate->status.span_my_gauss = Blackbox_Q10000ToFloat(record.span_my_q10000);
    candidate->status.span_mz_gauss = Blackbox_Q10000ToFloat(record.span_mz_q10000);
    candidate->valid = true;
    return true;
}

static bool Blackbox_ApplyBestMagCalSlot(void)
{
    BlackboxMagCalCandidate_t slot_a;
    BlackboxMagCalCandidate_t slot_b;
    const BlackboxMagCalCandidate_t *best = NULL;

    (void)Blackbox_ReadMagCalSlot(BLACKBOX_MAG_CAL_SLOT_A_OFFSET, &slot_a);
    (void)Blackbox_ReadMagCalSlot(BLACKBOX_MAG_CAL_SLOT_B_OFFSET, &slot_b);
    if (slot_a.valid && slot_b.valid) best = (slot_a.generation >= slot_b.generation) ? &slot_a : &slot_b;
    else if (slot_a.valid) best = &slot_a;
    else if (slot_b.valid) best = &slot_b;
    if (best == NULL) return false;
    blackbox_saved_mag_cal = best->status;
    blackbox_mag_cal_generation = best->generation;
    blackbox_has_saved_mag_cal = true;
    (void)Mag_CalibrationApplySaved(&best->status);
    return true;
}

static bool Blackbox_SaveMagCalibration(const MagCalibrationStatus_t *status)
{
    uint32_t generation;
    bool ok;

    if ((status == NULL) || !status->valid) return false;
    generation = blackbox_mag_cal_generation + 1U;
    if (W25Q64_EraseSector(&blackbox_flash, BLACKBOX_MAG_CAL_SECTOR) != W25Q64_STATUS_OK) return false;
    ok = Blackbox_WriteMagCalSlot(BLACKBOX_MAG_CAL_SLOT_A_OFFSET, generation, status);
    ok = Blackbox_WriteMagCalSlot(BLACKBOX_MAG_CAL_SLOT_B_OFFSET, generation, status) && ok;
    if (ok)
    {
        blackbox_mag_cal_generation = generation;
        blackbox_saved_mag_cal = *status;
        blackbox_has_saved_mag_cal = true;
    }
    return ok;
}

static void Blackbox_HandleMagCalSave(void)
{
    MagCalibrationStatus_t status;

    Mag_CalibrationGetStatus(&status);
    if (!status.valid)
    {
        Debug_Printf("[MAG_CAL] flash skip, RAM offset is not valid.\r\n");
        blackbox_mag_cal_save_pending = false;
        return;
    }
    if (Blackbox_SaveMagCalibration(&status)) Debug_Printf("[MAG_CAL] flash saved gen=%lu offset=%.3f,%.3f,%.3f.\r\n", (unsigned long)blackbox_mag_cal_generation, (double)status.offset_mx_gauss, (double)status.offset_my_gauss, (double)status.offset_mz_gauss);
    else Debug_Printf("[MAG_CAL] flash save failed, previous copy kept if present.\r\n");
    blackbox_mag_cal_save_pending = false;
}

static bool Blackbox_QueueMagCalSave(void)
{
    if (!blackbox_ready) return false;
    if (blackbox_recording) return false;
    if (blackbox_mag_cal_save_pending) return false;
    blackbox_mag_cal_save_pending = true;
    if (!Blackbox_QueueJob(BLACKBOX_JOB_SAVE_MAG_CAL, NULL))
    {
        blackbox_mag_cal_save_pending = false;
        return false;
    }
    return true;
}

static void Blackbox_RememberSaved(const int32_t *values_q10000, uint32_t generation)
{
    memcpy(blackbox_saved_values, values_q10000, sizeof(blackbox_saved_values));
    blackbox_parameter_generation = generation;
    blackbox_has_saved = true;
}

static bool Blackbox_ApplyBestSlot(void)
{
    BlackboxParameterCandidate_t slot_a;
    BlackboxParameterCandidate_t slot_b;
    const BlackboxParameterCandidate_t *best = NULL;

    (void)Blackbox_ReadSlot(BLACKBOX_PARAMETER_SECTOR_A, &slot_a);
    (void)Blackbox_ReadSlot(BLACKBOX_PARAMETER_SECTOR_B, &slot_b);
    if (slot_a.valid && slot_b.valid) best = (slot_a.generation >= slot_b.generation) ? &slot_a : &slot_b;
    else if (slot_a.valid) best = &slot_a;
    else if (slot_b.valid) best = &slot_b;
    if (best == NULL) return false;
    if (!PidTable_ImportSnapshot(best->values_q10000)) return false;
    Blackbox_RememberSaved(best->values_q10000, best->generation);
    return true;
}

static uint32_t Blackbox_LogCapacityRecords(void)
{
    return (BLACKBOX_LOG_END_ADDRESS - BLACKBOX_LOG_START_ADDRESS) / BLACKBOX_RECORD_SIZE;
}

static uint32_t Blackbox_NormalizedWriteAddress(void)
{
    if (blackbox_log_write_address >= BLACKBOX_LOG_END_ADDRESS) return BLACKBOX_LOG_START_ADDRESS;
    if (blackbox_log_write_address < BLACKBOX_LOG_START_ADDRESS) return BLACKBOX_LOG_START_ADDRESS;
    return blackbox_log_write_address;
}

static uint32_t Blackbox_LogUsedRecords(void)
{
    uint32_t write_address = Blackbox_NormalizedWriteAddress();
    if (blackbox_log_wrapped) return Blackbox_LogCapacityRecords();
    return (write_address - BLACKBOX_LOG_START_ADDRESS) / BLACKBOX_RECORD_SIZE;
}

static uint32_t Blackbox_AdvanceLogAddress(uint32_t address, uint32_t record_count)
{
    uint32_t capacity_bytes = BLACKBOX_LOG_END_ADDRESS - BLACKBOX_LOG_START_ADDRESS;
    uint32_t offset = (address - BLACKBOX_LOG_START_ADDRESS) + (record_count * BLACKBOX_RECORD_SIZE);
    return BLACKBOX_LOG_START_ADDRESS + (offset % capacity_bytes);
}

static uint32_t Blackbox_RewindLogAddress(uint32_t address, uint32_t record_count)
{
    uint32_t capacity_bytes = BLACKBOX_LOG_END_ADDRESS - BLACKBOX_LOG_START_ADDRESS;
    uint32_t offset = address - BLACKBOX_LOG_START_ADDRESS;
    uint32_t rewind_bytes = record_count * BLACKBOX_RECORD_SIZE;
    rewind_bytes %= capacity_bytes;
    if (offset >= rewind_bytes) return BLACKBOX_LOG_START_ADDRESS + (offset - rewind_bytes);
    return BLACKBOX_LOG_START_ADDRESS + (capacity_bytes - (rewind_bytes - offset));
}

static bool Blackbox_WriteLogIndex(void)
{
    BlackboxLogIndexRecord_t record;
    uint8_t bytes[BLACKBOX_LOG_INDEX_RECORD_SIZE];

    memset(&record, 0, sizeof(record));
    memset(bytes, 0xFF, sizeof(bytes));
    record.magic = BLACKBOX_LOG_INDEX_MAGIC;
    record.write_address = blackbox_log_write_address;
    record.next_session_id = blackbox_next_session_id;
    record.wrapped = blackbox_log_wrapped ? 1U : 0U;
    record.generation = blackbox_log_index_generation + 1U;
    memcpy(bytes, &record, sizeof(record));
    {
        uint16_t crc = ApexProtocol_Crc16(bytes, 16U);
        bytes[16] = (uint8_t)crc;
        bytes[17] = (uint8_t)(crc >> 8U);
    }
    if (W25Q64_EraseSector(&blackbox_flash, BLACKBOX_LOG_INDEX_ADDRESS) != W25Q64_STATUS_OK) return false;
    if (W25Q64_Write(&blackbox_flash, BLACKBOX_LOG_INDEX_ADDRESS, bytes, sizeof(bytes)) != W25Q64_STATUS_OK) return false;
    blackbox_log_index_generation = record.generation;
    return true;
}

static bool Blackbox_LoadLogIndex(void)
{
    uint8_t bytes[BLACKBOX_LOG_INDEX_RECORD_SIZE];
    uint32_t magic;
    uint32_t write_address;
    uint16_t next_session;
    uint16_t wrapped;
    uint32_t generation;
    uint16_t crc;
    uint16_t stored;

    if (W25Q64_Read(&blackbox_flash, BLACKBOX_LOG_INDEX_ADDRESS, bytes, sizeof(bytes)) != W25Q64_STATUS_OK) return false;
    magic = Blackbox_ReadU32(bytes, 0U);
    if (magic != BLACKBOX_LOG_INDEX_MAGIC) return false;
    crc = ApexProtocol_Crc16(bytes, 16U);
    stored = Blackbox_ReadU16(bytes, 16U);
    if (crc != stored) return false;
    write_address = Blackbox_ReadU32(bytes, 4U);
    next_session = Blackbox_ReadU16(bytes, 8U);
    wrapped = Blackbox_ReadU16(bytes, 10U);
    generation = Blackbox_ReadU32(bytes, 12U);
    if ((write_address < BLACKBOX_LOG_START_ADDRESS) || (write_address > BLACKBOX_LOG_END_ADDRESS)) return false;
    if ((write_address % BLACKBOX_RECORD_SIZE) != 0U) return false;
    blackbox_log_write_address = write_address;
    blackbox_next_session_id = (next_session == 0U) ? 1U : next_session;
    blackbox_log_wrapped = (wrapped != 0U);
    blackbox_log_index_generation = generation;
    return true;
}

static void Blackbox_RecoverWritePointer(void)
{
    uint8_t magic_bytes[8];
    uint32_t address = Blackbox_NormalizedWriteAddress();
    uint32_t sector_end;
    uint32_t limit;
    uint32_t scanned;

    /* 环写后只在当前扇区里补扫：后面要么是刚擦过的 0xFF，要么是上一圈旧数据，不能整圈盲扫。 */
    if (blackbox_log_wrapped)
    {
        sector_end = (address & ~(W25Q64_SECTOR_SIZE_BYTES - 1UL)) + W25Q64_SECTOR_SIZE_BYTES;
        if (sector_end > BLACKBOX_LOG_END_ADDRESS) sector_end = BLACKBOX_LOG_END_ADDRESS;
        if (sector_end <= address) return;
        limit = (sector_end - address) / BLACKBOX_RECORD_SIZE;
    }
    else
    {
        limit = (BLACKBOX_LOG_END_ADDRESS - address) / BLACKBOX_RECORD_SIZE;
    }

    for (scanned = 0U; scanned < limit; scanned++)
    {
        if (W25Q64_Read(&blackbox_flash, address, magic_bytes, 8U) != W25Q64_STATUS_OK) return;
        if (Blackbox_ReadU32(magic_bytes, 0U) == 0xFFFFFFFFUL) break;
        if (Blackbox_ReadU32(magic_bytes, 0U) != BLACKBOX_RECORD_MAGIC) break;
        if (magic_bytes[4] == BLACKBOX_RECORD_TYPE_SESSION_BEGIN)
        {
            uint16_t session_id = Blackbox_ReadU16(magic_bytes, 6U);
            if ((session_id + 1U) > blackbox_next_session_id)
            {
                blackbox_next_session_id = (uint16_t)(session_id + 1U);
                if (blackbox_next_session_id == 0U) blackbox_next_session_id = 1U;
            }
        }
        address += BLACKBOX_RECORD_SIZE;
        if (address >= BLACKBOX_LOG_END_ADDRESS)
        {
            address = BLACKBOX_LOG_START_ADDRESS;
            blackbox_log_wrapped = true;
            break;
        }
        if ((scanned & 0x0FU) == 0U) vTaskDelay(1);
    }
    blackbox_log_write_address = address;
}

static bool Blackbox_ScanLogEnd(void)
{
    uint8_t magic_bytes[8];
    uint32_t address;
    uint32_t magic;
    uint16_t max_session = 0U;
    uint32_t used_records = 0U;

    blackbox_log_write_address = BLACKBOX_LOG_START_ADDRESS;
    blackbox_log_wrapped = false;
    for (address = BLACKBOX_LOG_START_ADDRESS; address + BLACKBOX_RECORD_SIZE <= BLACKBOX_LOG_END_ADDRESS; address += BLACKBOX_RECORD_SIZE)
    {
        /* 读 magic + type + schema + session_id，上电时按真实场次号续编，不再用记录数除 64 估。 */
        if (W25Q64_Read(&blackbox_flash, address, magic_bytes, 8U) != W25Q64_STATUS_OK) return false;
        magic = Blackbox_ReadU32(magic_bytes, 0U);
        if (magic == 0xFFFFFFFFUL)
        {
            blackbox_log_write_address = address;
            blackbox_next_session_id = (uint16_t)(max_session + 1U);
            if (blackbox_next_session_id == 0U) blackbox_next_session_id = 1U;
            return true;
        }
        if (magic != BLACKBOX_RECORD_MAGIC)
        {
            /* 老固件或损坏数据不当新日志，从起点覆盖，避免扫整片 8MB。 */
            blackbox_log_write_address = BLACKBOX_LOG_START_ADDRESS;
            blackbox_next_session_id = 1U;
            blackbox_log_wrapped = false;
            return true;
        }
        used_records++;
        if (magic_bytes[4] == BLACKBOX_RECORD_TYPE_SESSION_BEGIN)
        {
            uint16_t session_id = Blackbox_ReadU16(magic_bytes, 6U);
            if (session_id > max_session) max_session = session_id;
        }
        if ((used_records & 0x0FU) == 0U) vTaskDelay(1);
    }
    /* 日志区已满且还没有索引：写指针停在末尾，下一次写入再绕回起点。 */
    blackbox_log_write_address = BLACKBOX_LOG_END_ADDRESS;
    blackbox_log_wrapped = true;
    blackbox_next_session_id = (uint16_t)(max_session + 1U);
    if (blackbox_next_session_id == 0U) blackbox_next_session_id = 1U;
    return true;
}

static bool Blackbox_AppendRecord(uint8_t *bytes)
{
    Blackbox_FinalizeRecord(bytes);
    if (blackbox_log_write_address >= BLACKBOX_LOG_END_ADDRESS)
    {
        blackbox_log_write_address = BLACKBOX_LOG_START_ADDRESS;
        blackbox_log_wrapped = true;
    }
    if ((blackbox_log_write_address % W25Q64_SECTOR_SIZE_BYTES) == 0U)
    {
        if (W25Q64_EraseSector(&blackbox_flash, blackbox_log_write_address) != W25Q64_STATUS_OK) return false;
        /* 擦完当前扇区立刻记下写指针。掉电后只在本扇区内向前补扫到 0xFF。 */
        (void)Blackbox_WriteLogIndex();
    }
    if (W25Q64_Write(&blackbox_flash, blackbox_log_write_address, bytes, BLACKBOX_RECORD_SIZE) != W25Q64_STATUS_OK) return false;
    blackbox_log_write_address += BLACKBOX_RECORD_SIZE;
    if (blackbox_log_write_address >= BLACKBOX_LOG_END_ADDRESS)
    {
        blackbox_log_write_address = BLACKBOX_LOG_START_ADDRESS;
        blackbox_log_wrapped = true;
    }
    return true;
}

static bool Blackbox_WriteSessionBegin(void)
{
    BlackboxFlashRecord_t record;
    int32_t values[APEX_PID_PARAMETER_COUNT];
    uint8_t index;

    memset(&record, 0, sizeof(record));
    PidTable_ExportSnapshot(values);
    record.begin.magic = BLACKBOX_RECORD_MAGIC;
    record.begin.type = BLACKBOX_RECORD_TYPE_SESSION_BEGIN;
    record.begin.schema = BLACKBOX_RECORD_SCHEMA;
    record.begin.session_id = blackbox_active_session_id;
    for (index = 0U; index < BLACKBOX_SESSION_PID_PARAMETER_COUNT; index++) record.begin.pid_q100[index] = Blackbox_Q10000ToQ100(values[index]);
    if (!Blackbox_AppendRecord(record.bytes)) return false;

    memset(&record, 0, sizeof(record));
    record.altitude.magic = BLACKBOX_RECORD_MAGIC;
    record.altitude.type = BLACKBOX_RECORD_TYPE_ALTITUDE_PARAMS;
    record.altitude.schema = BLACKBOX_RECORD_SCHEMA;
    record.altitude.session_id = blackbox_active_session_id;
    for (index = 0U; index < 4U; index++) record.altitude.altitude_q10000[index] = values[APEX_PID_VERTICAL_SPEED_KP + index];
    /* 高度环（外环）三增益。与上面那组分开写，因为它们在协议表里不连续
       （内环在 26~28、满杆在 29、外环在 30~32），用 + index 是安全的。 */
    for (index = 0U; index < 3U; index++) record.altitude.height_q10000[index] = values[APEX_PID_HEIGHT_KP + index];
    if (!GPS_GetUtcTime(&record.altitude.utc_seconds)) record.altitude.utc_seconds = 0U;
    blackbox_session_begin_utc_seconds = record.altitude.utc_seconds;
    return Blackbox_AppendRecord(record.bytes);
}

static bool Blackbox_WriteSessionEnd(void)
{
    BlackboxFlashRecord_t record;

    memset(&record, 0, sizeof(record));
    record.end.magic = BLACKBOX_RECORD_MAGIC;
    record.end.type = BLACKBOX_RECORD_TYPE_SESSION_END;
    record.end.schema = BLACKBOX_RECORD_SCHEMA;
    record.end.session_id = blackbox_active_session_id;
    record.end.t_ms = (uint32_t)(xTaskGetTickCount() - blackbox_session_start_tick);
    record.end.sample_count = blackbox_session_sample_count;
    if (!GPS_GetUtcTime(&record.end.utc_seconds)) record.end.utc_seconds = 0U;
    return Blackbox_AppendRecord(record.bytes);
}

static bool Blackbox_WriteSample(const BlackboxSample_t *sample)
{
    BlackboxFlashRecord_t record;

    memset(&record, 0, sizeof(record));
    record.sample.magic = BLACKBOX_RECORD_MAGIC;
    record.sample.type = BLACKBOX_RECORD_TYPE_SAMPLE;
    record.sample.schema = BLACKBOX_RECORD_SCHEMA;
    record.sample.session_id = blackbox_active_session_id;
    record.sample.t_ms = (uint32_t)(xTaskGetTickCount() - blackbox_session_start_tick);
    record.sample.roll_cdeg = sample->roll_cdeg;
    record.sample.pitch_cdeg = sample->pitch_cdeg;
    record.sample.yaw_cdeg = sample->yaw_cdeg;
    record.sample.roll_rate_cdps = sample->roll_rate_cdps;
    record.sample.pitch_rate_cdps = sample->pitch_rate_cdps;
    record.sample.yaw_rate_cdps = sample->yaw_rate_cdps;
    record.sample.roll_corr_us = sample->roll_corr_us;
    record.sample.pitch_corr_us = sample->pitch_corr_us;
    record.sample.yaw_corr_us = sample->yaw_corr_us;
    record.sample.throttle_us = sample->throttle_us;
    memcpy(record.sample.motor_us, sample->motor_us, sizeof(record.sample.motor_us));
    record.sample.altitude_cm = sample->altitude_cm;
    record.sample.vz_cms = sample->vz_cms;
    record.sample.mag_heading_cdeg = sample->mag_heading_cdeg;
    record.sample.vz_correction_us = sample->vz_correction_us;
    record.sample.vz_target_cms = sample->vz_target_cms;
    record.sample.altitude_state = sample->altitude_state;
    record.sample.armed = sample->armed;
    record.sample.connected = sample->connected;
    if (!Blackbox_AppendRecord(record.bytes)) return false;
    if (blackbox_session_sample_count < 0xFFFFU) blackbox_session_sample_count++;
    return true;
}

static bool Blackbox_WritePositionSample(const BlackboxPositionSample_t *sample)
{
    BlackboxFlashRecord_t record;
    uint16_t status;

    memset(&record, 0, sizeof(record));
    record.position.magic = BLACKBOX_RECORD_MAGIC;
    record.position.type = BLACKBOX_RECORD_TYPE_POSITION_SAMPLE;
    record.position.schema = BLACKBOX_RECORD_SCHEMA;
    record.position.session_id = blackbox_active_session_id;
    record.position.t_ms = (uint32_t)(sample->source_tick - blackbox_session_start_tick);
    record.position.flow_dx = sample->flow_dx;
    record.position.flow_dy = sample->flow_dy;
    status = (uint16_t)(sample->state & 0x07U);
    status |= (uint16_t)((sample->flow_valid ? 1U : 0U) << 3U);
    status |= (uint16_t)((sample->gate_mask & 0x1FU) << 4U);
    status |= (uint16_t)((sample->fault_reason & 0x07U) << 9U);
    record.position.status = status;
    record.position.flow_sum_x_q10 = sample->flow_sum_x_q10;
    record.position.flow_sum_y_q10 = sample->flow_sum_y_q10;
    record.position.flow_zero_x_q1000 = sample->flow_zero_x_q1000;
    record.position.flow_zero_y_q1000 = sample->flow_zero_y_q1000;
    record.position.velocity_forward_q10 = sample->velocity_forward_q10;
    record.position.velocity_right_q10 = sample->velocity_right_q10;
    record.position.velocity_forward_sp_q10 = sample->velocity_forward_sp_q10;
    record.position.velocity_right_sp_q10 = sample->velocity_right_sp_q10;
    record.position.raw_roll_cdeg = sample->raw_roll_cdeg;
    record.position.raw_pitch_cdeg = sample->raw_pitch_cdeg;
    record.position.roll_cdeg = sample->roll_cdeg;
    record.position.pitch_cdeg = sample->pitch_cdeg;
    record.position.flow_quality = sample->flow_quality;
    return Blackbox_AppendRecord(record.bytes);
}

static void Blackbox_HandleSave(void)
{
    int32_t values[APEX_PID_PARAMETER_COUNT];
    uint32_t generation;
    bool ok;

    PidTable_ExportSnapshot(values);
    generation = blackbox_parameter_generation + 1U;
    ok = Blackbox_SaveSlot(BLACKBOX_PARAMETER_SECTOR_A, generation, values);
    ok = Blackbox_SaveSlot(BLACKBOX_PARAMETER_SECTOR_B, generation, values) && ok;
    if (ok)
    {
        Blackbox_RememberSaved(values, generation);
        Debug_Printf("[BB] save gen=%lu ok.\r\n", (unsigned long)generation);
    }
    else Debug_Printf("[BB] save failed.\r\n");
    blackbox_save_pending = false;
}

static void Blackbox_HandleArm(void)
{
    if (blackbox_recording)
    {
        (void)Blackbox_WriteSessionEnd();
        (void)Blackbox_WriteLogIndex();
    }
    blackbox_active_session_id = blackbox_next_session_id;
    if (blackbox_next_session_id < 0xFFFFU) blackbox_next_session_id++;
    blackbox_session_start_tick = xTaskGetTickCount();
    blackbox_session_sample_count = 0U;
    blackbox_session_begin_utc_seconds = 0U;
    if (!Blackbox_WriteSessionBegin())
    {
        blackbox_recording = false;
        Debug_Printf("[BB] session begin failed.\r\n");
        return;
    }
    blackbox_recording = true;
    if (blackbox_session_begin_utc_seconds == 0U) Debug_Printf("[BB] session %u begin utc=NA.\r\n", (unsigned int)blackbox_active_session_id);
    else
    {
        char beijing_time[24];
        Blackbox_FormatBeijingTime(blackbox_session_begin_utc_seconds, beijing_time, sizeof(beijing_time));
        Debug_Printf("[BB] session %u begin utc=%lu bjt=%s.\r\n", (unsigned int)blackbox_active_session_id, (unsigned long)blackbox_session_begin_utc_seconds, beijing_time);
    }
}

static void Blackbox_HandleDisarm(void)
{
    uint32_t utc_seconds = 0U;
    char beijing_time[24];

    if (!blackbox_recording) return;
    if (!GPS_GetUtcTime(&utc_seconds)) utc_seconds = 0U;
    if (!Blackbox_WriteSessionEnd()) Debug_Printf("[BB] session end failed.\r\n");
    else
    {
        Blackbox_FormatBeijingTime(utc_seconds, beijing_time, sizeof(beijing_time));
        Debug_Printf("[BB] session %u end samples=%u bjt=%s.\r\n", (unsigned int)blackbox_active_session_id, (unsigned int)blackbox_session_sample_count, beijing_time);
    }
    (void)Blackbox_WriteLogIndex();
    blackbox_recording = false;
}

static void Blackbox_PrintPidHeader(uint16_t session_id, uint32_t utc_seconds, const int16_t *pid_q100)
{
    char line[400];
    char beijing_time[24];
    int length;
    uint8_t index;

    Blackbox_FormatBeijingTime(utc_seconds, beijing_time, sizeof(beijing_time));
    Debug_Printf("# session=%u t0_utc=%s unix=%lu\r\n", (unsigned int)session_id, beijing_time, (unsigned long)utc_seconds);
    /* 整行一次发出，避免电机任务的 [ATT] 插进 PID 数字中间。 */
    length = snprintf(line, sizeof(line), "# pid");
    for (index = 0U; (index < BLACKBOX_SESSION_PID_PARAMETER_COUNT) && (length > 0) && (length < (int)(sizeof(line) - 24)); index++)
    {
        length += snprintf(&line[length], sizeof(line) - (size_t)length, " %u=%.2f", (unsigned int)index, (double)pid_q100[index] / 100.0);
    }
    if (length > 0)
    {
        if (length + 2 < (int)sizeof(line))
        {
            line[length++] = '\r';
            line[length++] = '\n';
            line[length] = '\0';
        }
        (void)Debug_Send((const uint8_t *)line, (uint16_t)length);
    }
    Debug_Printf("t_ms,roll,pitch,yaw,roll_rate,pitch_rate,yaw_rate,roll_corr,pitch_corr,yaw_corr,thr,m1,m2,m3,m4,alt_cm,vz_cms,mag_hdg,vz_corr_us,vz_target_cms,alt_state,armed,conn\r\n");
    Debug_Printf("# position_log\r\npos_t_ms,flow_dx,flow_dy,flow_q,flow_valid,flow_sum_x,flow_sum_y,flow_zero_x,flow_zero_y,vel_f,vel_r,vel_sp_f,vel_sp_r,raw_roll,raw_pitch,roll,pitch,pc_state,pc_gate,pc_fault\r\n");
}

static void Blackbox_PrintSample(const BlackboxFlashRecord_t *record)
{
    int16_t altitude_cm = 0;
    int16_t vz_cms = 0;
    int16_t mag_heading_cdeg = 0;
    int16_t vz_correction_us = 0;
    int16_t vz_target_cms = 0;
    uint8_t altitude_state = 0U;
    uint8_t armed;
    uint8_t connected;

    if (record->sample.schema >= BLACKBOX_RECORD_SCHEMA)
    {
        altitude_cm = record->sample.altitude_cm;
        vz_cms = record->sample.vz_cms;
        mag_heading_cdeg = record->sample.mag_heading_cdeg;
        vz_correction_us = record->sample.vz_correction_us;
        vz_target_cms = record->sample.vz_target_cms;
        altitude_state = record->sample.altitude_state;
        armed = record->sample.armed;
        connected = record->sample.connected;
    }
    else if (record->bytes[44] == 1U)
    {
        /* 19 列旧样本只在解锁场次写入，armed 固定在偏移 44；最早 17 列该处是清零填充。 */
        altitude_cm = Blackbox_ReadI16(record->bytes, 40U);
        vz_cms = Blackbox_ReadI16(record->bytes, 42U);
        armed = record->bytes[44];
        connected = record->bytes[45];
    }
    else
    {
        /* 最早 17 列布局：状态位直接位于四路电机之后。 */
        armed = record->bytes[40];
        connected = record->bytes[41];
    }

    Debug_Printf("%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d,%d,%u,%u,%u,%u,%u,%d,%d,%.2f,%d,%d,%u,%u,%u\r\n", (unsigned long)record->sample.t_ms, (double)record->sample.roll_cdeg / 100.0, (double)record->sample.pitch_cdeg / 100.0, (double)record->sample.yaw_cdeg / 100.0, (double)record->sample.roll_rate_cdps / 100.0, (double)record->sample.pitch_rate_cdps / 100.0, (double)record->sample.yaw_rate_cdps / 100.0, (int)record->sample.roll_corr_us, (int)record->sample.pitch_corr_us, (int)record->sample.yaw_corr_us, (unsigned int)record->sample.throttle_us, (unsigned int)record->sample.motor_us[0], (unsigned int)record->sample.motor_us[1], (unsigned int)record->sample.motor_us[2], (unsigned int)record->sample.motor_us[3], (int)altitude_cm, (int)vz_cms, (mag_heading_cdeg < 0) ? -1.0 : ((double)mag_heading_cdeg / 100.0), (int)vz_correction_us, (int)vz_target_cms, (unsigned int)altitude_state, (unsigned int)armed, (unsigned int)connected);
}

static void Blackbox_PrintPositionSample(const BlackboxFlashRecord_t *record)
{
    uint16_t status = record->position.status;
    uint8_t state = (uint8_t)(status & 0x07U);
    uint8_t flow_valid = (uint8_t)((status >> 3U) & 0x01U);
    uint8_t gate_mask = (uint8_t)((status >> 4U) & 0x1FU);
    uint8_t fault_reason = (uint8_t)((status >> 9U) & 0x07U);

    Debug_Printf("%lu,%d,%d,%u,%u,%.1f,%.1f,%.4f,%.4f,%.1f,%.1f,%.1f,%.1f,%.2f,%.2f,%.2f,%.2f,%u,0x%02X,%u\r\n",
                 (unsigned long)record->position.t_ms,
                 (int)record->position.flow_dx,
                 (int)record->position.flow_dy,
                 (unsigned int)record->position.flow_quality,
                 (unsigned int)flow_valid,
                 (double)record->position.flow_sum_x_q10 / 10.0,
                 (double)record->position.flow_sum_y_q10 / 10.0,
                 (double)record->position.flow_zero_x_q1000 / 1000.0,
                 (double)record->position.flow_zero_y_q1000 / 1000.0,
                 (double)record->position.velocity_forward_q10 / 10.0,
                 (double)record->position.velocity_right_q10 / 10.0,
                 (double)record->position.velocity_forward_sp_q10 / 10.0,
                 (double)record->position.velocity_right_sp_q10 / 10.0,
                 (double)record->position.raw_roll_cdeg / 100.0,
                 (double)record->position.raw_pitch_cdeg / 100.0,
                 (double)record->position.roll_cdeg / 100.0,
                 (double)record->position.pitch_cdeg / 100.0,
                 (unsigned int)state,
                 (unsigned int)gate_mask,
                 (unsigned int)fault_reason);
}

static uint32_t Blackbox_DumpStartAddress(uint32_t max_records)
{
    uint32_t used = Blackbox_LogUsedRecords();
    uint32_t write_address = Blackbox_NormalizedWriteAddress();
    uint32_t count = used;

    if (used == 0U) return write_address;
    if ((max_records > 0U) && (max_records < used)) count = max_records;
    /* 整圈有效时，写指针本身就是最老的一条。 */
    if (count == Blackbox_LogCapacityRecords()) return write_address;
    return Blackbox_RewindLogAddress(write_address, count);
}

static uint32_t Blackbox_DumpRecordCount(uint32_t max_records)
{
    uint32_t used = Blackbox_LogUsedRecords();
    if ((max_records == 0U) || (max_records >= used)) return used;
    return max_records;
}

static void Blackbox_DumpLogs(uint32_t max_records)
{
    BlackboxFlashRecord_t record;
    uint32_t address;
    uint32_t start_address;
    uint32_t total;
    uint32_t scanned = 0U;
    uint32_t samples = 0U;
    uint32_t position_samples = 0U;
    uint32_t sessions = 0U;
    uint16_t pending_session_id = 0U;
    int16_t pending_pid_q100[BLACKBOX_SESSION_PID_PARAMETER_COUNT];
    bool pending_session_header = false;
    bool in_session = false;
    bool printed_orphan_header = false;

    if (blackbox_recording)
    {
        Debug_Printf("[BB] disarm first.\r\n");
        return;
    }
    blackbox_abort_dump = false;
    blackbox_dumping = true;
    start_address = Blackbox_DumpStartAddress(max_records);
    total = Blackbox_DumpRecordCount(max_records);
    Debug_Printf("[BB] dump start records=%lu addr=0x%08lX end=0x%08lX wrap=%u.\r\n", (unsigned long)total, (unsigned long)start_address, (unsigned long)blackbox_log_write_address, blackbox_log_wrapped ? 1U : 0U);
    address = start_address;
    while (scanned < total)
    {
        Blackbox_PollUartForStop();
        if (blackbox_abort_dump)
        {
            Debug_Printf("[BB] dump abort scanned=%lu.\r\n", (unsigned long)scanned);
            blackbox_abort_dump = false;
            blackbox_dumping = false;
            return;
        }
        if (W25Q64_Read(&blackbox_flash, address, record.bytes, BLACKBOX_RECORD_SIZE) != W25Q64_STATUS_OK)
        {
            Debug_Printf("[BB] dump read fail at 0x%08lX.\r\n", (unsigned long)address);
            break;
        }
        scanned++;
        if ((scanned % 64U) == 0U) Debug_Printf("[BB] dump progress scanned=%lu total=%lu.\r\n", (unsigned long)scanned, (unsigned long)total);
        if (Blackbox_ReadU32(record.bytes, 0U) == 0xFFFFFFFFUL)
        {
            address = Blackbox_AdvanceLogAddress(address, 1U);
            continue;
        }
        if (!Blackbox_RecordValid(record.bytes))
        {
            address = Blackbox_AdvanceLogAddress(address, 1U);
            continue;
        }
        if (record.begin.type == BLACKBOX_RECORD_TYPE_SESSION_BEGIN)
        {
            pending_session_id = record.begin.session_id;
            memcpy(pending_pid_q100, record.begin.pid_q100, sizeof(pending_pid_q100));
            pending_session_header = true;
            in_session = true;
            sessions++;
        }
        else if (record.altitude.type == BLACKBOX_RECORD_TYPE_ALTITUDE_PARAMS)
        {
            if (pending_session_header)
            {
                Blackbox_PrintPidHeader(pending_session_id, record.altitude.utc_seconds, pending_pid_q100);
                pending_session_header = false;
            }
            Debug_Printf("# altitude vz_kp=%.2f vz_ki=%.2f vz_kd=%.2f max_thr=%.0f h_kp=%.2f h_ki=%.2f h_kd=%.2f utc=%lu\r\n", (double)record.altitude.altitude_q10000[0] / 10000.0, (double)record.altitude.altitude_q10000[1] / 10000.0, (double)record.altitude.altitude_q10000[2] / 10000.0, (double)record.altitude.altitude_q10000[3] / 10000.0, (double)record.altitude.height_q10000[0] / 10000.0, (double)record.altitude.height_q10000[1] / 10000.0, (double)record.altitude.height_q10000[2] / 10000.0, (unsigned long)record.altitude.utc_seconds);
        }
        else if (record.sample.type == BLACKBOX_RECORD_TYPE_SAMPLE)
        {
            if (pending_session_header)
            {
                Blackbox_PrintPidHeader(pending_session_id, 0U, pending_pid_q100);
                pending_session_header = false;
            }
            if (!in_session && !printed_orphan_header)
            {
                Debug_Printf("# session=0 t0_utc=NA unix=0\r\n");
                Debug_Printf("t_ms,roll,pitch,yaw,roll_rate,pitch_rate,yaw_rate,roll_corr,pitch_corr,yaw_corr,thr,m1,m2,m3,m4,alt_cm,vz_cms,mag_hdg,vz_corr_us,vz_target_cms,alt_state,armed,conn\r\n");
                Debug_Printf("# position_log\r\npos_t_ms,flow_dx,flow_dy,flow_q,flow_valid,flow_sum_x,flow_sum_y,flow_zero_x,flow_zero_y,vel_f,vel_r,vel_sp_f,vel_sp_r,raw_roll,raw_pitch,roll,pitch,pc_state,pc_gate,pc_fault\r\n");
                printed_orphan_header = true;
            }
            if (in_session || printed_orphan_header)
            {
                samples++;
                Blackbox_PrintSample(&record);
            }
        }
        else if (record.position.type == BLACKBOX_RECORD_TYPE_POSITION_SAMPLE)
        {
            if (pending_session_header)
            {
                Blackbox_PrintPidHeader(pending_session_id, 0U, pending_pid_q100);
                pending_session_header = false;
            }
            if (in_session || printed_orphan_header)
            {
                position_samples++;
                Blackbox_PrintPositionSample(&record);
            }
        }
        else if (record.end.type == BLACKBOX_RECORD_TYPE_SESSION_END)
        {
            char beijing_time[24];
            if (pending_session_header)
            {
                Blackbox_PrintPidHeader(pending_session_id, 0U, pending_pid_q100);
                pending_session_header = false;
            }
            Blackbox_FormatBeijingTime(record.end.utc_seconds, beijing_time, sizeof(beijing_time));
            Debug_Printf("# end session=%u samples=%u t_ms=%lu t1_utc=%s unix=%lu\r\n", (unsigned int)record.end.session_id, (unsigned int)record.end.sample_count, (unsigned long)record.end.t_ms, beijing_time, (unsigned long)record.end.utc_seconds);
            in_session = false;
        }
        address = Blackbox_AdvanceLogAddress(address, 1U);
        if ((scanned % 8U) == 0U) vTaskDelay(1);
    }
    if (pending_session_header) Blackbox_PrintPidHeader(pending_session_id, 0U, pending_pid_q100);
    Debug_Printf("[BB] dump done scanned=%lu samples=%lu position=%lu sessions=%lu.\r\n", (unsigned long)scanned, (unsigned long)samples, (unsigned long)position_samples, (unsigned long)sessions);
    blackbox_dumping = false;
}

static void Blackbox_DumpAllRaw(uint32_t max_records)
{
    uint8_t record[BLACKBOX_RECORD_SIZE];
    uint32_t address;
    uint32_t start_address;
    uint32_t total;
    uint32_t scanned = 0U;
    uint32_t offset;

    if (blackbox_recording)
    {
        Debug_Printf("[BB] disarm first.\r\n");
        return;
    }
    blackbox_abort_dump = false;
    blackbox_dumping = true;
    start_address = Blackbox_DumpStartAddress(max_records);
    total = Blackbox_DumpRecordCount(max_records);
    Debug_Printf("[BB] dumpall start records=%lu addr=0x%08lX end=0x%08lX wrap=%u.\r\n", (unsigned long)total, (unsigned long)start_address, (unsigned long)blackbox_log_write_address, blackbox_log_wrapped ? 1U : 0U);
    address = start_address;
    while (scanned < total)
    {
        Blackbox_PollUartForStop();
        if (blackbox_abort_dump)
        {
            Debug_Printf("[BB] dumpall abort scanned=%lu.\r\n", (unsigned long)scanned);
            blackbox_abort_dump = false;
            blackbox_dumping = false;
            return;
        }
        if (W25Q64_Read(&blackbox_flash, address, record, BLACKBOX_RECORD_SIZE) != W25Q64_STATUS_OK)
        {
            Debug_Printf("[BB] dumpall read fail at 0x%08lX.\r\n", (unsigned long)address);
            break;
        }
        scanned++;
        /* 每条 64 字节拆成 4 行，避免超过 Debug_Printf 缓冲区。 */
        for (offset = 0U; offset < BLACKBOX_RECORD_SIZE; offset += 16U)
        {
            Debug_Printf("%08lX %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\r\n", (unsigned long)(address + offset), record[offset], record[offset + 1U], record[offset + 2U], record[offset + 3U], record[offset + 4U], record[offset + 5U], record[offset + 6U], record[offset + 7U], record[offset + 8U], record[offset + 9U], record[offset + 10U], record[offset + 11U], record[offset + 12U], record[offset + 13U], record[offset + 14U], record[offset + 15U]);
        }
        if ((scanned % 16U) == 0U) Debug_Printf("[BB] dumpall progress scanned=%lu total=%lu.\r\n", (unsigned long)scanned, (unsigned long)total);
        address = Blackbox_AdvanceLogAddress(address, 1U);
        if ((scanned % 8U) == 0U) vTaskDelay(1);
    }
    Debug_Printf("[BB] dumpall done scanned=%lu.\r\n", (unsigned long)scanned);
    blackbox_dumping = false;
}

static void Blackbox_ToUpperInPlace(char *text)
{
    while (*text != '\0')
    {
        if ((*text >= 'a') && (*text <= 'z')) *text = (char)(*text - ('a' - 'A'));
        text++;
    }
}

static bool Blackbox_MatchToken(const char **cursor, const char *token)
{
    size_t length = strlen(token);

    if (strncmp(*cursor, token, length) != 0) return false;
    if (((*cursor)[length] != '\0') && ((*cursor)[length] != ' ')) return false;
    *cursor += length;
    while (**cursor == ' ') (*cursor)++;
    return true;
}

static uint32_t Blackbox_ParseOptionalCount(const char *text)
{
    /* 不写条数：只导最近一段，避免从 Flash 开头把老 PID 先打出来。dump all 才从头到尾。 */
    if ((text == NULL) || (text[0] == '\0')) return BLACKBOX_DUMP_DEFAULT_RECORDS;
    if (strcmp(text, "ALL") == 0) return 0U;
    return (uint32_t)strtoul(text, NULL, 10);
}

static void Blackbox_PrintMagCalibrationStatus(void)
{
    MagCalibrationStatus_t status;

    Mag_CalibrationGetStatus(&status);
    Debug_Printf("[MAG_CAL] active=%u valid=%u quality=%u samples=%lu span=%.3f,%.3f,%.3f offset=%.3f,%.3f,%.3f.\r\n", status.active ? 1U : 0U, status.valid ? 1U : 0U, status.quality_ok ? 1U : 0U, (unsigned long)status.sample_count, (double)status.span_mx_gauss, (double)status.span_my_gauss, (double)status.span_mz_gauss, (double)status.offset_mx_gauss, (double)status.offset_my_gauss, (double)status.offset_mz_gauss);
}

static void Blackbox_HandleMagCalibration(const char *cursor)
{
    MagCalibrationStatus_t status;

    if (Blackbox_MatchToken(&cursor, "START"))
    {
        if (blackbox_recording)
        {
            Debug_Printf("[MAG_CAL] disarm first.\r\n");
            return;
        }
        if (Mag_CalibrationStart()) Debug_Printf("[MAG_CAL] started; rotate all axes slowly, then send calmag stop.\r\n");
        else Debug_Printf("[MAG_CAL] start fail; sensor is not ready or calibration is already active.\r\n");
    }
    else if (Blackbox_MatchToken(&cursor, "STOP"))
    {
        if (Mag_CalibrationStop(&status))
        {
            Debug_Printf("[MAG_CAL] done samples=%lu span=%.3f,%.3f,%.3f offset=%.3f,%.3f,%.3f.\r\n", (unsigned long)status.sample_count, (double)status.span_mx_gauss, (double)status.span_my_gauss, (double)status.span_mz_gauss, (double)status.offset_mx_gauss, (double)status.offset_my_gauss, (double)status.offset_mz_gauss);
            if (Blackbox_QueueMagCalSave()) Debug_Printf("[MAG_CAL] queued flash save at 0x002000.\r\n");
            else Debug_Printf("[MAG_CAL] RAM applied, flash save not queued; retry after dump/log idle.\r\n");
        }
        else
        {
            Mag_CalibrationGetStatus(&status);
            Debug_Printf("[MAG_CAL] rejected samples=%lu span=%.3f,%.3f,%.3f; rotate farther on all 3 axes.\r\n", (unsigned long)status.sample_count, (double)status.span_mx_gauss, (double)status.span_my_gauss, (double)status.span_mz_gauss);
        }
    }
    else if (Blackbox_MatchToken(&cursor, "STATUS")) Blackbox_PrintMagCalibrationStatus();
    else if (Blackbox_MatchToken(&cursor, "CANCEL"))
    {
        Mag_CalibrationCancel();
        Debug_Printf("[MAG_CAL] cancelled.\r\n");
    }
    else Debug_Printf("[MAG_CAL] use calmag start|stop|status|cancel.\r\n");
}

static void Blackbox_HandleCommandLine(void)
{
    const char *cursor;

    if (blackbox_command_length == 0U) return;
    Blackbox_ToUpperInPlace(blackbox_command);
    cursor = blackbox_command;
    (void)Blackbox_MatchToken(&cursor, "BB");
    if (Blackbox_MatchToken(&cursor, "STOP") || Blackbox_MatchToken(&cursor, "ABORT"))
    {
        blackbox_abort_dump = true;
        Debug_Printf("[BB] stop requested.\r\n");
    }
    else if (blackbox_dumping) Debug_Printf("[BB] busy, send stop first.\r\n");
    else if (Blackbox_MatchToken(&cursor, "DUMPALL")) Blackbox_DumpAllRaw(Blackbox_ParseOptionalCount(cursor));
    else if (Blackbox_MatchToken(&cursor, "DUMP")) Blackbox_DumpLogs(Blackbox_ParseOptionalCount(cursor));
    else if (Blackbox_MatchToken(&cursor, "CALMAG")) Blackbox_HandleMagCalibration(cursor);
    else if (Blackbox_MatchToken(&cursor, "HELP")) Debug_Printf("[BB] commands: dump [N|all] dumpall [N|all] calmag start|stop|status|cancel stop\r\n");
    else Debug_Printf("[BB] unknown cmd, send dump dumpall calmag or stop\r\n");
    blackbox_command_length = 0U;
    blackbox_command[0] = '\0';
}

static void Blackbox_CollectUartByte(uint8_t byte, bool commands_allowed)
{
    if ((byte == (uint8_t)'\r') || (byte == (uint8_t)'\n'))
    {
        if (commands_allowed) Blackbox_HandleCommandLine();
        else
        {
            Blackbox_ToUpperInPlace(blackbox_command);
            if ((strcmp(blackbox_command, "STOP") == 0) || (strcmp(blackbox_command, "ABORT") == 0) || (strcmp(blackbox_command, "BB STOP") == 0))
            {
                blackbox_abort_dump = true;
                Debug_Printf("[BB] stop requested.\r\n");
            }
            blackbox_command_length = 0U;
            blackbox_command[0] = '\0';
        }
        return;
    }
    if (blackbox_command_length + 1U >= BLACKBOX_COMMAND_BUFFER_SIZE)
    {
        blackbox_command_length = 0U;
        return;
    }
    if ((byte >= (uint8_t)' ') && (byte <= (uint8_t)'~'))
    {
        blackbox_command[blackbox_command_length++] = (char)byte;
        blackbox_command[blackbox_command_length] = '\0';
    }
}

static void Blackbox_PollUartForStop(void)
{
    uint8_t byte;

    if (__HAL_UART_GET_FLAG(BLACKBOX_UART, UART_FLAG_ORE) != RESET) __HAL_UART_CLEAR_FLAG(BLACKBOX_UART, UART_CLEAR_OREF);
    while (__HAL_UART_GET_FLAG(BLACKBOX_UART, UART_FLAG_RXNE) != RESET)
    {
        byte = (uint8_t)(BLACKBOX_UART->Instance->RDR & 0xFFU);
        Blackbox_CollectUartByte(byte, false);
    }
}

static void Blackbox_PollUart(void)
{
    uint8_t byte;

    if (__HAL_UART_GET_FLAG(BLACKBOX_UART, UART_FLAG_ORE) != RESET) __HAL_UART_CLEAR_FLAG(BLACKBOX_UART, UART_CLEAR_OREF);
    while (__HAL_UART_GET_FLAG(BLACKBOX_UART, UART_FLAG_RXNE) != RESET)
    {
        byte = (uint8_t)(BLACKBOX_UART->Instance->RDR & 0xFFU);
        Blackbox_CollectUartByte(byte, true);
    }
}

static bool Blackbox_Initialize(void)
{
    W25Q64_Status_t status;

    blackbox_ready = false;
    blackbox_recording = false;
    blackbox_save_pending = false;
    blackbox_mag_cal_save_pending = false;
    blackbox_has_saved = false;
    blackbox_has_saved_mag_cal = false;
    blackbox_mag_cal_generation = 0U;
    memset(&blackbox_saved_mag_cal, 0, sizeof(blackbox_saved_mag_cal));
    blackbox_dropped_samples = 0U;
    blackbox_parameter_generation = 0U;
    blackbox_next_session_id = 1U;
    blackbox_log_write_address = BLACKBOX_LOG_START_ADDRESS;
    blackbox_log_index_generation = 0U;
    blackbox_log_wrapped = false;
    blackbox_command_length = 0U;
    if (blackbox_queue == NULL)
    {
        blackbox_queue = xQueueCreateStatic(BLACKBOX_SAMPLE_QUEUE_LENGTH, sizeof(BlackboxJob_t), blackbox_queue_buffer, &blackbox_queue_storage);
    }
    /* dump 五个字节可能一次到齐，开 USART3 RX FIFO，避免轮询前溢出。 */
    (void)HAL_UARTEx_EnableFifoMode(BLACKBOX_UART);
    status = W25Q64_Init(&blackbox_flash, &hspi3, W_CS_GPIO_Port, W_CS_Pin);
    if (status != W25Q64_STATUS_OK)
    {
        Debug_Printf("[BB] flash init fail %u.\r\n", (unsigned int)status);
        return false;
    }
    if (Blackbox_ApplyBestSlot()) (void)blackbox_parameter_generation;
    else Debug_Printf("[BB] no saved pid, using defaults.\r\n");
    if (Blackbox_ApplyBestMagCalSlot()) (void)blackbox_mag_cal_generation;
    else Debug_Printf("[MAG_CAL] no saved offset, heading stays uncalibrated.\r\n");
    /* 先放行 SAVE，再扫日志，避免上电 SPI 长扫描拖死电机任务。 */
    blackbox_ready = true;
    vTaskDelay(pdMS_TO_TICKS(50U));
    if (Blackbox_LoadLogIndex())
    {
        Blackbox_RecoverWritePointer();
    }
    else if (!Blackbox_ScanLogEnd())
    {
        Debug_Printf("[BB] log scan fail, start at 0x%08lX.\r\n", (unsigned long)BLACKBOX_LOG_START_ADDRESS);
        blackbox_log_write_address = BLACKBOX_LOG_START_ADDRESS;
        blackbox_next_session_id = 1U;
        blackbox_log_wrapped = false;
    }
    else (void)Blackbox_WriteLogIndex();
    return true;
}

bool Blackbox_IsReady(void)
{
    return blackbox_ready;
}

bool Blackbox_IsRecording(void)
{
    return blackbox_recording;
}

bool Blackbox_IsDumping(void)
{
    return blackbox_dumping;
}

bool Blackbox_RequestSave(void)
{
    if (!blackbox_ready) return false;
    if (blackbox_save_pending) return false;
    blackbox_save_pending = true;
    if (!Blackbox_QueueJob(BLACKBOX_JOB_SAVE, NULL))
    {
        blackbox_save_pending = false;
        return false;
    }
    return true;
}

bool Blackbox_RequestMagCalibrationSave(void)
{
    return Blackbox_QueueMagCalSave();
}

bool Blackbox_ApplySavedMagCalibration(void)
{
    if (!blackbox_has_saved_mag_cal) return false;
    return Mag_CalibrationApplySaved(&blackbox_saved_mag_cal);
}

bool Blackbox_LoadSavedParameters(void)
{
    if (!blackbox_ready || !blackbox_has_saved) return false;
    return PidTable_ImportSnapshot(blackbox_saved_values);
}

void Blackbox_NotifyArmed(void)
{
    (void)Blackbox_QueueJob(BLACKBOX_JOB_ARM, NULL);
}

void Blackbox_NotifyDisarmed(void)
{
    (void)Blackbox_QueueJob(BLACKBOX_JOB_DISARM, NULL);
}

void Blackbox_SubmitSample(const BlackboxSample_t *sample)
{
    if (sample == NULL) return;
    (void)Blackbox_QueueJob(BLACKBOX_JOB_SAMPLE, sample);
}

void Blackbox_SubmitPositionSample(const BlackboxPositionSample_t *sample)
{
    if ((sample == NULL) || !blackbox_recording) return;
    (void)Blackbox_QueuePositionJob(sample);
}

void Blackbox_Task(void *argument)
{
    BlackboxJob_t job;

    (void)argument;
    PidTable_Init();
    for (;;)
    {
        if (!Blackbox_Initialize())
        {
            vTaskDelay(pdMS_TO_TICKS(1000U));
            continue;
        }
        for (;;)
        {
            if (xQueueReceive(blackbox_queue, &job, pdMS_TO_TICKS(BLACKBOX_UART_POLL_MS)) == pdTRUE)
            {
                if (job.type == BLACKBOX_JOB_SAVE) Blackbox_HandleSave();
                else if (job.type == BLACKBOX_JOB_SAVE_MAG_CAL) Blackbox_HandleMagCalSave();
                else if (job.type == BLACKBOX_JOB_ARM) Blackbox_HandleArm();
                else if (job.type == BLACKBOX_JOB_DISARM) Blackbox_HandleDisarm();
                else if ((job.type == BLACKBOX_JOB_SAMPLE) && blackbox_recording)
                {
                    if (!Blackbox_WriteSample(&job.sample))
                    {
                        blackbox_recording = false;
                        Debug_Printf("[BB] write fail, stop.\r\n");
                    }
                }
                else if ((job.type == BLACKBOX_JOB_POSITION_SAMPLE) && blackbox_recording)
                {
                    if (!Blackbox_WritePositionSample(&job.position))
                    {
                        blackbox_recording = false;
                        Debug_Printf("[BB] position write fail, stop.\r\n");
                    }
                }
            }
            Blackbox_PollUart();
        }
    }
}
