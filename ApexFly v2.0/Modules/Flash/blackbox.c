/**
 * @file blackbox.c
 * @brief W25Q64 黑匣子实现：PID 双备份、场次日志、串口 CSV 导出。
 *
 * 分区：
 * 0x000000 参数槽 A，4KB
 * 0x001000 参数槽 B，4KB
 * 0x002000 保留（老六面校准，不读不写）
 * 0x004000 日志起点
 * 最后 4KB 自检扇区，不拿来记飞
 *
 * 日志记录固定 64 字节。写满则停记，不环写。
 * 场次头里的 PID 用 q100（百分之一）钉死当时那套参，方便导出后对上波形。
 * USART3 轮询收 dump / dumpall / help；SAVE / 日志写入只在本任务做。
 * dump 只打校验通过的新场次 CSV；dumpall 把日志区每条 64 字节原样打成十六进制，含老数据。
 */

#include "blackbox.h"

#include "apex_protocol.h"
#include "debug.h"
#include "main.h"
#include "pid_table.h"
#include "spi.h"
#include "usart.h"
#include "w25q64.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define BLACKBOX_PARAMETER_SECTOR_A 0x000000UL
#define BLACKBOX_PARAMETER_SECTOR_B 0x001000UL
#define BLACKBOX_LOG_START_ADDRESS 0x004000UL
#define BLACKBOX_TEST_SECTOR_ADDRESS (W25Q64_TOTAL_SIZE_BYTES - W25Q64_SECTOR_SIZE_BYTES)
#define BLACKBOX_LOG_END_ADDRESS BLACKBOX_TEST_SECTOR_ADDRESS

#define BLACKBOX_PARAMETER_MAGIC 0x50445032UL /* "PDP2"，与老工程 magic 区分。 */
#define BLACKBOX_PARAMETER_SCHEMA 1U
#define BLACKBOX_RECORD_MAGIC 0x424C5832UL /* "BLX2" */
#define BLACKBOX_RECORD_SCHEMA 1U
#define BLACKBOX_RECORD_TYPE_SESSION_BEGIN 1U
#define BLACKBOX_RECORD_TYPE_SAMPLE 2U
#define BLACKBOX_RECORD_TYPE_SESSION_END 3U
#define BLACKBOX_RECORD_SIZE 64U
#define BLACKBOX_RECORD_CRC_OFFSET 60U
#define BLACKBOX_SAMPLE_QUEUE_LENGTH 32U

#define BLACKBOX_UART (&huart3)
#define BLACKBOX_COMMAND_BUFFER_SIZE 40U
#define BLACKBOX_UART_POLL_MS 5U
#define BLACKBOX_STATUS_PRINT_MS 10000U
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

#pragma pack(push, 1)
typedef struct
{
    uint32_t magic;
    uint8_t type;
    uint8_t schema;
    uint16_t session_id;
    int16_t pid_q100[APEX_PID_PARAMETER_COUNT];
    uint16_t crc16;
    uint16_t reserved;
} BlackboxSessionBeginRecord_t;

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
    uint8_t armed;
    uint8_t connected;
    uint8_t pad[18];
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
    BlackboxSampleRecord_t sample;
    BlackboxSessionEndRecord_t end;
} BlackboxFlashRecord_t;

typedef struct
{
    bool valid;
    uint32_t generation;
    int32_t values_q10000[APEX_PID_PARAMETER_COUNT];
} BlackboxParameterCandidate_t;

typedef enum
{
    BLACKBOX_JOB_NONE = 0,
    BLACKBOX_JOB_SAMPLE,
    BLACKBOX_JOB_SAVE,
    BLACKBOX_JOB_ARM,
    BLACKBOX_JOB_DISARM
} BlackboxJobType_t;

typedef struct
{
    BlackboxJobType_t type;
    BlackboxSample_t sample;
} BlackboxJob_t;

static W25Q64_Device_t blackbox_flash;
static QueueHandle_t blackbox_queue;
static StaticQueue_t blackbox_queue_storage;
static uint8_t blackbox_queue_buffer[BLACKBOX_SAMPLE_QUEUE_LENGTH * sizeof(BlackboxJob_t)];
static volatile bool blackbox_ready;
static volatile bool blackbox_recording;
static volatile bool blackbox_save_pending;
static volatile bool blackbox_has_saved;
static volatile uint32_t blackbox_dropped_samples;
static uint32_t blackbox_log_write_address;
static uint32_t blackbox_parameter_generation;
static uint16_t blackbox_next_session_id;
static uint16_t blackbox_active_session_id;
static uint32_t blackbox_session_start_tick;
static uint16_t blackbox_session_sample_count;
static int32_t blackbox_saved_values[APEX_PID_PARAMETER_COUNT];
static char blackbox_command[BLACKBOX_COMMAND_BUFFER_SIZE];
static uint8_t blackbox_command_length;

typedef char BlackboxParameterRecordMustFit[(sizeof(BlackboxParameterRecord_t) <= W25Q64_SECTOR_SIZE_BYTES) ? 1 : -1];
typedef char BlackboxSessionBeginMustBe64[(sizeof(BlackboxSessionBeginRecord_t) == BLACKBOX_RECORD_SIZE) ? 1 : -1];
typedef char BlackboxSampleMustBe64[(sizeof(BlackboxSampleRecord_t) == BLACKBOX_RECORD_SIZE) ? 1 : -1];
typedef char BlackboxSessionEndMustBe64[(sizeof(BlackboxSessionEndRecord_t) == BLACKBOX_RECORD_SIZE) ? 1 : -1];
typedef char BlackboxFlashRecordMustBe64[(sizeof(BlackboxFlashRecord_t) == BLACKBOX_RECORD_SIZE) ? 1 : -1];

static volatile bool blackbox_abort_dump;
static bool blackbox_dumping;

static void Blackbox_PollUartForStop(void);

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
    uint16_t crc;

    memset(candidate, 0, sizeof(*candidate));
    if (W25Q64_Read(&blackbox_flash, address, (uint8_t *)&record, sizeof(record)) != W25Q64_STATUS_OK) return false;
    if (record.magic != BLACKBOX_PARAMETER_MAGIC) return false;
    if (record.schema != BLACKBOX_PARAMETER_SCHEMA) return false;
    if (record.parameter_count != APEX_PID_PARAMETER_COUNT) return false;
    crc = ApexProtocol_Crc16((const uint8_t *)&record, (uint16_t)(sizeof(record) - 4U));
    if (crc != record.crc16) return false;
    candidate->valid = true;
    candidate->generation = record.generation;
    memcpy(candidate->values_q10000, record.values_q10000, sizeof(candidate->values_q10000));
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

static bool Blackbox_ScanLogEnd(void)
{
    uint8_t magic_bytes[8];
    uint32_t address;
    uint32_t magic;
    uint16_t max_session = 0U;
    uint32_t used_records = 0U;

    blackbox_log_write_address = BLACKBOX_LOG_START_ADDRESS;
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
    blackbox_log_write_address = BLACKBOX_LOG_END_ADDRESS;
    blackbox_next_session_id = (uint16_t)(max_session + 1U);
    if (blackbox_next_session_id == 0U) blackbox_next_session_id = 1U;
    return true;
}

static bool Blackbox_LogHasSpace(void)
{
    return (blackbox_log_write_address + BLACKBOX_RECORD_SIZE) <= BLACKBOX_LOG_END_ADDRESS;
}

static bool Blackbox_AppendRecord(uint8_t *bytes)
{
    Blackbox_FinalizeRecord(bytes);
    if (!Blackbox_LogHasSpace()) return false;
    if ((blackbox_log_write_address % W25Q64_SECTOR_SIZE_BYTES) == 0U)
    {
        if (W25Q64_EraseSector(&blackbox_flash, blackbox_log_write_address) != W25Q64_STATUS_OK) return false;
    }
    if (W25Q64_Write(&blackbox_flash, blackbox_log_write_address, bytes, BLACKBOX_RECORD_SIZE) != W25Q64_STATUS_OK) return false;
    blackbox_log_write_address += BLACKBOX_RECORD_SIZE;
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
    for (index = 0U; index < APEX_PID_PARAMETER_COUNT; index++) record.begin.pid_q100[index] = Blackbox_Q10000ToQ100(values[index]);
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
    record.end.utc_seconds = 0U;
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
    record.sample.armed = sample->armed;
    record.sample.connected = sample->connected;
    if (!Blackbox_AppendRecord(record.bytes)) return false;
    if (blackbox_session_sample_count < 0xFFFFU) blackbox_session_sample_count++;
    return true;
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
    if (blackbox_recording) (void)Blackbox_WriteSessionEnd();
    if (!Blackbox_LogHasSpace())
    {
        blackbox_recording = false;
        Debug_Printf("[BB] log full, not recording.\r\n");
        return;
    }
    blackbox_active_session_id = blackbox_next_session_id;
    if (blackbox_next_session_id < 0xFFFFU) blackbox_next_session_id++;
    blackbox_session_start_tick = xTaskGetTickCount();
    blackbox_session_sample_count = 0U;
    if (!Blackbox_WriteSessionBegin())
    {
        blackbox_recording = false;
        Debug_Printf("[BB] session begin failed.\r\n");
        return;
    }
    blackbox_recording = true;
    Debug_Printf("[BB] session %u begin.\r\n", (unsigned int)blackbox_active_session_id);
}

static void Blackbox_HandleDisarm(void)
{
    if (!blackbox_recording) return;
    if (!Blackbox_WriteSessionEnd()) Debug_Printf("[BB] session end failed.\r\n");
    else Debug_Printf("[BB] session %u end samples=%u.\r\n", (unsigned int)blackbox_active_session_id, (unsigned int)blackbox_session_sample_count);
    blackbox_recording = false;
}

static void Blackbox_PrintPidHeader(uint16_t session_id, const int16_t *pid_q100)
{
    char line[400];
    int length;
    uint8_t index;

    Debug_Printf("# session=%u t0_utc=000000\r\n", (unsigned int)session_id);
    /* 整行一次发出，避免电机任务的 [ATT] 插进 PID 数字中间。 */
    length = snprintf(line, sizeof(line), "# pid");
    for (index = 0U; (index < APEX_PID_PARAMETER_COUNT) && (length > 0) && (length < (int)(sizeof(line) - 24)); index++)
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
    Debug_Printf("t_ms,roll,pitch,yaw,roll_rate,pitch_rate,yaw_rate,roll_corr,pitch_corr,yaw_corr,thr,m1,m2,m3,m4,armed,conn\r\n");
}

static uint32_t Blackbox_DumpStartAddress(uint32_t max_records)
{
    uint32_t total;
    uint32_t skip;

    if (blackbox_log_write_address <= BLACKBOX_LOG_START_ADDRESS) return BLACKBOX_LOG_START_ADDRESS;
    total = (blackbox_log_write_address - BLACKBOX_LOG_START_ADDRESS) / BLACKBOX_RECORD_SIZE;
    if ((max_records == 0U) || (max_records >= total)) return BLACKBOX_LOG_START_ADDRESS;
    skip = total - max_records;
    return BLACKBOX_LOG_START_ADDRESS + (skip * BLACKBOX_RECORD_SIZE);
}

static uint32_t Blackbox_DumpRecordCount(uint32_t start_address)
{
    if (blackbox_log_write_address <= start_address) return 0U;
    return (blackbox_log_write_address - start_address) / BLACKBOX_RECORD_SIZE;
}

static void Blackbox_DumpLogs(uint32_t max_records)
{
    BlackboxFlashRecord_t record;
    uint32_t address;
    uint32_t start_address;
    uint32_t total;
    uint32_t scanned = 0U;
    uint32_t samples = 0U;
    uint32_t sessions = 0U;
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
    total = Blackbox_DumpRecordCount(start_address);
    Debug_Printf("[BB] dump start records=%lu addr=0x%08lX end=0x%08lX.\r\n", (unsigned long)total, (unsigned long)start_address, (unsigned long)blackbox_log_write_address);
    for (address = start_address; address < blackbox_log_write_address; address += BLACKBOX_RECORD_SIZE)
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
        if (Blackbox_ReadU32(record.bytes, 0U) == 0xFFFFFFFFUL) break;
        if (!Blackbox_RecordValid(record.bytes)) continue;
        if (record.begin.type == BLACKBOX_RECORD_TYPE_SESSION_BEGIN)
        {
            in_session = true;
            sessions++;
            Blackbox_PrintPidHeader(record.begin.session_id, record.begin.pid_q100);
        }
        else if (record.sample.type == BLACKBOX_RECORD_TYPE_SAMPLE)
        {
            if (!in_session && !printed_orphan_header)
            {
                Debug_Printf("# session=0 t0_utc=000000\r\n");
                Debug_Printf("t_ms,roll,pitch,yaw,roll_rate,pitch_rate,yaw_rate,roll_corr,pitch_corr,yaw_corr,thr,m1,m2,m3,m4,armed,conn\r\n");
                printed_orphan_header = true;
            }
            if (in_session || printed_orphan_header)
            {
                samples++;
                Debug_Printf("%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d,%d,%u,%u,%u,%u,%u,%u,%u\r\n", (unsigned long)record.sample.t_ms, (double)record.sample.roll_cdeg / 100.0, (double)record.sample.pitch_cdeg / 100.0, (double)record.sample.yaw_cdeg / 100.0, (double)record.sample.roll_rate_cdps / 100.0, (double)record.sample.pitch_rate_cdps / 100.0, (double)record.sample.yaw_rate_cdps / 100.0, (int)record.sample.roll_corr_us, (int)record.sample.pitch_corr_us, (int)record.sample.yaw_corr_us, (unsigned int)record.sample.throttle_us, (unsigned int)record.sample.motor_us[0], (unsigned int)record.sample.motor_us[1], (unsigned int)record.sample.motor_us[2], (unsigned int)record.sample.motor_us[3], (unsigned int)record.sample.armed, (unsigned int)record.sample.connected);
            }
        }
        else if (record.end.type == BLACKBOX_RECORD_TYPE_SESSION_END)
        {
            Debug_Printf("# end session=%u samples=%u t_ms=%lu utc=000000\r\n", (unsigned int)record.end.session_id, (unsigned int)record.end.sample_count, (unsigned long)record.end.t_ms);
            in_session = false;
        }
        if ((scanned % 8U) == 0U) vTaskDelay(1);
    }
    Debug_Printf("[BB] dump done scanned=%lu samples=%lu sessions=%lu.\r\n", (unsigned long)scanned, (unsigned long)samples, (unsigned long)sessions);
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
    total = Blackbox_DumpRecordCount(start_address);
    Debug_Printf("[BB] dumpall start records=%lu addr=0x%08lX end=0x%08lX.\r\n", (unsigned long)total, (unsigned long)start_address, (unsigned long)blackbox_log_write_address);
    for (address = start_address; address < blackbox_log_write_address; address += BLACKBOX_RECORD_SIZE)
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
    else if (Blackbox_MatchToken(&cursor, "HELP")) Debug_Printf("[BB] commands: dump [N|all] dumpall [N|all] stop\r\n");
    else Debug_Printf("[BB] unknown cmd, send dump dumpall or stop\r\n");
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
    blackbox_has_saved = false;
    blackbox_dropped_samples = 0U;
    blackbox_parameter_generation = 0U;
    blackbox_next_session_id = 1U;
    blackbox_log_write_address = BLACKBOX_LOG_START_ADDRESS;
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
    if (Blackbox_ApplyBestSlot()) Debug_Printf("[BB] pid loaded gen=%lu.\r\n", (unsigned long)blackbox_parameter_generation);
    else Debug_Printf("[BB] no saved pid, using defaults.\r\n");
    /* 先放行 SAVE，再扫日志，避免上电 SPI 长扫描拖死电机任务。 */
    blackbox_ready = true;
    vTaskDelay(pdMS_TO_TICKS(50U));
    if (!Blackbox_ScanLogEnd())
    {
        Debug_Printf("[BB] log scan fail, start at 0x%08lX.\r\n", (unsigned long)BLACKBOX_LOG_START_ADDRESS);
        blackbox_log_write_address = BLACKBOX_LOG_START_ADDRESS;
        blackbox_next_session_id = 1U;
    }
    Debug_Printf("[BB] ready id=%02X%02X%02X next_session=%u log=0x%08lX.\r\n", (unsigned int)blackbox_flash.manufacturer_id, (unsigned int)blackbox_flash.memory_type, (unsigned int)blackbox_flash.capacity_id, (unsigned int)blackbox_next_session_id, (unsigned long)blackbox_log_write_address);
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

void Blackbox_Task(void *argument)
{
    BlackboxJob_t job;
    uint32_t next_status_tick;

    (void)argument;
    Debug_Init();
    PidTable_Init();
    for (;;)
    {
        if (!Blackbox_Initialize())
        {
            vTaskDelay(pdMS_TO_TICKS(1000U));
            continue;
        }
        next_status_tick = (uint32_t)xTaskGetTickCount() + BLACKBOX_STATUS_PRINT_MS;
        for (;;)
        {
            if (xQueueReceive(blackbox_queue, &job, pdMS_TO_TICKS(BLACKBOX_UART_POLL_MS)) == pdTRUE)
            {
                if (job.type == BLACKBOX_JOB_SAVE) Blackbox_HandleSave();
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
            }
            Blackbox_PollUart();
            if ((int32_t)(xTaskGetTickCount() - next_status_tick) >= 0)
            {
                Debug_Printf("[BB] rec=%u sess=%u wr=0x%08lX drop=%lu.\r\n", blackbox_recording ? 1U : 0U, (unsigned int)blackbox_active_session_id, (unsigned long)blackbox_log_write_address, (unsigned long)blackbox_dropped_samples);
                next_status_tick = (uint32_t)xTaskGetTickCount() + BLACKBOX_STATUS_PRINT_MS;
            }
        }
    }
}
