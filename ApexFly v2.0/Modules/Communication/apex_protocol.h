/**
 * @file apex_protocol.h
 * @brief 遥控器与飞机端共用的二进制控制/遥测协议。
 *
 * 两端必须使用同一份文件。当前版本 0x07，与 CoreFly F2 Remoto 一致。
 * 帧中多字节数据均为小端，不直接发送 C 结构体，避免对齐差异。
 * 本文件只做编解码，不读写 LoRa，也不写电机。
 * 飞机 v2 尚未接入的遥测字段填 0。
 * home_distance_m 现复用为磁航向整度 0～359；无效填 0xFFFF。HOME_VALID 表示磁力计样本有效。
 * 操作码 5/6/7 已废弃，飞机回 INVALID_OPERATION。
 * 8/9/10 给磁力计硬铁校准：开始采集、结束并写入 0x002000、取消本次。
 */

#ifndef APEX_PROTOCOL_H
#define APEX_PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define APEX_PROTOCOL_HEADER_0 0xA5U
#define APEX_PROTOCOL_HEADER_1 0x5AU
#define APEX_PROTOCOL_VERSION 0x07U
/* 单帧负载上限。⚠️ 2026-09-15 由 128 抬到 140：
   加了高度环 3 项之后，PID 回包 = 4 + 33×4 = **136** 字节，
   原先的 128 会让下面那句 #error 在编译期直接拦下。
   140 = 136 + 4 字节余量，仍远低于 LoRa 模块的能力上限（ra01sc.h 的 255）。
   ⚠️ 代价与为什么可以接受：PID 回包帧从 133 → 145 字节，空口时间约 +9%，
      但它是**请求触发的一次性报文**（进 PID 页 GET / 改参 SET / SAVE），
      不是周期流量 ⇒ **不影响遥测帧与控制帧**，飞行中的链路余量不变。
   ⚠️ 这一项是两端共用的常量（遥控端 apex_protocol.h 必须同值），改了两端都要重烧。 */
#define APEX_PROTOCOL_MAX_PAYLOAD_LENGTH 140U
#define APEX_PROTOCOL_MAX_FRAME_LENGTH (9U + APEX_PROTOCOL_MAX_PAYLOAD_LENGTH)
#define APEX_PROTOCOL_CONTROL_PAYLOAD_LENGTH 10U
#define APEX_PROTOCOL_TELEMETRY_PAYLOAD_LENGTH 31U
#define APEX_PROTOCOL_PID_COMMAND_PAYLOAD_LENGTH 6U
#define APEX_PID_LEGACY_PARAMETER_COUNT 15U
#define APEX_PID_V5_PARAMETER_COUNT 21U
#define APEX_PID_V6_PARAMETER_COUNT 26U
#define APEX_PID_V7_PARAMETER_COUNT 30U
#define APEX_PID_PARAMETER_COUNT 33U
#define APEX_PROTOCOL_PID_RESPONSE_PAYLOAD_LENGTH (4U + (APEX_PID_PARAMETER_COUNT * 4U))
#define APEX_PID_VALUE_SCALE 10000L

/* PID 回包包含序号、状态、操作类型、参数版本及全部 33 个参数；负载上限不足时必须在编译期报错，避免遥控器永久等待确认。 */
#if (APEX_PROTOCOL_PID_RESPONSE_PAYLOAD_LENGTH > APEX_PROTOCOL_MAX_PAYLOAD_LENGTH)
#error "APEX_PROTOCOL_MAX_PAYLOAD_LENGTH is too small for the PID response."
#endif

/* 遥控器请求保持解锁的控制标志。 */
#define APEX_CONTROL_FLAG_ARM_REQUESTED 0x01U

/* 飞机返回给遥控器的状态标志。 */
#define APEX_TELEMETRY_FLAG_ARMED 0x01U
#define APEX_TELEMETRY_FLAG_GPS_FIX 0x02U
/* 磁力计样本有效，home_distance_m 是 0～359 的磁航向整度。以后做 GPS 返航再另开字段。 */
#define APEX_TELEMETRY_FLAG_HOME_VALID 0x04U
#define APEX_TELEMETRY_FLAG_BLACKBOX_RECORDING 0x08U
#define APEX_TELEMETRY_FLAG_IMU_READY 0x10U

/* 飞机端四路电机PWM已经完成初始化，可以接受控制指令。 */
#define APEX_TELEMETRY_FLAG_MOTOR_READY 0x20U

/* 飞机端已从 GPS RMC 语句取得有效 UTC 日期时间，遥控器可据此显示北京时间。 */
#define APEX_TELEMETRY_FLAG_GPS_TIME_VALID 0x40U

/* 飞机正在采集磁力计硬铁校准样本。 */
#define APEX_TELEMETRY_FLAG_MAG_CALIBRATING 0x80U

typedef enum
{
    APEX_MESSAGE_CONTROL = 0x01,
    APEX_MESSAGE_PING = 0x10,
    APEX_MESSAGE_PID_COMMAND = 0x20,
    APEX_MESSAGE_TELEMETRY = 0x81,
    APEX_MESSAGE_PONG = 0x90,
    APEX_MESSAGE_PID_RESPONSE = 0xA0
} ApexMessageType_t;

typedef enum
{
    APEX_PID_OPERATION_GET = 0,
    APEX_PID_OPERATION_SET,
    APEX_PID_OPERATION_SAVE,
    APEX_PID_OPERATION_LOAD_SAVED,
    APEX_PID_OPERATION_LOAD_DEFAULTS,
    /* 已废弃的操作码，飞机收到后回 INVALID_OPERATION，数值保留以免和遥控错位。 */
    APEX_PID_OPERATION_RESERVED_5 = 5,
    APEX_PID_OPERATION_RESERVED_6 = 6,
    APEX_PID_OPERATION_RESERVED_7 = 7,
    /* 磁力计硬铁校准：开始采集三轴极值。飞机必须锁定。 */
    APEX_PID_OPERATION_MAG_CAL_START = 8,
    /* 磁力计硬铁校准：结束采集、计算偏移，并写入 0x002000。 */
    APEX_PID_OPERATION_MAG_CAL_STOP = 9,
    /* 磁力计硬铁校准：取消本次采集，不更新偏移。 */
    APEX_PID_OPERATION_MAG_CAL_CANCEL = 10
} ApexPidOperation_t;

typedef enum
{
    APEX_PID_STATUS_OK = 0,
    APEX_PID_STATUS_INVALID_OPERATION,
    APEX_PID_STATUS_INVALID_PARAMETER,
    APEX_PID_STATUS_OUT_OF_RANGE,
    APEX_PID_STATUS_LOCKED_REQUIRED,
    APEX_PID_STATUS_STORAGE_NOT_READY,
    APEX_PID_STATUS_STORAGE_ERROR
} ApexPidStatus_t;

typedef enum
{
    APEX_PID_ROLL_OUTER_KP = 0,
    APEX_PID_ROLL_OUTER_KI,
    APEX_PID_ROLL_OUTER_KD,
    APEX_PID_ROLL_INNER_KP,
    APEX_PID_ROLL_INNER_KI,
    APEX_PID_ROLL_INNER_KD,
    APEX_PID_PITCH_OUTER_KP,
    APEX_PID_PITCH_OUTER_KI,
    APEX_PID_PITCH_OUTER_KD,
    APEX_PID_PITCH_INNER_KP,
    APEX_PID_PITCH_INNER_KI,
    APEX_PID_PITCH_INNER_KD,
    APEX_PID_YAW_RATE_KP,
    APEX_PID_YAW_RATE_KI,
    APEX_PID_YAW_RATE_KD,
    APEX_PID_ROLL_MAX_RATE_DPS,
    APEX_PID_ROLL_MAX_CORRECTION_US,
    APEX_PID_PITCH_MAX_RATE_DPS,
    APEX_PID_PITCH_MAX_CORRECTION_US,
    APEX_PID_YAW_MAX_RATE_DPS,
    APEX_PID_YAW_MAX_CORRECTION_US,
    /* 悬停姿态 Trim 只改变摇杆回中时的目标角，不参与陀螺仪零偏校准。 */
    APEX_PID_ROLL_TRIM_DEG,
    APEX_PID_PITCH_TRIM_DEG,
    /* Yaw航向角度环PID参数（GPS定点模式使用） */
    APEX_PID_YAW_ANGLE_KP,
    APEX_PID_YAW_ANGLE_KI,
    APEX_PID_YAW_ANGLE_KD,
    /* 高度控制追加在旧 26 项之后，旧姿态参数编号保持不变。 */
    APEX_PID_VERTICAL_SPEED_KP,
    APEX_PID_VERTICAL_SPEED_KI,
    APEX_PID_VERTICAL_SPEED_KD,
    /* 满杆油门上限，单位微秒，最低油门固定 1000。
       枚举名里的 HOVER 是历史遗留：它不表示"悬停需要多少油门"，
       飞机端模块内部一律叫 max_throttle_us，改名只为对齐语义，协议数值不变。 */
    APEX_PID_HOVER_PWM_US,
    /* 高度环（**外环**）PID，2026-09-15 追加在最后，旧编号一律不变。
       ⚠️⚠️ 与上面那组 VERTICAL_SPEED_* 是**两个不同的环**，千万不要混：
            VERTICAL_SPEED_KP/KI/KD = **垂速环（内环）**：垂速误差 cm/s → 油门修正量 µs
            HEIGHT_KP/KI/KD         = **高度环（外环）**：高度误差 cm   → 目标垂速 cm/s
       控制代码要到 H13 才会去读这三个值；在那之前它们只是"协议 / 遥控可读写、
       可存 Flash、可进黑匣子场次头"的参数，**不参与任何控制**（改它们不会影响飞行）。 */
    APEX_PID_HEIGHT_KP,
    APEX_PID_HEIGHT_KI,
    APEX_PID_HEIGHT_KD
} ApexPidParameterId_t;

typedef enum
{
    APEX_PROTOCOL_OK = 0,
    APEX_PROTOCOL_INVALID_PARAM,
    APEX_PROTOCOL_BUFFER_TOO_SMALL,
    APEX_PROTOCOL_BAD_HEADER,
    APEX_PROTOCOL_BAD_VERSION,
    APEX_PROTOCOL_BAD_LENGTH,
    APEX_PROTOCOL_BAD_CRC,
    APEX_PROTOCOL_BAD_MESSAGE_TYPE
} ApexProtocolStatus_t;

/* 遥控器每100 ms发送一次的控制负载。量程：油门0~1000，姿态/偏航-1000~1000。 */
typedef struct
{
    uint16_t sequence;
    uint16_t throttle;
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
    uint8_t flags;
    uint8_t flight_mode;
} ApexControlCommand_t;

/* 飞机回传的遥测负载。当前电机通信阶段未接入的传感器字段填0。 */
typedef struct
{
    uint16_t sequence;
    int16_t roll_cdeg;
    int16_t pitch_cdeg;
    int16_t yaw_cdeg;
    int32_t altitude_cm;
    int16_t vertical_speed_cms;
    uint16_t ground_speed_cms;
    uint16_t battery_mv;
    /* 磁航向整度，0～359。无效为 0xFFFF。字段名沿用 home_distance_m，包布局不变。 */
    uint16_t home_distance_m;
    int16_t rssi_dbm;
    int8_t snr_q4;
    uint8_t satellites;
    uint8_t link_quality;
    uint8_t flags;
    uint8_t flight_mode;
    uint16_t throttle_pwm_us;
    uint32_t gps_utc_seconds;
} ApexTelemetry_t;

typedef struct
{
    uint16_t sequence;
    ApexPidOperation_t operation;
    uint8_t parameter_id;
    int32_t value_q10000;
} ApexPidCommand_t;

typedef struct
{
    uint16_t sequence;
    ApexPidStatus_t status;
    ApexPidOperation_t operation;
    uint16_t parameter_version;
    int32_t values_q10000[APEX_PID_PARAMETER_COUNT];
} ApexPidResponse_t;

typedef struct
{
    ApexMessageType_t message_type;
    uint16_t sequence;
    uint8_t payload_length;
    uint8_t payload[APEX_PROTOCOL_MAX_PAYLOAD_LENGTH];
} ApexFrame_t;

uint16_t ApexProtocol_Crc16(const uint8_t *data, uint16_t length);
ApexProtocolStatus_t ApexProtocol_EncodeControl(const ApexControlCommand_t *command, uint8_t *frame, uint8_t capacity, uint8_t *frame_length);
ApexProtocolStatus_t ApexProtocol_EncodeTelemetry(const ApexTelemetry_t *telemetry, uint8_t *frame, uint8_t capacity, uint8_t *frame_length);
ApexProtocolStatus_t ApexProtocol_EncodePidCommand(const ApexPidCommand_t *command, uint8_t *frame, uint8_t capacity, uint8_t *frame_length);
ApexProtocolStatus_t ApexProtocol_EncodePidResponse(const ApexPidResponse_t *response, uint8_t *frame, uint8_t capacity, uint8_t *frame_length);
ApexProtocolStatus_t ApexProtocol_DecodeFrame(const uint8_t *frame, uint8_t frame_length, ApexFrame_t *decoded_frame);
ApexProtocolStatus_t ApexProtocol_DecodeControl(const ApexFrame_t *frame, ApexControlCommand_t *command);
ApexProtocolStatus_t ApexProtocol_DecodeTelemetry(const ApexFrame_t *frame, ApexTelemetry_t *telemetry);
ApexProtocolStatus_t ApexProtocol_DecodePidCommand(const ApexFrame_t *frame, ApexPidCommand_t *command);
ApexProtocolStatus_t ApexProtocol_DecodePidResponse(const ApexFrame_t *frame, ApexPidResponse_t *response);

#ifdef __cplusplus
}
#endif

#endif
