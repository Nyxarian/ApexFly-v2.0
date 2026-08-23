/**
 * @file apex_protocol.c
 * @brief ApexFly 二进制通信协议编解码实现。
 *
 * 与遥控器 CoreFly F2 Remoto 使用同一套 0x06 帧格式。
 * 改字节布局必须两端一起改，否则解不开。
 * 本文件只负责打包/拆包，不接触 SPI 和电机。
 */

#include "apex_protocol.h"
#include <stddef.h>
#include <string.h>

/* 帧中协议版本字段的偏移。 */
#define APEX_FRAME_VERSION_OFFSET 2U

/* 帧中消息类型字段的偏移。 */
#define APEX_FRAME_TYPE_OFFSET 3U

/* 帧中序号低字节的偏移。 */
#define APEX_FRAME_SEQUENCE_OFFSET 4U

/* 帧中负载长度字段的偏移。 */
#define APEX_FRAME_LENGTH_OFFSET 6U

/* 帧中有效负载开始位置。 */
#define APEX_FRAME_PAYLOAD_OFFSET 7U

/**
 * @brief 按小端顺序写入16位无符号数。
 */
static void ApexProtocol_WriteU16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8U);
}

/**
 * @brief 按小端顺序写入16位有符号数。
 */
static void ApexProtocol_WriteI16(uint8_t *data, int16_t value)
{
    ApexProtocol_WriteU16(data, (uint16_t)value);
}

/**
 * @brief 按小端顺序写入32位有符号数。
 */
static void ApexProtocol_WriteI32(uint8_t *data, int32_t value)
{
    uint32_t unsigned_value;

    unsigned_value = (uint32_t)value;
    data[0] = (uint8_t)unsigned_value;
    data[1] = (uint8_t)(unsigned_value >> 8U);
    data[2] = (uint8_t)(unsigned_value >> 16U);
    data[3] = (uint8_t)(unsigned_value >> 24U);
}

/**
 * @brief 按小端顺序读取16位无符号数。
 */
static uint16_t ApexProtocol_ReadU16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

/**
 * @brief 按小端顺序读取16位有符号数。
 */
static int16_t ApexProtocol_ReadI16(const uint8_t *data)
{
    return (int16_t)ApexProtocol_ReadU16(data);
}

/**
 * @brief 按小端顺序读取32位有符号数。
 */
static int32_t ApexProtocol_ReadI32(const uint8_t *data)
{
    uint32_t value;

    value = (uint32_t)data[0];
    value |= (uint32_t)data[1] << 8U;
    value |= (uint32_t)data[2] << 16U;
    value |= (uint32_t)data[3] << 24U;

    return (int32_t)value;
}

/**
 * @brief 将已经准备好的负载封装成完整协议帧。
 */
static ApexProtocolStatus_t ApexProtocol_EncodeFrame(ApexMessageType_t message_type, uint16_t sequence, const uint8_t *payload, uint8_t payload_length, uint8_t *frame, uint8_t capacity, uint8_t *frame_length)
{
    uint8_t required_length;
    uint16_t crc;

    if ((payload == NULL) || (frame == NULL) || (frame_length == NULL))
    {
        return APEX_PROTOCOL_INVALID_PARAM;
    }

    if (payload_length > APEX_PROTOCOL_MAX_PAYLOAD_LENGTH)
    {
        return APEX_PROTOCOL_BAD_LENGTH;
    }

    required_length = (uint8_t)(APEX_FRAME_PAYLOAD_OFFSET + payload_length + 2U);

    if (capacity < required_length)
    {
        return APEX_PROTOCOL_BUFFER_TOO_SMALL;
    }

    frame[0] = APEX_PROTOCOL_HEADER_0;
    frame[1] = APEX_PROTOCOL_HEADER_1;
    frame[APEX_FRAME_VERSION_OFFSET] = APEX_PROTOCOL_VERSION;
    frame[APEX_FRAME_TYPE_OFFSET] = (uint8_t)message_type;
    ApexProtocol_WriteU16(&frame[APEX_FRAME_SEQUENCE_OFFSET], sequence);
    frame[APEX_FRAME_LENGTH_OFFSET] = payload_length;
    memcpy(&frame[APEX_FRAME_PAYLOAD_OFFSET], payload, payload_length);
    crc = ApexProtocol_Crc16(&frame[APEX_FRAME_VERSION_OFFSET], (uint16_t)(5U + payload_length));
    ApexProtocol_WriteU16(&frame[APEX_FRAME_PAYLOAD_OFFSET + payload_length], crc);
    *frame_length = required_length;

    return APEX_PROTOCOL_OK;
}

uint16_t ApexProtocol_Crc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc;
    uint16_t index;
    uint8_t bit;

    if (data == NULL)
    {
        return 0U;
    }

    crc = 0xFFFFU;

    for (index = 0U; index < length; index++)
    {
        crc ^= (uint16_t)data[index] << 8U;

        for (bit = 0U; bit < 8U; bit++)
        {
            if ((crc & 0x8000U) != 0U)
            {
                crc = (uint16_t)((crc << 1U) ^ 0x1021U);
            }
            else
            {
                crc <<= 1U;
            }
        }
    }

    return crc;
}

ApexProtocolStatus_t ApexProtocol_EncodeControl(const ApexControlCommand_t *command, uint8_t *frame, uint8_t capacity, uint8_t *frame_length)
{
    uint8_t payload[APEX_PROTOCOL_CONTROL_PAYLOAD_LENGTH];

    if (command == NULL)
    {
        return APEX_PROTOCOL_INVALID_PARAM;
    }

    ApexProtocol_WriteU16(&payload[0], command->throttle);
    ApexProtocol_WriteI16(&payload[2], command->roll);
    ApexProtocol_WriteI16(&payload[4], command->pitch);
    ApexProtocol_WriteI16(&payload[6], command->yaw);
    payload[8] = command->flags;
    payload[9] = command->flight_mode;

    return ApexProtocol_EncodeFrame(APEX_MESSAGE_CONTROL, command->sequence, payload, sizeof(payload), frame, capacity, frame_length);
}

ApexProtocolStatus_t ApexProtocol_EncodeTelemetry(const ApexTelemetry_t *telemetry, uint8_t *frame, uint8_t capacity, uint8_t *frame_length)
{
    uint8_t payload[APEX_PROTOCOL_TELEMETRY_PAYLOAD_LENGTH];

    if (telemetry == NULL)
    {
        return APEX_PROTOCOL_INVALID_PARAM;
    }

    ApexProtocol_WriteI16(&payload[0], telemetry->roll_cdeg);
    ApexProtocol_WriteI16(&payload[2], telemetry->pitch_cdeg);
    ApexProtocol_WriteI16(&payload[4], telemetry->yaw_cdeg);
    ApexProtocol_WriteI32(&payload[6], telemetry->altitude_cm);
    ApexProtocol_WriteI16(&payload[10], telemetry->vertical_speed_cms);
    ApexProtocol_WriteU16(&payload[12], telemetry->ground_speed_cms);
    ApexProtocol_WriteU16(&payload[14], telemetry->battery_mv);
    ApexProtocol_WriteU16(&payload[16], telemetry->home_distance_m);
    ApexProtocol_WriteI16(&payload[18], telemetry->rssi_dbm);
    payload[20] = (uint8_t)telemetry->snr_q4;
    payload[21] = telemetry->satellites;
    payload[22] = telemetry->link_quality;
    payload[23] = telemetry->flags;
    payload[24] = telemetry->flight_mode;
    ApexProtocol_WriteU16(&payload[25], telemetry->throttle_pwm_us);
    /* Unix UTC 秒固定占四个小端字节，协议字段与飞机端保持完全一致。 */
    ApexProtocol_WriteI32(&payload[27], (int32_t)telemetry->gps_utc_seconds);

    return ApexProtocol_EncodeFrame(APEX_MESSAGE_TELEMETRY, telemetry->sequence, payload, sizeof(payload), frame, capacity, frame_length);
}

ApexProtocolStatus_t ApexProtocol_EncodePidCommand(const ApexPidCommand_t *command, uint8_t *frame, uint8_t capacity, uint8_t *frame_length)
{
    uint8_t payload[APEX_PROTOCOL_PID_COMMAND_PAYLOAD_LENGTH];
    if (command == NULL) return APEX_PROTOCOL_INVALID_PARAM;
    payload[0] = (uint8_t)command->operation;
    payload[1] = command->parameter_id;
    ApexProtocol_WriteI32(&payload[2], command->value_q10000);
    return ApexProtocol_EncodeFrame(APEX_MESSAGE_PID_COMMAND, command->sequence, payload, sizeof(payload), frame, capacity, frame_length);
}

ApexProtocolStatus_t ApexProtocol_EncodePidResponse(const ApexPidResponse_t *response, uint8_t *frame, uint8_t capacity, uint8_t *frame_length)
{
    uint8_t payload[APEX_PROTOCOL_PID_RESPONSE_PAYLOAD_LENGTH];
    uint8_t index;
    if (response == NULL) return APEX_PROTOCOL_INVALID_PARAM;
    payload[0] = (uint8_t)response->status;
    payload[1] = (uint8_t)response->operation;
    ApexProtocol_WriteU16(&payload[2], response->parameter_version);
    for (index = 0U; index < APEX_PID_PARAMETER_COUNT; index++) ApexProtocol_WriteI32(&payload[4U + ((uint16_t)index * 4U)], response->values_q10000[index]);
    return ApexProtocol_EncodeFrame(APEX_MESSAGE_PID_RESPONSE, response->sequence, payload, sizeof(payload), frame, capacity, frame_length);
}

ApexProtocolStatus_t ApexProtocol_DecodeFrame(const uint8_t *frame, uint8_t frame_length, ApexFrame_t *decoded_frame)
{
    uint8_t payload_length;
    uint8_t expected_length;
    uint16_t received_crc;
    uint16_t calculated_crc;

    if ((frame == NULL) || (decoded_frame == NULL))
    {
        return APEX_PROTOCOL_INVALID_PARAM;
    }

    if (frame_length < 9U)
    {
        return APEX_PROTOCOL_BAD_LENGTH;
    }

    if ((frame[0] != APEX_PROTOCOL_HEADER_0) || (frame[1] != APEX_PROTOCOL_HEADER_1))
    {
        return APEX_PROTOCOL_BAD_HEADER;
    }

    if (frame[APEX_FRAME_VERSION_OFFSET] != APEX_PROTOCOL_VERSION)
    {
        return APEX_PROTOCOL_BAD_VERSION;
    }

    payload_length = frame[APEX_FRAME_LENGTH_OFFSET];

    if (payload_length > APEX_PROTOCOL_MAX_PAYLOAD_LENGTH)
    {
        return APEX_PROTOCOL_BAD_LENGTH;
    }

    expected_length = (uint8_t)(APEX_FRAME_PAYLOAD_OFFSET + payload_length + 2U);

    if (frame_length != expected_length)
    {
        return APEX_PROTOCOL_BAD_LENGTH;
    }

    received_crc = ApexProtocol_ReadU16(&frame[APEX_FRAME_PAYLOAD_OFFSET + payload_length]);
    calculated_crc = ApexProtocol_Crc16(&frame[APEX_FRAME_VERSION_OFFSET], (uint16_t)(5U + payload_length));

    if (received_crc != calculated_crc)
    {
        return APEX_PROTOCOL_BAD_CRC;
    }

    decoded_frame->message_type = (ApexMessageType_t)frame[APEX_FRAME_TYPE_OFFSET];
    decoded_frame->sequence = ApexProtocol_ReadU16(&frame[APEX_FRAME_SEQUENCE_OFFSET]);
    decoded_frame->payload_length = payload_length;
    memcpy(decoded_frame->payload, &frame[APEX_FRAME_PAYLOAD_OFFSET], payload_length);

    return APEX_PROTOCOL_OK;
}

ApexProtocolStatus_t ApexProtocol_DecodeControl(const ApexFrame_t *frame, ApexControlCommand_t *command)
{
    if ((frame == NULL) || (command == NULL))
    {
        return APEX_PROTOCOL_INVALID_PARAM;
    }

    if (frame->message_type != APEX_MESSAGE_CONTROL)
    {
        return APEX_PROTOCOL_BAD_MESSAGE_TYPE;
    }

    if (frame->payload_length != APEX_PROTOCOL_CONTROL_PAYLOAD_LENGTH)
    {
        return APEX_PROTOCOL_BAD_LENGTH;
    }

    command->sequence = frame->sequence;
    command->throttle = ApexProtocol_ReadU16(&frame->payload[0]);
    command->roll = ApexProtocol_ReadI16(&frame->payload[2]);
    command->pitch = ApexProtocol_ReadI16(&frame->payload[4]);
    command->yaw = ApexProtocol_ReadI16(&frame->payload[6]);
    command->flags = frame->payload[8];
    command->flight_mode = frame->payload[9];

    return APEX_PROTOCOL_OK;
}

ApexProtocolStatus_t ApexProtocol_DecodeTelemetry(const ApexFrame_t *frame, ApexTelemetry_t *telemetry)
{
    if ((frame == NULL) || (telemetry == NULL))
    {
        return APEX_PROTOCOL_INVALID_PARAM;
    }

    if (frame->message_type != APEX_MESSAGE_TELEMETRY)
    {
        return APEX_PROTOCOL_BAD_MESSAGE_TYPE;
    }

    if (frame->payload_length != APEX_PROTOCOL_TELEMETRY_PAYLOAD_LENGTH)
    {
        return APEX_PROTOCOL_BAD_LENGTH;
    }

    telemetry->sequence = frame->sequence;
    telemetry->roll_cdeg = ApexProtocol_ReadI16(&frame->payload[0]);
    telemetry->pitch_cdeg = ApexProtocol_ReadI16(&frame->payload[2]);
    telemetry->yaw_cdeg = ApexProtocol_ReadI16(&frame->payload[4]);
    telemetry->altitude_cm = ApexProtocol_ReadI32(&frame->payload[6]);
    telemetry->vertical_speed_cms = ApexProtocol_ReadI16(&frame->payload[10]);
    telemetry->ground_speed_cms = ApexProtocol_ReadU16(&frame->payload[12]);
    telemetry->battery_mv = ApexProtocol_ReadU16(&frame->payload[14]);
    telemetry->home_distance_m = ApexProtocol_ReadU16(&frame->payload[16]);
    telemetry->rssi_dbm = ApexProtocol_ReadI16(&frame->payload[18]);
    telemetry->snr_q4 = (int8_t)frame->payload[20];
    telemetry->satellites = frame->payload[21];
    telemetry->link_quality = frame->payload[22];
    telemetry->flags = frame->payload[23];
    telemetry->flight_mode = frame->payload[24];
    telemetry->throttle_pwm_us = ApexProtocol_ReadU16(&frame->payload[25]);
    telemetry->gps_utc_seconds = (uint32_t)ApexProtocol_ReadI32(&frame->payload[27]);

    return APEX_PROTOCOL_OK;
}

ApexProtocolStatus_t ApexProtocol_DecodePidCommand(const ApexFrame_t *frame, ApexPidCommand_t *command)
{
    if ((frame == NULL) || (command == NULL)) return APEX_PROTOCOL_INVALID_PARAM;
    if (frame->message_type != APEX_MESSAGE_PID_COMMAND) return APEX_PROTOCOL_BAD_MESSAGE_TYPE;
    if (frame->payload_length != APEX_PROTOCOL_PID_COMMAND_PAYLOAD_LENGTH) return APEX_PROTOCOL_BAD_LENGTH;
    command->sequence = frame->sequence;
    command->operation = (ApexPidOperation_t)frame->payload[0];
    command->parameter_id = frame->payload[1];
    command->value_q10000 = ApexProtocol_ReadI32(&frame->payload[2]);
    return APEX_PROTOCOL_OK;
}

ApexProtocolStatus_t ApexProtocol_DecodePidResponse(const ApexFrame_t *frame, ApexPidResponse_t *response)
{
    uint8_t index;
    if ((frame == NULL) || (response == NULL)) return APEX_PROTOCOL_INVALID_PARAM;
    if (frame->message_type != APEX_MESSAGE_PID_RESPONSE) return APEX_PROTOCOL_BAD_MESSAGE_TYPE;
    if (frame->payload_length != APEX_PROTOCOL_PID_RESPONSE_PAYLOAD_LENGTH) return APEX_PROTOCOL_BAD_LENGTH;
    response->sequence = frame->sequence;
    response->status = (ApexPidStatus_t)frame->payload[0];
    response->operation = (ApexPidOperation_t)frame->payload[1];
    response->parameter_version = ApexProtocol_ReadU16(&frame->payload[2]);
    for (index = 0U; index < APEX_PID_PARAMETER_COUNT; index++) response->values_q10000[index] = ApexProtocol_ReadI32(&frame->payload[4U + ((uint16_t)index * 4U)]);
    return APEX_PROTOCOL_OK;
}
