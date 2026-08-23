/**
 * @file radio.c
 * @brief 飞机端 RA-01SC 收发任务。
 *
 * 遥控器约 100ms 发一包控制帧；飞机解码后写入 control_link，立刻回遥测。
 * 收到 PID 命令则回整张参数表，不写 TIM8。
 * DIO1 中断只通知本任务，SPI 读写都在任务上下文完成。
 */

#include "radio.h"

#include "apex_protocol.h"
#include "battery.h"
#include "blackbox.h"
#include "control_link.h"
#include "debug.h"
#include "pid_table.h"
#include "FreeRTOS.h"
#include "main.h"
#include "ra01sc.h"
#include "spi.h"
#include "task.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define RADIO_TASK_RETRY_DELAY_MS 1000U
#define RADIO_TX_GUARD_TIMEOUT_MS 4000U
#define RADIO_NOTIFY_DIO1 0x00000001UL

typedef struct
{
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    uint16_t throttle_pwm_us;
    bool imu_ready;
    bool motor_ready;
    bool armed;
} RadioFlightState_t;

static TaskHandle_t radio_task_handle;
static RA01SC_Device_t radio_device;
static RA01SC_Hardware_t radio_hardware;
static RA01SC_Config_t radio_config;
static uint8_t radio_tx_frame[APEX_PROTOCOL_MAX_FRAME_LENGTH];
static uint8_t radio_rx_frame[APEX_PROTOCOL_MAX_FRAME_LENGTH];
static RadioFlightState_t radio_flight_state;

static int16_t Radio_ClampI16(int32_t value, int16_t minimum, int16_t maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return (int16_t)value;
}

static int16_t Radio_DegToCdeg(float degrees)
{
    return Radio_ClampI16((int32_t)(degrees * 100.0f), -18000, 18000);
}

static void Radio_ConfigureHardware(void)
{
    radio_hardware.spi = &hspi1;
    radio_hardware.nss_port = LORA_NSS_GPIO_Port;
    radio_hardware.nss_pin = LORA_NSS_Pin;
    radio_hardware.reset_port = LORA_NRST_GPIO_Port;
    radio_hardware.reset_pin = LORA_NRST_Pin;
    radio_hardware.busy_port = LORA_BUSY_GPIO_Port;
    radio_hardware.busy_pin = LORA_BUSY_Pin;
    radio_hardware.dio1_port = LORA_DIO1_GPIO_Port;
    radio_hardware.dio1_pin = LORA_DIO1_Pin;
    radio_hardware.txen_port = LORA_TXEN_GPIO_Port;
    radio_hardware.txen_pin = LORA_TXEN_Pin;
    radio_hardware.rxen_port = LORA_RXEN_GPIO_Port;
    radio_hardware.rxen_pin = LORA_RXEN_Pin;
}

static void Radio_CopyFlightState(RadioFlightState_t *state)
{
    taskENTER_CRITICAL();
    *state = radio_flight_state;
    taskEXIT_CRITICAL();
}

void Radio_PublishFlightState(float roll_deg, float pitch_deg, float yaw_deg, uint16_t throttle_pwm_us, bool imu_ready, bool motor_ready, bool armed)
{
    taskENTER_CRITICAL();
    radio_flight_state.roll_deg = roll_deg;
    radio_flight_state.pitch_deg = pitch_deg;
    radio_flight_state.yaw_deg = yaw_deg;
    radio_flight_state.throttle_pwm_us = throttle_pwm_us;
    radio_flight_state.imu_ready = imu_ready;
    radio_flight_state.motor_ready = motor_ready;
    radio_flight_state.armed = armed;
    taskEXIT_CRITICAL();
}

static RA01SC_Status_t Radio_Initialize(void)
{
    RA01SC_Status_t status;

    ControlLink_Init();
    Radio_ConfigureHardware();
    RA01SC_GetDefaultConfig(&radio_config);
    status = RA01SC_Init(&radio_device, &radio_hardware, &radio_config);
    if (status != RA01SC_STATUS_OK)
    {
        Debug_Printf("[RADIO] init failed status=%u.\r\n", (unsigned int)status);
        return status;
    }

    status = RA01SC_StartReceive(&radio_device);
    if (status == RA01SC_STATUS_OK) Debug_Printf("[RADIO] rx ready.\r\n");
    else Debug_Printf("[RADIO] start rx failed status=%u.\r\n", (unsigned int)status);
    return status;
}

static RA01SC_Status_t Radio_SendTelemetry(uint16_t sequence, uint8_t flight_mode)
{
    ApexTelemetry_t telemetry;
    ApexProtocolStatus_t protocol_status;
    RadioFlightState_t flight_state;
    uint8_t frame_length;

    Radio_CopyFlightState(&flight_state);
    memset(&telemetry, 0, sizeof(telemetry));
    telemetry.sequence = sequence;
    telemetry.roll_cdeg = Radio_DegToCdeg(flight_state.roll_deg);
    telemetry.pitch_cdeg = Radio_DegToCdeg(flight_state.pitch_deg);
    telemetry.yaw_cdeg = Radio_DegToCdeg(flight_state.yaw_deg);
    telemetry.throttle_pwm_us = flight_state.throttle_pwm_us;
    telemetry.battery_mv = Battery_GetMillivolts();
    telemetry.flight_mode = flight_mode;
    if (flight_state.armed) telemetry.flags |= APEX_TELEMETRY_FLAG_ARMED;
    if (flight_state.imu_ready) telemetry.flags |= APEX_TELEMETRY_FLAG_IMU_READY;
    if (flight_state.motor_ready) telemetry.flags |= APEX_TELEMETRY_FLAG_MOTOR_READY;
    if (Blackbox_IsRecording()) telemetry.flags |= APEX_TELEMETRY_FLAG_BLACKBOX_RECORDING;

    protocol_status = ApexProtocol_EncodeTelemetry(&telemetry, radio_tx_frame, sizeof(radio_tx_frame), &frame_length);
    if (protocol_status != APEX_PROTOCOL_OK) return RA01SC_STATUS_INVALID_PARAM;
    return RA01SC_Send(&radio_device, radio_tx_frame, frame_length);
}

static RA01SC_Status_t Radio_SendPidResponse(const ApexPidCommand_t *command)
{
    ApexPidResponse_t response;
    ApexProtocolStatus_t protocol_status;
    RadioFlightState_t flight_state;
    uint8_t frame_length;

    Radio_CopyFlightState(&flight_state);
    PidTable_HandleCommand(command, flight_state.armed, &response);
    protocol_status = ApexProtocol_EncodePidResponse(&response, radio_tx_frame, sizeof(radio_tx_frame), &frame_length);
    if (protocol_status != APEX_PROTOCOL_OK) return RA01SC_STATUS_INVALID_PARAM;
    return RA01SC_Send(&radio_device, radio_tx_frame, frame_length);
}

static RA01SC_Status_t Radio_ProcessReceivedPacket(bool *tx_in_progress)
{
    RA01SC_Status_t radio_status;
    ApexProtocolStatus_t protocol_status;
    ApexFrame_t decoded_frame;
    ApexControlCommand_t command;
    ApexPidCommand_t pid_command;
    uint8_t received_length;
    uint32_t current_tick;

    received_length = 0U;
    radio_status = RA01SC_ReadPacket(&radio_device, radio_rx_frame, sizeof(radio_rx_frame), &received_length);
    if (radio_status != RA01SC_STATUS_OK) return radio_status;

    protocol_status = ApexProtocol_DecodeFrame(radio_rx_frame, received_length, &decoded_frame);
    if (protocol_status != APEX_PROTOCOL_OK)
    {
        ControlLink_RecordRejectedFrame();
        return RA01SC_StartReceive(&radio_device);
    }

    if (decoded_frame.message_type == APEX_MESSAGE_PID_COMMAND)
    {
        protocol_status = ApexProtocol_DecodePidCommand(&decoded_frame, &pid_command);
        if (protocol_status != APEX_PROTOCOL_OK)
        {
            ControlLink_RecordRejectedFrame();
            Debug_Printf("[RADIO] pid cmd decode fail %u.\r\n", (unsigned int)protocol_status);
            return RA01SC_StartReceive(&radio_device);
        }
        Debug_Printf("[RADIO] pid cmd op=%u id=%u seq=%u.\r\n", (unsigned int)pid_command.operation, (unsigned int)pid_command.parameter_id, (unsigned int)pid_command.sequence);
        radio_status = Radio_SendPidResponse(&pid_command);
        if (radio_status != RA01SC_STATUS_OK)
        {
            Debug_Printf("[RADIO] pid rsp send fail %u.\r\n", (unsigned int)radio_status);
            return radio_status;
        }
        Debug_Printf("[RADIO] pid rsp sent.\r\n");
        *tx_in_progress = true;
        return RA01SC_STATUS_OK;
    }

    if (decoded_frame.message_type != APEX_MESSAGE_CONTROL)
    {
        return RA01SC_StartReceive(&radio_device);
    }

    protocol_status = ApexProtocol_DecodeControl(&decoded_frame, &command);
    if (protocol_status != APEX_PROTOCOL_OK)
    {
        ControlLink_RecordRejectedFrame();
        return RA01SC_StartReceive(&radio_device);
    }

    current_tick = (uint32_t)xTaskGetTickCount();
    if (!ControlLink_AcceptCommand(&command, current_tick))
    {
        return RA01SC_StartReceive(&radio_device);
    }

    radio_status = Radio_SendTelemetry(command.sequence, command.flight_mode);
    if (radio_status != RA01SC_STATUS_OK) return radio_status;
    *tx_in_progress = true;
    return RA01SC_STATUS_OK;
}

static RA01SC_Status_t Radio_ProcessDio1(bool *tx_in_progress)
{
    RA01SC_Status_t status;
    uint16_t irq_status;

    status = RA01SC_GetAndClearIrqStatus(&radio_device, &irq_status);
    if (status != RA01SC_STATUS_OK) return status;

    if ((irq_status & (RA01SC_IRQ_HEADER_ERROR | RA01SC_IRQ_CRC_ERROR)) != 0U)
    {
        ControlLink_RecordRejectedFrame();
        *tx_in_progress = false;
        return RA01SC_StartReceive(&radio_device);
    }

    if ((irq_status & RA01SC_IRQ_RX_DONE) != 0U)
    {
        status = Radio_ProcessReceivedPacket(tx_in_progress);
        if (status != RA01SC_STATUS_OK) return status;
    }

    if ((irq_status & RA01SC_IRQ_TX_DONE) != 0U)
    {
        *tx_in_progress = false;
        status = RA01SC_StartReceive(&radio_device);
        if (status != RA01SC_STATUS_OK) return status;
    }

    if ((irq_status & RA01SC_IRQ_TIMEOUT) != 0U)
    {
        *tx_in_progress = false;
        return RA01SC_StartReceive(&radio_device);
    }

    return RA01SC_STATUS_OK;
}

void Radio_Task(void *argument)
{
    RA01SC_Status_t status;
    TickType_t current_tick;
    TickType_t tx_start_tick;
    TickType_t wait_ticks;
    uint32_t notification_value;
    bool tx_in_progress;
    bool was_transmitting;

    (void)argument;
    Debug_Init();
    PidTable_Init();
    radio_task_handle = xTaskGetCurrentTaskHandle();
    memset(&radio_flight_state, 0, sizeof(radio_flight_state));

    for (;;)
    {
        status = Radio_Initialize();
        if (status != RA01SC_STATUS_OK)
        {
            vTaskDelay(pdMS_TO_TICKS(RADIO_TASK_RETRY_DELAY_MS));
            continue;
        }

        tx_in_progress = false;
        tx_start_tick = 0U;

        for (;;)
        {
            current_tick = xTaskGetTickCount();
            if (tx_in_progress && ((current_tick - tx_start_tick) > pdMS_TO_TICKS(RADIO_TX_GUARD_TIMEOUT_MS)))
            {
                Debug_Printf("[RADIO] tx timeout, reinit.\r\n");
                break;
            }

            wait_ticks = tx_in_progress ? pdMS_TO_TICKS(20U) : pdMS_TO_TICKS(1000U);
            notification_value = 0U;
            (void)xTaskNotifyWait(0U, 0xFFFFFFFFUL, &notification_value, wait_ticks);

            if ((notification_value & RADIO_NOTIFY_DIO1) != 0U)
            {
                was_transmitting = tx_in_progress;
                status = Radio_ProcessDio1(&tx_in_progress);
                if (status != RA01SC_STATUS_OK)
                {
                    Debug_Printf("[RADIO] irq failed status=%u, reinit.\r\n", (unsigned int)status);
                    break;
                }
                /* 刚启动遥测发送时记下时刻，超时未等到 TX_DONE 就复位模块。 */
                if (tx_in_progress && !was_transmitting) tx_start_tick = xTaskGetTickCount();
            }
        }

        vTaskDelay(pdMS_TO_TICKS(RADIO_TASK_RETRY_DELAY_MS));
    }
}

/**
 * @brief STM32 HAL GPIO 外部中断统一回调。
 *
 * 只处理 LoRa DIO1。IMU_INT 当前没有任务在用，这里忽略。
 * 中断里只通知电台任务，禁止在这里做 SPI。
 */
void HAL_GPIO_EXTI_Callback(uint16_t gpio_pin)
{
    BaseType_t higher_priority_task_woken;

    if ((gpio_pin != LORA_DIO1_Pin) || (radio_task_handle == NULL)) return;

    higher_priority_task_woken = pdFALSE;
    xTaskNotifyFromISR(radio_task_handle, RADIO_NOTIFY_DIO1, eSetBits, &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
}
