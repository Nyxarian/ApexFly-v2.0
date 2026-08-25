/**
 * @file pid_table.c
 * @brief RAM 参数表实现：30 个协议 ID 与姿态串级、满杆油门上限互转。
 *
 * 增益和限幅改完后只刷新 Pid_t 的数字并 Reset 积分，不重新 Init 整轴以外的状态机。
 * trim 先存着给 GET/SET 显示，不送进控制目标角。
 * APEX_PID_HOVER_PWM_US 表示满杆油门上限；垂速 PID 项仍可读写，但不再参与电机输出。
 */

#include "pid_table.h"

#include "attitude_control.h"
#include "altitude_control.h"
#include "blackbox.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stddef.h>
#include <string.h>

#define PID_TABLE_PARAMETER_VERSION 2U
#define PID_TABLE_GAIN_MAX 50.0f
#define PID_TABLE_VZ_KP_MAX 500.0f
#define PID_TABLE_VZ_KI_MAX 100.0f
#define PID_TABLE_VZ_KD_MAX 50.0f
#define PID_TABLE_MAX_THROTTLE_MIN_US 1200.0f
#define PID_TABLE_MAX_THROTTLE_MAX_US 2000.0f
#define PID_TABLE_RATE_MIN 20.0f
#define PID_TABLE_RATE_MAX 300.0f
#define PID_TABLE_CORR_MIN 20.0f
#define PID_TABLE_CORR_MAX 300.0f
#define PID_TABLE_TRIM_ABS_MAX 10.0f

typedef struct
{
    AttitudePidParams_t pid;
    AltitudeControlParams_t altitude;
    float roll_trim_deg;
    float pitch_trim_deg;
} PidTableState_t;

static PidTableState_t pid_table;
static bool pid_table_ready;

static int32_t PidTable_FloatToQ10000(float value)
{
    if (value >= 0.0f) return (int32_t)(value * (float)APEX_PID_VALUE_SCALE + 0.5f);
    return (int32_t)(value * (float)APEX_PID_VALUE_SCALE - 0.5f);
}

static float PidTable_Q10000ToFloat(int32_t value_q10000)
{
    return (float)value_q10000 / (float)APEX_PID_VALUE_SCALE;
}

static float PidTable_Absf(float value)
{
    if (value < 0.0f) return -value;
    return value;
}

static void PidTable_ExportValues(int32_t *values_q10000)
{
    const AttitudePidParams_t *pid = &pid_table.pid;

    if (values_q10000 == NULL) return;

    values_q10000[APEX_PID_ROLL_OUTER_KP] = PidTable_FloatToQ10000(pid->roll_angle.kp);
    values_q10000[APEX_PID_ROLL_OUTER_KI] = PidTable_FloatToQ10000(pid->roll_angle.ki);
    values_q10000[APEX_PID_ROLL_OUTER_KD] = PidTable_FloatToQ10000(pid->roll_angle.kd);
    values_q10000[APEX_PID_ROLL_INNER_KP] = PidTable_FloatToQ10000(pid->roll_rate.kp);
    values_q10000[APEX_PID_ROLL_INNER_KI] = PidTable_FloatToQ10000(pid->roll_rate.ki);
    values_q10000[APEX_PID_ROLL_INNER_KD] = PidTable_FloatToQ10000(pid->roll_rate.kd);
    values_q10000[APEX_PID_PITCH_OUTER_KP] = PidTable_FloatToQ10000(pid->pitch_angle.kp);
    values_q10000[APEX_PID_PITCH_OUTER_KI] = PidTable_FloatToQ10000(pid->pitch_angle.ki);
    values_q10000[APEX_PID_PITCH_OUTER_KD] = PidTable_FloatToQ10000(pid->pitch_angle.kd);
    values_q10000[APEX_PID_PITCH_INNER_KP] = PidTable_FloatToQ10000(pid->pitch_rate.kp);
    values_q10000[APEX_PID_PITCH_INNER_KI] = PidTable_FloatToQ10000(pid->pitch_rate.ki);
    values_q10000[APEX_PID_PITCH_INNER_KD] = PidTable_FloatToQ10000(pid->pitch_rate.kd);
    values_q10000[APEX_PID_YAW_RATE_KP] = PidTable_FloatToQ10000(pid->yaw_rate.kp);
    values_q10000[APEX_PID_YAW_RATE_KI] = PidTable_FloatToQ10000(pid->yaw_rate.ki);
    values_q10000[APEX_PID_YAW_RATE_KD] = PidTable_FloatToQ10000(pid->yaw_rate.kd);
    values_q10000[APEX_PID_ROLL_MAX_RATE_DPS] = PidTable_FloatToQ10000(pid->roll_angle.out_max);
    values_q10000[APEX_PID_ROLL_MAX_CORRECTION_US] = PidTable_FloatToQ10000(pid->roll_rate.out_max);
    values_q10000[APEX_PID_PITCH_MAX_RATE_DPS] = PidTable_FloatToQ10000(pid->pitch_angle.out_max);
    values_q10000[APEX_PID_PITCH_MAX_CORRECTION_US] = PidTable_FloatToQ10000(pid->pitch_rate.out_max);
    values_q10000[APEX_PID_YAW_MAX_RATE_DPS] = PidTable_FloatToQ10000(pid->yaw_angle.out_max);
    values_q10000[APEX_PID_YAW_MAX_CORRECTION_US] = PidTable_FloatToQ10000(pid->yaw_rate.out_max);
    values_q10000[APEX_PID_ROLL_TRIM_DEG] = PidTable_FloatToQ10000(pid_table.roll_trim_deg);
    values_q10000[APEX_PID_PITCH_TRIM_DEG] = PidTable_FloatToQ10000(pid_table.pitch_trim_deg);
    values_q10000[APEX_PID_YAW_ANGLE_KP] = PidTable_FloatToQ10000(pid->yaw_angle.kp);
    values_q10000[APEX_PID_YAW_ANGLE_KI] = PidTable_FloatToQ10000(pid->yaw_angle.ki);
    values_q10000[APEX_PID_YAW_ANGLE_KD] = PidTable_FloatToQ10000(pid->yaw_angle.kd);
    values_q10000[APEX_PID_VERTICAL_SPEED_KP] = PidTable_FloatToQ10000(pid_table.altitude.vz_kp);
    values_q10000[APEX_PID_VERTICAL_SPEED_KI] = PidTable_FloatToQ10000(pid_table.altitude.vz_ki);
    values_q10000[APEX_PID_VERTICAL_SPEED_KD] = PidTable_FloatToQ10000(pid_table.altitude.vz_kd);
    values_q10000[APEX_PID_HOVER_PWM_US] = PidTable_FloatToQ10000(pid_table.altitude.hover_pwm_us);
}

static ApexPidStatus_t PidTable_SetValue(uint8_t parameter_id, float value)
{
    AttitudePidParams_t *pid = &pid_table.pid;

    switch (parameter_id)
    {
    case APEX_PID_ROLL_OUTER_KP:
    case APEX_PID_ROLL_OUTER_KI:
    case APEX_PID_ROLL_OUTER_KD:
    case APEX_PID_ROLL_INNER_KP:
    case APEX_PID_ROLL_INNER_KI:
    case APEX_PID_ROLL_INNER_KD:
    case APEX_PID_PITCH_OUTER_KP:
    case APEX_PID_PITCH_OUTER_KI:
    case APEX_PID_PITCH_OUTER_KD:
    case APEX_PID_PITCH_INNER_KP:
    case APEX_PID_PITCH_INNER_KI:
    case APEX_PID_PITCH_INNER_KD:
    case APEX_PID_YAW_RATE_KP:
    case APEX_PID_YAW_RATE_KI:
    case APEX_PID_YAW_RATE_KD:
    case APEX_PID_YAW_ANGLE_KP:
    case APEX_PID_YAW_ANGLE_KI:
    case APEX_PID_YAW_ANGLE_KD:
        if ((value < 0.0f) || (value > PID_TABLE_GAIN_MAX)) return APEX_PID_STATUS_OUT_OF_RANGE;
        break;
    case APEX_PID_ROLL_MAX_RATE_DPS:
    case APEX_PID_PITCH_MAX_RATE_DPS:
    case APEX_PID_YAW_MAX_RATE_DPS:
        if ((value < PID_TABLE_RATE_MIN) || (value > PID_TABLE_RATE_MAX)) return APEX_PID_STATUS_OUT_OF_RANGE;
        break;
    case APEX_PID_ROLL_MAX_CORRECTION_US:
    case APEX_PID_PITCH_MAX_CORRECTION_US:
    case APEX_PID_YAW_MAX_CORRECTION_US:
        if ((value < PID_TABLE_CORR_MIN) || (value > PID_TABLE_CORR_MAX)) return APEX_PID_STATUS_OUT_OF_RANGE;
        break;
    case APEX_PID_ROLL_TRIM_DEG:
    case APEX_PID_PITCH_TRIM_DEG:
        if (PidTable_Absf(value) > PID_TABLE_TRIM_ABS_MAX) return APEX_PID_STATUS_OUT_OF_RANGE;
        break;
    case APEX_PID_VERTICAL_SPEED_KP:
        if ((value < 0.0f) || (value > PID_TABLE_VZ_KP_MAX)) return APEX_PID_STATUS_OUT_OF_RANGE;
        break;
    case APEX_PID_VERTICAL_SPEED_KI:
        if ((value < 0.0f) || (value > PID_TABLE_VZ_KI_MAX)) return APEX_PID_STATUS_OUT_OF_RANGE;
        break;
    case APEX_PID_VERTICAL_SPEED_KD:
        if ((value < 0.0f) || (value > PID_TABLE_VZ_KD_MAX)) return APEX_PID_STATUS_OUT_OF_RANGE;
        break;
    case APEX_PID_HOVER_PWM_US:
        if ((value < PID_TABLE_MAX_THROTTLE_MIN_US) || (value > PID_TABLE_MAX_THROTTLE_MAX_US)) return APEX_PID_STATUS_OUT_OF_RANGE;
        break;
    default:
        return APEX_PID_STATUS_INVALID_PARAMETER;
    }

    switch (parameter_id)
    {
    case APEX_PID_ROLL_OUTER_KP: pid->roll_angle.kp = value; break;
    case APEX_PID_ROLL_OUTER_KI: pid->roll_angle.ki = value; break;
    case APEX_PID_ROLL_OUTER_KD: pid->roll_angle.kd = value; break;
    case APEX_PID_ROLL_INNER_KP: pid->roll_rate.kp = value; break;
    case APEX_PID_ROLL_INNER_KI: pid->roll_rate.ki = value; break;
    case APEX_PID_ROLL_INNER_KD: pid->roll_rate.kd = value; break;
    case APEX_PID_PITCH_OUTER_KP: pid->pitch_angle.kp = value; break;
    case APEX_PID_PITCH_OUTER_KI: pid->pitch_angle.ki = value; break;
    case APEX_PID_PITCH_OUTER_KD: pid->pitch_angle.kd = value; break;
    case APEX_PID_PITCH_INNER_KP: pid->pitch_rate.kp = value; break;
    case APEX_PID_PITCH_INNER_KI: pid->pitch_rate.ki = value; break;
    case APEX_PID_PITCH_INNER_KD: pid->pitch_rate.kd = value; break;
    case APEX_PID_YAW_RATE_KP: pid->yaw_rate.kp = value; break;
    case APEX_PID_YAW_RATE_KI: pid->yaw_rate.ki = value; break;
    case APEX_PID_YAW_RATE_KD: pid->yaw_rate.kd = value; break;
    case APEX_PID_ROLL_MAX_RATE_DPS:
        pid->roll_angle.out_max = value;
        pid->roll_angle.out_min = -value;
        pid->roll_angle.i_limit = value;
        break;
    case APEX_PID_ROLL_MAX_CORRECTION_US:
        pid->roll_rate.out_max = value;
        pid->roll_rate.out_min = -value;
        pid->roll_rate.i_limit = value;
        break;
    case APEX_PID_PITCH_MAX_RATE_DPS:
        pid->pitch_angle.out_max = value;
        pid->pitch_angle.out_min = -value;
        pid->pitch_angle.i_limit = value;
        break;
    case APEX_PID_PITCH_MAX_CORRECTION_US:
        pid->pitch_rate.out_max = value;
        pid->pitch_rate.out_min = -value;
        pid->pitch_rate.i_limit = value;
        break;
    case APEX_PID_YAW_MAX_RATE_DPS:
        pid->yaw_angle.out_max = value;
        pid->yaw_angle.out_min = -value;
        pid->yaw_angle.i_limit = value;
        break;
    case APEX_PID_YAW_MAX_CORRECTION_US:
        pid->yaw_rate.out_max = value;
        pid->yaw_rate.out_min = -value;
        pid->yaw_rate.i_limit = value;
        break;
    case APEX_PID_ROLL_TRIM_DEG: pid_table.roll_trim_deg = value; break;
    case APEX_PID_PITCH_TRIM_DEG: pid_table.pitch_trim_deg = value; break;
    case APEX_PID_YAW_ANGLE_KP: pid->yaw_angle.kp = value; break;
    case APEX_PID_YAW_ANGLE_KI: pid->yaw_angle.ki = value; break;
    case APEX_PID_YAW_ANGLE_KD: pid->yaw_angle.kd = value; break;
    case APEX_PID_VERTICAL_SPEED_KP: pid_table.altitude.vz_kp = value; break;
    case APEX_PID_VERTICAL_SPEED_KI: pid_table.altitude.vz_ki = value; break;
    case APEX_PID_VERTICAL_SPEED_KD: pid_table.altitude.vz_kd = value; break;
    case APEX_PID_HOVER_PWM_US: pid_table.altitude.hover_pwm_us = value; break;
    default:
        return APEX_PID_STATUS_INVALID_PARAMETER;
    }

    return APEX_PID_STATUS_OK;
}

static void PidTable_FillResponse(ApexPidResponse_t *response, uint16_t sequence, ApexPidOperation_t operation, ApexPidStatus_t status)
{
    if (response == NULL) return;
    memset(response, 0, sizeof(*response));
    response->sequence = sequence;
    response->status = status;
    response->operation = operation;
    response->parameter_version = PID_TABLE_PARAMETER_VERSION;
    PidTable_ExportValues(response->values_q10000);
}

void PidTable_ExportSnapshot(int32_t *values_q10000)
{
    taskENTER_CRITICAL();
    PidTable_ExportValues(values_q10000);
    taskEXIT_CRITICAL();
}

bool PidTable_ImportSnapshot(const int32_t *values_q10000)
{
    uint8_t index;
    float value;
    ApexPidStatus_t status;
    PidTableState_t backup;

    if (values_q10000 == NULL) return false;

    taskENTER_CRITICAL();
    backup = pid_table;
    for (index = 0U; index < APEX_PID_PARAMETER_COUNT; index++)
    {
        value = PidTable_Q10000ToFloat(values_q10000[index]);
        status = PidTable_SetValue(index, value);
        if (status != APEX_PID_STATUS_OK)
        {
            pid_table = backup;
            taskEXIT_CRITICAL();
            return false;
        }
    }
    taskEXIT_CRITICAL();
    AttitudeControl_ApplyParams(&pid_table.pid);
    AltitudeControl_ApplyParams(&pid_table.altitude);
    return true;
}

uint16_t PidTable_GetMaxThrottleUs(void)
{
    float max_throttle_us;

    taskENTER_CRITICAL();
    max_throttle_us = pid_table.altitude.hover_pwm_us;
    taskEXIT_CRITICAL();
    if (max_throttle_us < PID_TABLE_MAX_THROTTLE_MIN_US) max_throttle_us = PID_TABLE_MAX_THROTTLE_MIN_US;
    if (max_throttle_us > PID_TABLE_MAX_THROTTLE_MAX_US) max_throttle_us = PID_TABLE_MAX_THROTTLE_MAX_US;
    return (uint16_t)(max_throttle_us + 0.5f);
}

static void PidTable_LoadCompiledDefaults(void)
{
    taskENTER_CRITICAL();
    pid_table.pid = *AttitudeControl_GetDefaultParams();
    pid_table.altitude = *AltitudeControl_GetDefaultParams();
    pid_table.roll_trim_deg = 0.0f;
    pid_table.pitch_trim_deg = 0.0f;
    taskEXIT_CRITICAL();
    AttitudeControl_ApplyParams(&pid_table.pid);
    AltitudeControl_ApplyParams(&pid_table.altitude);
}

void PidTable_Init(void)
{
    taskENTER_CRITICAL();
    if (!pid_table_ready)
    {
        memset(&pid_table, 0, sizeof(pid_table));
        pid_table.pid = *AttitudeControl_GetDefaultParams();
        pid_table.altitude = *AltitudeControl_GetDefaultParams();
        pid_table_ready = true;
        taskEXIT_CRITICAL();
        AttitudeControl_ApplyParams(&pid_table.pid);
        AltitudeControl_ApplyParams(&pid_table.altitude);
        return;
    }
    taskEXIT_CRITICAL();
}

void PidTable_HandleCommand(const ApexPidCommand_t *command, bool armed, ApexPidResponse_t *response)
{
    ApexPidStatus_t status;
    float value;

    if ((command == NULL) || (response == NULL)) return;

    switch (command->operation)
    {
    case APEX_PID_OPERATION_GET:
        taskENTER_CRITICAL();
        PidTable_FillResponse(response, command->sequence, command->operation, APEX_PID_STATUS_OK);
        taskEXIT_CRITICAL();
        return;

    case APEX_PID_OPERATION_SET:
        if (armed)
        {
            PidTable_FillResponse(response, command->sequence, command->operation, APEX_PID_STATUS_LOCKED_REQUIRED);
            return;
        }
        if (command->parameter_id >= APEX_PID_PARAMETER_COUNT)
        {
            PidTable_FillResponse(response, command->sequence, command->operation, APEX_PID_STATUS_INVALID_PARAMETER);
            return;
        }
        value = PidTable_Q10000ToFloat(command->value_q10000);
        taskENTER_CRITICAL();
        status = PidTable_SetValue(command->parameter_id, value);
        taskEXIT_CRITICAL();
        if (status == APEX_PID_STATUS_OK)
        {
            AltitudeControl_ApplyParams(&pid_table.altitude);
            AttitudeControl_ApplyParams(&pid_table.pid);
        }
        taskENTER_CRITICAL();
        PidTable_FillResponse(response, command->sequence, command->operation, status);
        taskEXIT_CRITICAL();
        return;

    case APEX_PID_OPERATION_LOAD_DEFAULTS:
        if (armed)
        {
            PidTable_FillResponse(response, command->sequence, command->operation, APEX_PID_STATUS_LOCKED_REQUIRED);
            return;
        }
        PidTable_LoadCompiledDefaults();
        PidTable_FillResponse(response, command->sequence, command->operation, APEX_PID_STATUS_OK);
        return;

    case APEX_PID_OPERATION_SAVE:
        if (armed)
        {
            PidTable_FillResponse(response, command->sequence, command->operation, APEX_PID_STATUS_LOCKED_REQUIRED);
            return;
        }
        if (!Blackbox_IsReady())
        {
            PidTable_FillResponse(response, command->sequence, command->operation, APEX_PID_STATUS_STORAGE_NOT_READY);
            return;
        }
        if (!Blackbox_RequestSave())
        {
            PidTable_FillResponse(response, command->sequence, command->operation, APEX_PID_STATUS_STORAGE_ERROR);
            return;
        }
        PidTable_FillResponse(response, command->sequence, command->operation, APEX_PID_STATUS_OK);
        return;

    case APEX_PID_OPERATION_LOAD_SAVED:
        if (armed)
        {
            PidTable_FillResponse(response, command->sequence, command->operation, APEX_PID_STATUS_LOCKED_REQUIRED);
            return;
        }
        if (!Blackbox_IsReady())
        {
            PidTable_FillResponse(response, command->sequence, command->operation, APEX_PID_STATUS_STORAGE_NOT_READY);
            return;
        }
        if (!Blackbox_LoadSavedParameters())
        {
            PidTable_FillResponse(response, command->sequence, command->operation, APEX_PID_STATUS_STORAGE_ERROR);
            return;
        }
        PidTable_FillResponse(response, command->sequence, command->operation, APEX_PID_STATUS_OK);
        return;

    case APEX_PID_OPERATION_ACCEL_CAL_SAMPLE:
    case APEX_PID_OPERATION_ACCEL_CAL_FINISH:
    case APEX_PID_OPERATION_ACCEL_CAL_CANCEL:
    default:
        PidTable_FillResponse(response, command->sequence, command->operation, APEX_PID_STATUS_INVALID_OPERATION);
        return;
    }
}
