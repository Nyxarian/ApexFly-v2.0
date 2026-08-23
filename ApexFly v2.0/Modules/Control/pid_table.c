/**
 * @file pid_table.c
 * @brief RAM PID 表实现：26 个协议 ID 与 6 组串级参数互转。
 *
 * 增益和限幅改完后只刷新 Pid_t 的数字并 Reset 积分，不重新 Init 整轴以外的状态机。
 * trim 先存着给 GET/SET 显示，不送进控制目标角。
 */

#include "pid_table.h"

#include "attitude_control.h"
#include "blackbox.h"
#include "debug.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stddef.h>
#include <string.h>

#define PID_TABLE_PARAMETER_VERSION 1U
#define PID_TABLE_GAIN_MAX 50.0f
#define PID_TABLE_RATE_MIN 20.0f
#define PID_TABLE_RATE_MAX 300.0f
#define PID_TABLE_CORR_MIN 20.0f
#define PID_TABLE_CORR_MAX 300.0f
#define PID_TABLE_TRIM_ABS_MAX 10.0f

typedef struct
{
    AttitudePidParams_t pid;
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
    return true;
}

static void PidTable_LoadCompiledDefaults(void)
{
    taskENTER_CRITICAL();
    pid_table.pid = *AttitudeControl_GetDefaultParams();
    pid_table.roll_trim_deg = 0.0f;
    pid_table.pitch_trim_deg = 0.0f;
    taskEXIT_CRITICAL();
    AttitudeControl_ApplyParams(&pid_table.pid);
}

void PidTable_Init(void)
{
    taskENTER_CRITICAL();
    if (!pid_table_ready)
    {
        memset(&pid_table, 0, sizeof(pid_table));
        pid_table.pid = *AttitudeControl_GetDefaultParams();
        pid_table_ready = true;
        taskEXIT_CRITICAL();
        AttitudeControl_ApplyParams(&pid_table.pid);
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
        if (status == APEX_PID_STATUS_OK) AttitudeControl_ApplyParams(&pid_table.pid);
        taskENTER_CRITICAL();
        PidTable_FillResponse(response, command->sequence, command->operation, status);
        taskEXIT_CRITICAL();
        Debug_Printf("[PID] SET id=%u value=%.4f status=%u.\r\n", (unsigned int)command->parameter_id, (double)value, (unsigned int)status);
        return;

    case APEX_PID_OPERATION_LOAD_DEFAULTS:
        if (armed)
        {
            PidTable_FillResponse(response, command->sequence, command->operation, APEX_PID_STATUS_LOCKED_REQUIRED);
            return;
        }
        PidTable_LoadCompiledDefaults();
        PidTable_FillResponse(response, command->sequence, command->operation, APEX_PID_STATUS_OK);
        Debug_Printf("[PID] load defaults.\r\n");
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
        Debug_Printf("[PID] save queued.\r\n");
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
        Debug_Printf("[PID] load saved.\r\n");
        return;

    case APEX_PID_OPERATION_ACCEL_CAL_SAMPLE:
    case APEX_PID_OPERATION_ACCEL_CAL_FINISH:
    case APEX_PID_OPERATION_ACCEL_CAL_CANCEL:
    default:
        PidTable_FillResponse(response, command->sequence, command->operation, APEX_PID_STATUS_INVALID_OPERATION);
        return;
    }
}
