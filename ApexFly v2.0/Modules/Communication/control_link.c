/**
 * @file control_link.c
 * @brief 飞机端遥控指令快照实现。
 *
 * 无线任务调用 AcceptCommand() 写入最新杆量。
 * 电机任务调用 Update() 判断失联，再 GetSnapshot() 取油门。
 * 失联后油门和解锁标志清零，但无线任务仍然不写 PWM。
 */

#include "control_link.h"

#include "FreeRTOS.h"
#include "task.h"
#include <stddef.h>
#include <string.h>

static ControlLinkSnapshot_t control_link_snapshot;
static bool control_link_has_sequence;

/**
 * @brief 用有符号差值判断 16 位序号是否更新，65535 之后的 0 仍算新帧。
 */
static bool ControlLink_IsNewerSequence(uint16_t incoming, uint16_t previous)
{
    return ((int16_t)(incoming - previous) > 0);
}

/**
 * @brief 杆量必须落在协议约定量程内，否则整帧丢掉。
 */
static bool ControlLink_IsCommandInRange(const ApexControlCommand_t *command)
{
    if (command->throttle > 1000U)
    {
        return false;
    }

    if ((command->roll < -1000) || (command->roll > 1000))
    {
        return false;
    }

    if ((command->pitch < -1000) || (command->pitch > 1000))
    {
        return false;
    }

    if ((command->yaw < -1000) || (command->yaw > 1000))
    {
        return false;
    }

    return true;
}

void ControlLink_Init(void)
{
    taskENTER_CRITICAL();
    memset(&control_link_snapshot, 0, sizeof(control_link_snapshot));
    control_link_has_sequence = false;
    taskEXIT_CRITICAL();
}

bool ControlLink_AcceptCommand(const ApexControlCommand_t *command, uint32_t current_tick)
{
    bool accepted;

    accepted = false;
    if (command == NULL)
    {
        return false;
    }

    if (!ControlLink_IsCommandInRange(command))
    {
        return false;
    }

    taskENTER_CRITICAL();
    /* 失联或尚未收到过序号时，允许任意序号重新建链，方便遥控器重启。 */
    if ((!control_link_snapshot.connected) || (!control_link_has_sequence) || ControlLink_IsNewerSequence(command->sequence, control_link_snapshot.command.sequence))
    {
        control_link_snapshot.command = *command;
        control_link_snapshot.command_valid = true;
        control_link_snapshot.connected = true;
        control_link_snapshot.last_valid_tick = current_tick;
        control_link_snapshot.accepted_count++;
        control_link_has_sequence = true;
        accepted = true;
    }
    else
    {
        control_link_snapshot.rejected_count++;
    }
    taskEXIT_CRITICAL();

    return accepted;
}

void ControlLink_RecordRejectedFrame(void)
{
    taskENTER_CRITICAL();
    control_link_snapshot.rejected_count++;
    taskEXIT_CRITICAL();
}

void ControlLink_Update(uint32_t current_tick)
{
    taskENTER_CRITICAL();
    if (control_link_snapshot.connected && ((current_tick - control_link_snapshot.last_valid_tick) > CONTROL_LINK_TIMEOUT_MS))
    {
        control_link_snapshot.connected = false;
        control_link_snapshot.command_valid = false;
        control_link_snapshot.command.flags = 0U;
        control_link_snapshot.command.throttle = 0U;
    }
    taskEXIT_CRITICAL();
}

void ControlLink_GetSnapshot(ControlLinkSnapshot_t *snapshot)
{
    if (snapshot == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    *snapshot = control_link_snapshot;
    taskEXIT_CRITICAL();
}
