/**
 * @file control_link.h
 * @brief 飞机端遥控指令快照：最新杆量、序号过滤和失联判断。
 *
 * 无线任务收包后写入快照，电机/姿态任务读取快照。
 * 本模块不读写 LoRa，也不写 TIM8。
 *
 * 遥控大约 100ms 一包，控制环 10ms 一拍，两边不能共用同一帧缓冲区。
 * 超过 CONTROL_LINK_TIMEOUT_MS 没有新的有效控制帧，视为失联，油门清零。
 */

#ifndef CONTROL_LINK_H
#define CONTROL_LINK_H

#include "apex_protocol.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 台架阶段失联门限。LoRa 一问一答实测间隔约 300~320ms，放宽到 800ms 避免正常抖动被判失联。 */
#define CONTROL_LINK_TIMEOUT_MS 800U

/**
 * @brief 电机任务看到的当前遥控状态。
 */
typedef struct
{
    /* true 表示最近 CONTROL_LINK_TIMEOUT_MS 内收到过有效控制帧。 */
    bool connected;
    /* true 表示 command 里的杆量可以使用。 */
    bool command_valid;
    /* 最新有效控制指令：油门 0~1000，姿态/偏航 -1000~1000。 */
    ApexControlCommand_t command;
    /* 上一包有效控制帧的 FreeRTOS 节拍，单位毫秒。 */
    uint32_t last_valid_tick;
    /* 已接受的有效控制帧计数。 */
    uint32_t accepted_count;
    /* 校验失败、乱序或超范围而丢掉的帧计数。 */
    uint32_t rejected_count;
} ControlLinkSnapshot_t;

/**
 * @brief 清空快照，上电或无线模块重新初始化时调用。
 */
void ControlLink_Init(void);

/**
 * @brief 尝试收下一条已解码的控制指令。
 * @param command 协议层解出的控制负载。
 * @param current_tick 当前 FreeRTOS 节拍，单位毫秒。
 * @return 序号更新且量程合法则返回 true，否则返回 false。
 */
bool ControlLink_AcceptCommand(const ApexControlCommand_t *command, uint32_t current_tick);

/**
 * @brief 记录一帧被协议校验丢掉，只增加 rejected_count。
 */
void ControlLink_RecordRejectedFrame(void);

/**
 * @brief 用当前节拍刷新失联状态。电机任务每拍调用一次。
 * @param current_tick 当前 FreeRTOS 节拍，单位毫秒。
 */
void ControlLink_Update(uint32_t current_tick);

/**
 * @brief 复制一份当前快照给调用者，读写都在临界区里完成。
 * @param snapshot 调用者提供的快照地址。
 */
void ControlLink_GetSnapshot(ControlLinkSnapshot_t *snapshot);

#ifdef __cplusplus
}
#endif

#endif
