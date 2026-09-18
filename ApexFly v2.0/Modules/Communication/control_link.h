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

/* 失联门限，单位毫秒：超过这么久没收到新的有效控制帧就判失联。

   ⚠️⚠️ **这个值直接决定"空中会不会停桨"**：失联 ⇒ `armed=false` ⇒
      `motor_test.c` 的 `!armed` 分支立刻四路 1000（**空中停桨**）。
      它是**整个飞控的既有设计**，比定高的失效处理更靠前，定高救不了它。

   ⚠️ **2026-09-15 由 800 放宽到 1500**（用户决定，首飞前）。依据：
      LoRa 一问一答**实测间隔 300~320ms** ⇒ 800ms 只容忍**连丢 2~3 帧**就停桨，
      对首飞（视距内、链路本来就不稳）太紧。1500ms 容忍连丢 **4~5 帧**。
   ⚠️ **但这只是"把停桨推迟 0.7 秒"，不是根治**：
      真正的做法是"失联后**维持最后一次指令**继续飞一段时间（failsafe hold）"，
      那要改 `!armed` 分支，属独立改动，**登记为待办**（见施工顺序文档待办 9）。
   ⚠️ 放宽门限的代价：真失联时会**多飞 0.7 秒才停桨**，
      期间飞机带着最后一次指令继续走（定高模式下它会维持最后的目标高度）。
      对首飞来说这个取舍是划算的。 */
#define CONTROL_LINK_TIMEOUT_MS 1500U

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
