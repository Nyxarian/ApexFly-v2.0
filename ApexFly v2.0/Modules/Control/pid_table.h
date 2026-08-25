/**
 * @file pid_table.h
 * @brief 飞机端 RAM 参数表：对接遥控 0x07 的 30 个姿态/油门参数 ID。
 *
 * 本模块把协议 ID 映射到 attitude_control 的串级参数，以及满杆油门上限。
 * GET 回整张表；SET 改一项后立刻写进正在跑的 PID 或油门映射。
 * SAVE / LOAD_SAVED 交给黑匣子任务写 W25Q64，电台任务不碰 SPI3。
 * 六面校准操作码协议里还在，这里一律回 INVALID_OPERATION，不再实现。
 *
 * APEX_PID_HOVER_PWM_US 现在表示满杆对应的最高油门，单位微秒，最低油门固定 1000。
 * 无线任务调用本模块，不写 TIM8。解锁中禁止改参。
 */

#ifndef PID_TABLE_H
#define PID_TABLE_H

#include "apex_protocol.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 用编译默认值初始化 RAM 表，并灌进三轴串级 PID。
 */
void PidTable_Init(void);

/**
 * @brief 处理一条遥控 PID 命令，结果写入 response，始终带回整张 30 项表。
 * @param command 已解码的 PID 命令。
 * @param armed true 表示当前已解锁，SET 将被拒绝。
 * @param response 应答，调用者负责发出去。
 */
void PidTable_HandleCommand(const ApexPidCommand_t *command, bool armed, ApexPidResponse_t *response);

/**
 * @brief 把当前 RAM 表导出成协议 q10000，给黑匣子场次头和 SAVE 用。
 * @param values_q10000 长度必须是 APEX_PID_PARAMETER_COUNT。
 */
void PidTable_ExportSnapshot(int32_t *values_q10000);

/**
 * @brief 用 Flash 或已保存快照覆盖 RAM 表，并灌进正在跑的 PID。
 * @param values_q10000 长度必须是 APEX_PID_PARAMETER_COUNT。
 * @return 全部项合法并已应用返回 true。
 */
bool PidTable_ImportSnapshot(const int32_t *values_q10000);

/**
 * @brief 读取当前满杆油门上限。
 * @return 满杆对应的 PWM 脉宽，单位微秒，范围 1200～2000。
 */
uint16_t PidTable_GetMaxThrottleUs(void);

#ifdef __cplusplus
}
#endif

#endif
