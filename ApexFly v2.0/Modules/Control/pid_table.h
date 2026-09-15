/**
 * @file pid_table.h
 * @brief 飞机端 RAM 参数表：对接遥控 0x07 的 30 个姿态/油门参数 ID。
 *
 * 本模块把协议 ID 映射到 attitude_control 的串级参数，以及满杆油门上限。
 * GET 回整张表；SET 改一项后立刻写进正在跑的 PID 或油门映射。
 * SAVE / LOAD_SAVED 交给黑匣子任务写 W25Q64，电台任务不碰 SPI3。
 * 操作码 5/6/7 已废弃，飞机一律回 INVALID_OPERATION。
 * MAG_CAL_START/STOP/CANCEL 转给磁力计任务；STOP 成功后再排队写 0x002000。
 *
 * APEX_PID_HOVER_PWM_US 这个枚举名是历史遗留，它表示的是【满杆对应的最高油门】，
 * 不是"悬停需要多少油门"，单位微秒，最低油门固定 1000。
 * 模块内部一律叫 max_throttle_us；只有协议枚举名保留旧写法，免得两端错位。
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

/**
 * @brief 读取定高**垂速环（内环）**的三个增益（协议 ID 25~27）。
 *
 * ⚠️ 这是**内环**。高度轴的**外环**（高度误差 → 目标垂速）是另一组，
 *    ID 30~32，要到 H13 才会被控制代码读取，届时在这里补一个对应的
 *    `PidTable_GetHeightParams()`（现在不提前加：没有调用方的接口就是死代码）。
 *
 * 这三项在 H9 之前只作为协议 ID 的读写载体存在、不参与任何控制；
 * H9 把垂速内环接上之后它们才真正生效。**本函数只提供读，不写 PID**——
 * 谁用谁负责判断"参数变了没有、要不要 Pid_ApplyParams"，因为 ApplyParams 会清积分。
 *
 * @param vz_kp 输出，垂速内环比例增益，单位 µs/(cm/s)。可为空。
 * @param vz_ki 输出，垂速内环积分增益，单位 µs/cm。可为空。
 * @param vz_kd 输出，垂速内环微分增益。可为空。
 */
void PidTable_GetAltitudeParams(float *vz_kp, float *vz_ki, float *vz_kd);

/**
 * @brief 读取横滚、俯仰 Trim，加在跟杆目标角上，单位度。
 * @param roll_trim_deg 横滚 Trim，右倾为正。
 * @param pitch_trim_deg 俯仰 Trim，抬头为正。
 */
void PidTable_GetAttitudeTrimDeg(float *roll_trim_deg, float *pitch_trim_deg);

#ifdef __cplusplus
}
#endif

#endif
