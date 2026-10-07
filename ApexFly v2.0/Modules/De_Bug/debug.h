/**
 * @file debug.h
 * @brief 调试串口输出接口。
 *
 * 本模块使用 USART3（PC10 发送，PC11 接收，波特率 115200），与老工程一致。
 * USART2 留给 GPS，不要用来输出日志或波形。
 *
 * 当前 USART3 用于三类文本：
 * - 姿态 [ATT] 低频观测行
 * - 故障、参数和校准结果
 * - 黑匣子 dump / dumpall 的 CSV 和进度
 * VOFA 波形接口保留为空操作，避免和 CSV 混流。
 */

#ifndef DEBUG_H
#define DEBUG_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Debug_Printf() 单次格式化缓冲区长度，超出部分会被截断。 */
#define DEBUG_PRINTF_BUFFER_SIZE 384U

/* VOFA+ 单帧最多发送的浮点通道数量。 */
#define DEBUG_VOFA_MAX_CHANNELS  12U

/**
 * @brief 创建调试串口互斥锁。
 *
 * 调度器启动前、启动后都可以调用。
 * 各个发送接口第一次使用时也会自动完成初始化。
 */
void Debug_Init(void);

/** @brief 发布最近一拍姿态，供低优先级任务打印，不执行串口发送。 */
void Debug_SetAttitudeSnapshot(float roll_deg, float pitch_deg, float yaw_deg);

/** @brief 读取最近一拍姿态；尚未收到有效姿态时返回 false。 */
bool Debug_GetAttitudeSnapshot(float *roll_deg, float *pitch_deg, float *yaw_deg);

/** @brief 发布姿态健康快照，供低优先级任务打印。 */
void Debug_SetAttitudeHealthSnapshot(float accel_norm_g, bool accel_correction_valid, uint8_t accel_health_state, bool stationary, float fusion_error_norm, float accel_correction_weight, float gyro_bias_x_dps, float gyro_bias_y_dps, float gyro_bias_z_dps, bool gyro_bias_learning);

/** @brief 读取姿态健康快照。 */
bool Debug_GetAttitudeHealthSnapshot(float *accel_norm_g, bool *accel_correction_valid, uint8_t *accel_health_state, bool *stationary, float *fusion_error_norm, float *accel_correction_weight, float *gyro_bias_x_dps, float *gyro_bias_y_dps, float *gyro_bias_z_dps, bool *gyro_bias_learning);

/** @brief 输出姿态和健康字段，供阶段 4 低频诊断。 */
int Debug_PrintAttitudeHealthAngles(float roll_deg, float pitch_deg, float yaw_deg, float accel_norm_g, bool accel_correction_valid, uint8_t accel_health_state, bool stationary, float fusion_error_norm, float accel_correction_weight, float gyro_bias_x_dps, float gyro_bias_y_dps, float gyro_bias_z_dps, bool gyro_bias_learning);

/**
 * @brief 仅供姿态调试任务输出机体系三角。
 * @param roll_deg 横滚角，单位度。
 * @param pitch_deg 俯仰角，单位度。
 * @param yaw_deg 偏航角，单位度。
 * @return 成功返回发送字符数，失败返回 -1。
 */
int Debug_PrintAttitudeAngles(float roll_deg, float pitch_deg, float yaw_deg);

/**
 * @brief 通过 USART3 发送一段原始字节。
 * @param data 待发送数据。
 * @param length 字节数。
 * @return 成功返回 HAL_OK；参数无效返回 HAL_ERROR；未拿到互斥锁返回 HAL_BUSY。
 */
HAL_StatusTypeDef Debug_Send(const uint8_t *data, uint16_t length);

/**
 * @brief 格式化文本日志并发送，用法与 printf 相同。
 *
 * 建议以回车换行结尾。VOFA+ 选择 FireWater 协议即可看到这些字符串。
 * @return 实际发送的字符数；失败返回 -1。
 */
int Debug_Printf(const char *format, ...);

/**
 * @brief 按 VOFA+ 的 JustFloat 协议发送一组浮点通道。
 *
 * 帧格式：若干个小端浮点数，再附加 4 字节帧尾 00 00 80 7F。
 * VOFA+ 协议请选择 JustFloat。
 * @param channels 通道数组。
 * @param channel_count 通道数量，范围 1 到 DEBUG_VOFA_MAX_CHANNELS。
 */
HAL_StatusTypeDef Debug_VofaJustFloat(const float *channels, uint8_t channel_count);

/**
 * @brief 按 VOFA+ 的 FireWater 文本协议发送一组浮点通道。
 *
 * 帧格式：逗号分隔的浮点文本，以回车换行结束，例如 "0.12,0.34,1.00"。
 * VOFA+ 协议请选择 FireWater。适合同时查看波形和少量文本。
 */
HAL_StatusTypeDef Debug_VofaFireWater(const float *channels, uint8_t channel_count);

/**
 * @brief 把 MPU6050 六轴物理量发送到 VOFA+ 的 JustFloat 波形。
 *
 * 通道顺序固定为：
 * 第 0 路：X 轴加速度，单位 g
 * 第 1 路：Y 轴加速度，单位 g
 * 第 2 路：Z 轴加速度，单位 g
 * 第 3 路：X 轴角速度，单位 度每秒
 * 第 4 路：Y 轴角速度，单位 度每秒
 * 第 5 路：Z 轴角速度，单位 度每秒
 */
HAL_StatusTypeDef Debug_VofaImu6Axis(float accel_x_g, float accel_y_g, float accel_z_g, float gyro_x_dps, float gyro_y_dps, float gyro_z_dps);

#ifdef __cplusplus
}
#endif

#endif
