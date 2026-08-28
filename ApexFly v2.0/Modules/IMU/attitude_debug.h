/**
 * @file attitude_debug.h
 * @brief 姿态角调试串口任务接口。
 *
 * 当前只打印磁力计快照，姿态和 GPS 观察量先关掉。
 * 不读取 MPU6050，不参与姿态控制，不写电机输出。
 */

#ifndef ATTITUDE_DEBUG_H
#define ATTITUDE_DEBUG_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 姿态角调试串口任务入口。
 * @param argument 未使用。
 */
void AttitudeDebug_Task(void *argument);

#ifdef __cplusplus
}
#endif

#endif
