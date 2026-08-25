/**
 * @file attitude_debug.h
 * @brief 姿态角调试串口任务接口。
 *
 * 本任务读取电机任务发布的姿态快照，以及 GPS 模块的时间和定位快照。
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
