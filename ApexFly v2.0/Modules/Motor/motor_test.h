/**
 * @file motor_test.h
 * @brief IMU 姿态自稳 + 遥控油门测试任务。
 *
 * 测试前必须拆桨。
 * 电调解锁后先采陀螺零偏，再平放采水平零点，然后基础油门改由遥控 control_link 快照提供。
 * 未连接、未解锁或失联时四路保持 1000us。
 * 当前 6 组 PID 增益为 0，推油门只改变四路共同转速，倾斜不会纠偏。
 */

#ifndef MOTOR_TEST_H
#define MOTOR_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 遥控跟杆测试任务入口。
 * @param argument 未使用。
 */
void MotorTest_Task(void *argument);

#ifdef __cplusplus
}
#endif

#endif
