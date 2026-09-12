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

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 遥控跟杆测试任务入口。
 * @param argument 未使用。
 */
void MotorTest_Task(void *argument);

/**
 * @brief IMU 数据就绪（DRDY）中断回调，由 EXTI0 的 HAL_GPIO_EXTI_Callback 调用。
 *
 * 运行在中断上下文：只释放信号量通知控制任务，不做任何 I2C 访问。
 * 一次 DRDY 对应一帧控制，用 SMPLRT_DIV 把芯片输出做成 100Hz。
 */
void MotorTest_NotifyImuDrdyFromISR(void);

/**
 * @brief 【临时探针，H4' 验证完删除】取控制环单拍耗时峰值，单位微秒，读走即清零。
 *
 * 每次调用返回"上次调用以来"的最大单拍耗时。一拍预算约 4008us。
 * 由 tofTask 在打印里调用（控制环本身禁止打印）。
 */
uint32_t MotorTest_TakeMaxFrameUs(void);

#ifdef __cplusplus
}
#endif

#endif
