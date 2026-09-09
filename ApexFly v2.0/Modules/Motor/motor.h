/**
 * @file motor.h
 * @brief 四路电调 PWM 输出。
 *
 * 使用 TIM8 的 CH1～CH4，协议为普通 50Hz PWM，脉宽单位微秒。
 * 本模块只负责把脉宽写到定时器，不负责混控、PID 和解锁。
 *
 * 通道对应（沿用老工程）：
 * M1：PC6 / TIM8_CH1，后右
 * M2：PC7 / TIM8_CH2，前右
 * M3：PC8 / TIM8_CH3，后左
 * M4：PC9 / TIM8_CH4，前左
 *
 * TIM8 计数频率 1MHz，比较值 1000 表示 1000us。
 * 上电和锁定时输出 1000us，电调保持停转。
 * 正常停转用 Motor_Disarm()；Motor_StopOutput() 只给故障用。
 */

#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    MOTOR_M1 = 0, /* 后右，PC6 / TIM8_CH1 */
    MOTOR_M2,     /* 前右，PC7 / TIM8_CH2 */
    MOTOR_M3,     /* 后左，PC8 / TIM8_CH3 */
    MOTOR_M4,     /* 前左，PC9 / TIM8_CH4 */
    MOTOR_COUNT
} MotorId_t;

/* 电调最小油门，停转和锁定都用这个脉宽。 */
#define MOTOR_PWM_MIN_US  1000U
/* 可选怠速脉宽，点转测试时不要直接拉到最大。
   必须不低于混控有效下限（MIXER_PWM_EFFECTIVE_MIN_US，当前 1100us）：
   驱动层不能反向包含混控层头文件，所以这里手工同步。 */
#define MOTOR_PWM_IDLE_US 1100U
/* 电调最大油门，驱动层会把超出的值裁到这里。 */
#define MOTOR_PWM_MAX_US  2000U

/**
 * @brief 启动 TIM8 四路 PWM，先输出最小油门。
 *
 * 调用前必须已经执行 MX_TIM8_Init()。
 * 可以重复调用，已经启动过则直接返回成功。
 * @return 四路都启动成功返回 true。
 */
bool Motor_Init(void);

/**
 * @brief 设置单个电机的 PWM 脉宽。
 * @param motor 电机编号，MOTOR_M1～MOTOR_M4。
 * @param pulse_us 脉宽，单位微秒，会被限制在 1000～2000。
 */
void Motor_SetPulseUs(MotorId_t motor, uint16_t pulse_us);

/**
 * @brief 四路电机写成同一个脉宽。
 * @param pulse_us 脉宽，单位微秒，会被限制在 1000～2000。
 */
void Motor_SetAllPulseUs(uint16_t pulse_us);

/**
 * @brief 四路保持 1000us，电调进入锁定/停转。
 *
 * PWM 波形继续输出。普通停转请用这个函数，不要停定时器。
 */
void Motor_Disarm(void);

/**
 * @brief 停止四路 PWM 波形。
 *
 * 正常锁定不要用这个函数，一般只用于严重故障或程序关闭。
 */
void Motor_StopOutput(void);

/**
 * @brief 查询四路 PWM 是否已经启动。
 * @return 已经启动返回 true。
 */
bool Motor_IsStarted(void);

#ifdef __cplusplus
}
#endif

#endif
