/**
 * @file motor.c
 * @brief TIM8 四路电调 PWM 输出实现。
 *
 * 调用 Motor_Init() 前必须已经执行 MX_TIM8_Init()。
 * 启动前先把四路比较值写成 1000us，避免电调上电时看到无效脉宽。
 *
 * TIM8 计数频率为 1MHz，一个计数值等于 1us。
 * 周期 20000 对应 50Hz。本文件只改比较寄存器，不改定时器时基。
 */

#include "motor.h"
#include "tim.h"

/* true 表示四路 PWM 已经启动，未启动时禁止改脉宽。 */
static bool motor_pwm_started = false;

/* 电机编号到 TIM8 通道的对应关系。 */
static const uint32_t motor_channels[MOTOR_COUNT] =
{
    TIM_CHANNEL_1, /* M1：PC6，后右 */
    TIM_CHANNEL_2, /* M2：PC7，前右 */
    TIM_CHANNEL_3, /* M3：PC8，后左 */
    TIM_CHANNEL_4  /* M4：PC9，前左 */
};

/**
 * @brief 把脉宽限制在电调允许的 1000～2000us。
 */
static uint16_t Motor_LimitPulse(uint16_t pulse_us)
{
    if (pulse_us < MOTOR_PWM_MIN_US)
    {
        pulse_us = MOTOR_PWM_MIN_US;
    }
    if (pulse_us > MOTOR_PWM_MAX_US)
    {
        pulse_us = MOTOR_PWM_MAX_US;
    }
    return pulse_us;
}

/**
 * @brief 把已经限制过的脉宽写入对应通道的比较寄存器。
 */
static void Motor_WriteCompare(MotorId_t motor, uint16_t pulse_us)
{
    if (motor >= MOTOR_COUNT)
    {
        return;
    }
    __HAL_TIM_SET_COMPARE(&htim8, motor_channels[motor], pulse_us);
}

/**
 * @brief 启动 TIM8 四路 PWM，并保证上电时就是最小油门。
 */
bool Motor_Init(void)
{
    uint8_t i;

    /* 已经启动过就不要再 Start，避免打乱当前脉宽。 */
    if (motor_pwm_started)
    {
        Motor_Disarm();
        return true;
    }

    /* TIM8 是高级定时器，必须打开主输出，否则引脚没有稳定 PWM。 */
    __HAL_TIM_MOE_ENABLE(&htim8);
    /* 启动波形前先写成 1000us，防止第一帧出现 CubeMX 以外的值。 */
    for (i = 0U; i < (uint8_t)MOTOR_COUNT; i++)
    {
        Motor_WriteCompare((MotorId_t)i, MOTOR_PWM_MIN_US);
        /* main 里可能已经 Start 过；重复 Start 失败也继续，后面再写一次 1000us。 */
        (void)HAL_TIM_PWM_Start(&htim8, motor_channels[i]);
    }

    motor_pwm_started = true;
    Motor_Disarm();
    return true;
}

/**
 * @brief 设置单个电机脉宽。PWM 未启动或编号非法时直接返回。
 */
void Motor_SetPulseUs(MotorId_t motor, uint16_t pulse_us)
{
    if (!motor_pwm_started)
    {
        return;
    }
    if (motor >= MOTOR_COUNT)
    {
        return;
    }

    Motor_WriteCompare(motor, Motor_LimitPulse(pulse_us));
}

/**
 * @brief 四路写成同一个脉宽，用于怠速或统一停转。
 */
void Motor_SetAllPulseUs(uint16_t pulse_us)
{
    uint8_t i;

    if (!motor_pwm_started)
    {
        return;
    }

    pulse_us = Motor_LimitPulse(pulse_us);
    for (i = 0U; i < (uint8_t)MOTOR_COUNT; i++)
    {
        Motor_WriteCompare((MotorId_t)i, pulse_us);
    }
}

/**
 * @brief 四路保持 1000us。PWM 继续输出，电调维持锁定。
 */
void Motor_Disarm(void)
{
    Motor_SetAllPulseUs(MOTOR_PWM_MIN_US);
}

/**
 * @brief 先回到最小油门，再停止四路 PWM 波形。
 */
void Motor_StopOutput(void)
{
    if (motor_pwm_started)
    {
        Motor_Disarm();
    }

    (void)HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_2);
    (void)HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_3);
    (void)HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_4);
    motor_pwm_started = false;
}

/**
 * @brief 查询四路 PWM 是否已经启动。
 */
bool Motor_IsStarted(void)
{
    return motor_pwm_started;
}
