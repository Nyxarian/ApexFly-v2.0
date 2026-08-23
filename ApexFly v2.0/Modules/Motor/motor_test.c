/**
 * @file motor_test.c
 * @brief 遥控油门跟杆测试任务。
 *
 * 本文件负责电调解锁、IMU 读取，以及按遥控快照写电机。
 * 无线任务只写 control_link，这里读快照；禁止在电台任务里写 TIM8。
 *
 * 流程：
 * 1. 初始化电机和 MPU6050
 * 2. 四路 1000us 保持 3 秒，电调解锁
 * 3. 静置采集陀螺仪零偏 200 点
 * 4. 静置采集机体系横滚/俯仰水平零点 200 点，之后控制角减这个平均
 * 5. 停在 1000us 等待遥控
 * 6. 100Hz：刷新失联 → 读快照 → 已解锁则把油门 0~1000 映射到 1000~2000us，再进串级混控
 *
 * 本阶段不使用 roll/pitch/yaw 杆量。Kp 全 0 时倾斜不会纠偏，这是预期现象。
 */

#include "motor_test.h"

#include "attitude_control.h"
#include "battery.h"
#include "blackbox.h"
#include "cmsis_os.h"
#include "control_link.h"
#include "debug.h"
#include "i2c.h"
#include "imu_attitude.h"
#include "imu_bias.h"
#include "imu_level.h"
#include "mixer.h"
#include "motor.h"
#include "mpu6050.h"
#include "pid_table.h"
#include "radio.h"
#include <string.h>

#define ESC_ARM_WAIT_MS 3000U
#define IMU_BIAS_SAMPLE_COUNT 200U
#define IMU_LEVEL_SAMPLE_COUNT 200U
#define IMU_SAMPLE_PERIOD_MS 10U
#define IMU_DELTA_TIME_S 0.01f
#define MOTOR_STATUS_PRINT_MS 1000U

/**
 * @brief 把混控算出的四路脉宽写到对应电机。
 */
static void MotorTest_ApplyMixer(const MixerOutput_t *output)
{
    Motor_SetPulseUs(MOTOR_M1, output->motor_1_us);
    Motor_SetPulseUs(MOTOR_M2, output->motor_2_us);
    Motor_SetPulseUs(MOTOR_M3, output->motor_3_us);
    Motor_SetPulseUs(MOTOR_M4, output->motor_4_us);
}

/**
 * @brief 遥控油门 0~1000 映射到电调 1000~2000 微秒。
 */
static uint16_t MotorTest_ThrottleToPwm(uint16_t throttle)
{
    if (throttle > 1000U) throttle = 1000U;
    return (uint16_t)(MOTOR_PWM_MIN_US + throttle);
}

static int16_t MotorTest_ToCenti(float value)
{
    if (value >= 327.67f) return 32767;
    if (value <= -327.68f) return (int16_t)-32768;
    if (value >= 0.0f) return (int16_t)(value * 100.0f + 0.5f);
    return (int16_t)(value * 100.0f - 0.5f);
}

static int16_t MotorTest_ToI16(float value)
{
    if (value >= 32767.0f) return 32767;
    if (value <= -32768.0f) return (int16_t)-32768;
    if (value >= 0.0f) return (int16_t)(value + 0.5f);
    return (int16_t)(value - 0.5f);
}

/**
 * @brief 电调解锁、采零偏后，按遥控快照输出电机。
 */
void MotorTest_Task(void *argument)
{
    static MPU6050_Device_t imu_device;
    static MPU6050_Data_t imu_data;
    static IMU_GyroBiasCollector_t bias_collector;
    static IMU_LevelCollector_t level_collector;
    static IMU_Attitude_t attitude;
    static IMU_BodyAttitude_t body;
    static MixerOutput_t mixer_output;
    static ControlLinkSnapshot_t link;
    static BlackboxSample_t sample;
    MPU6050_Status_t status;
    bool bias_ready;
    bool level_ready;
    bool armed;
    bool was_armed;
    uint16_t throttle_us;
    uint32_t next_print_tick;
    float roll_corr_us;
    float pitch_corr_us;
    float yaw_corr_us;

    (void)argument;
    Debug_Init();
    PidTable_Init();
    Battery_Init();
    memset(&body, 0, sizeof(body));

    if (!Motor_Init())
    {
        Debug_Printf("[ATT] motor init failed.\r\n");
        for (;;)
        {
            osDelay(1000U);
        }
    }

    /* 上电第一件事就是四路 1000us，避免电调把空 PWM 当成油门。 */
    Motor_Disarm();
    Radio_PublishFlightState(0.0f, 0.0f, 0.0f, MOTOR_PWM_MIN_US, false, true, false);
    Debug_Printf("[ATT] hold 1000us for %ums, ESC unlock.\r\n", (unsigned int)ESC_ARM_WAIT_MS);
    osDelay(ESC_ARM_WAIT_MS);

    for (;;)
    {
        status = MPU6050_Init(&imu_device, &hi2c1);
        if (status != MPU6050_STATUS_OK)
        {
            Motor_Disarm();
            Radio_PublishFlightState(0.0f, 0.0f, 0.0f, MOTOR_PWM_MIN_US, false, true, false);
            Debug_Printf("[ATT] IMU init failed status=%u.\r\n", (unsigned int)status);
            osDelay(1000U);
            continue;
        }

        IMU_GyroBias_Start(&bias_collector, IMU_BIAS_SAMPLE_COUNT);
        IMU_Level_Start(&level_collector, IMU_LEVEL_SAMPLE_COUNT);
        IMU_Attitude_Init(&attitude);
        AttitudeControl_Reset();
        bias_ready = false;
        level_ready = false;
        was_armed = false;
        next_print_tick = osKernelGetTickCount();
        Debug_Printf("[ATT] keep still, gyro bias %u samples.\r\n", (unsigned int)IMU_BIAS_SAMPLE_COUNT);

        for (;;)
        {
            status = MPU6050_ReadData(&imu_device, &imu_data);
            if (status != MPU6050_STATUS_OK)
            {
                if (was_armed) Blackbox_NotifyDisarmed();
                Motor_Disarm();
                Radio_PublishFlightState(0.0f, 0.0f, 0.0f, MOTOR_PWM_MIN_US, false, true, false);
                Debug_Printf("[ATT] IMU read failed, disarm and reinit.\r\n");
                break;
            }

            if (!bias_ready)
            {
                if (!IMU_GyroBias_Update(&bias_collector, &imu_data))
                {
                    osDelay(IMU_SAMPLE_PERIOD_MS);
                    continue;
                }

                bias_ready = true;
                Motor_Disarm();
                AttitudeControl_Reset();
                Debug_Printf("[ATT] gyro bias ready, keep still, level %u samples.\r\n", (unsigned int)IMU_LEVEL_SAMPLE_COUNT);
                continue;
            }

            IMU_GyroBias_Apply(&bias_collector.bias, &imu_data);
            if (!IMU_Attitude_Update(&attitude, &imu_data, IMU_DELTA_TIME_S))
            {
                osDelay(IMU_SAMPLE_PERIOD_MS);
                continue;
            }
            IMU_Attitude_MapToBody(&attitude, &imu_data, &body);

            if (!level_ready)
            {
                if (!IMU_Level_Update(&level_collector, &body))
                {
                    osDelay(IMU_SAMPLE_PERIOD_MS);
                    continue;
                }

                level_ready = true;
                Motor_Disarm();
                AttitudeControl_Reset();
                Debug_Printf("[ATT] level roll=%.2f pitch=%.2f, wait remote arm then throttle.\r\n", (double)level_collector.offset.roll_deg, (double)level_collector.offset.pitch_deg);
                continue;
            }

            IMU_Level_Apply(&level_collector.offset, &body);

            ControlLink_Update((uint32_t)osKernelGetTickCount());
            ControlLink_GetSnapshot(&link);
            armed = link.connected && link.command_valid && ((link.command.flags & APEX_CONTROL_FLAG_ARM_REQUESTED) != 0U);

            if (!armed)
            {
                throttle_us = MOTOR_PWM_MIN_US;
                AttitudeControl_Reset();
                Motor_Disarm();
                if (was_armed) Blackbox_NotifyDisarmed();
            }
            else
            {
                if (!was_armed) Blackbox_NotifyArmed();
                throttle_us = MotorTest_ThrottleToPwm(link.command.throttle);
                if (AttitudeControl_Update((float)throttle_us, &body, IMU_DELTA_TIME_S, &mixer_output))
                {
                    MotorTest_ApplyMixer(&mixer_output);
                    AttitudeControl_GetLastCorrections(&roll_corr_us, &pitch_corr_us, &yaw_corr_us);
                    memset(&sample, 0, sizeof(sample));
                    sample.roll_cdeg = MotorTest_ToCenti(body.roll_deg);
                    sample.pitch_cdeg = MotorTest_ToCenti(body.pitch_deg);
                    sample.yaw_cdeg = MotorTest_ToCenti(body.yaw_deg);
                    sample.roll_rate_cdps = MotorTest_ToCenti(body.roll_rate_dps);
                    sample.pitch_rate_cdps = MotorTest_ToCenti(body.pitch_rate_dps);
                    sample.yaw_rate_cdps = MotorTest_ToCenti(body.yaw_rate_dps);
                    sample.roll_corr_us = MotorTest_ToI16(roll_corr_us);
                    sample.pitch_corr_us = MotorTest_ToI16(pitch_corr_us);
                    sample.yaw_corr_us = MotorTest_ToI16(yaw_corr_us);
                    sample.throttle_us = throttle_us;
                    sample.motor_us[0] = mixer_output.motor_1_us;
                    sample.motor_us[1] = mixer_output.motor_2_us;
                    sample.motor_us[2] = mixer_output.motor_3_us;
                    sample.motor_us[3] = mixer_output.motor_4_us;
                    sample.armed = 1U;
                    sample.connected = link.connected ? 1U : 0U;
                    Blackbox_SubmitSample(&sample);
                }
                else
                {
                    Motor_SetAllPulseUs(throttle_us);
                }
            }
            was_armed = armed;

            Radio_PublishFlightState(body.roll_deg, body.pitch_deg, body.yaw_deg, throttle_us, true, true, armed);

            if ((int32_t)(osKernelGetTickCount() - next_print_tick) >= 0)
            {
                Battery_Update();
                Debug_Printf("[ATT] r=%.2f p=%.2f y=%.2f conn=%u arm=%u thr=%u pwm=%u bat=%u acc=%lu rej=%lu.\r\n", (double)body.roll_deg, (double)body.pitch_deg, (double)body.yaw_deg, link.connected ? 1U : 0U, armed ? 1U : 0U, (unsigned int)link.command.throttle, (unsigned int)throttle_us, (unsigned int)Battery_GetMillivolts(), (unsigned long)link.accepted_count, (unsigned long)link.rejected_count);
                next_print_tick = osKernelGetTickCount() + MOTOR_STATUS_PRINT_MS;
            }

            osDelay(IMU_SAMPLE_PERIOD_MS);
        }
    }
}
