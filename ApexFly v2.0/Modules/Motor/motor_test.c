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
 * 6. 100Hz：刷新失联 → 读快照 → 已解锁则把油门杆直通映射成基础油门，再进串级混控
 *
 * 油门映射：最低固定 1000us，最高由 PID 表 APEX_PID_HOVER_PWM_US 决定，默认 1500us。
 * 气压和高度估计只用于遥测显示，不再控制电机。
 * 本阶段不使用 roll/pitch/yaw 杆量。Kp 全 0 时倾斜不会纠偏，这是预期现象。
 */

#include "motor_test.h"

#include "altitude_est.h"
#include "attitude_control.h"
#include "baro.h"
#include "battery.h"
#include "blackbox.h"
#include "cmsis_os.h"
#include "control_link.h"
#include "i2c.h"
#include "imu_attitude.h"
#include "imu_bias.h"
#include "imu_level.h"
#include "mixer.h"
#include "motor.h"
#include "mpu6050.h"
#include "pid_table.h"
#include "radio.h"
#include <math.h>
#include <string.h>

#define ESC_ARM_WAIT_MS 3000U
#define IMU_BIAS_SAMPLE_COUNT 200U
#define IMU_LEVEL_SAMPLE_COUNT 200U
#define IMU_SAMPLE_PERIOD_MS 10U
#define IMU_DELTA_TIME_S 0.01f
#define MOTOR_BATTERY_UPDATE_MS 1000U
#define MOTOR_DT_MIN_S 0.005f
#define MOTOR_DT_MAX_S 0.03f
#define MOTOR_GROUND_RATE_LIMIT_DPS 3.0f
#define MOTOR_GROUND_ACCEL_LIMIT_MPS2 0.5f
#define MOTOR_ATTITUDE_ENABLE_PWM_US 1050U

/**
 * @brief 把混控算出的四路脉宽写到对应电机。
 */
static void MotorTest_ApplyMixer(MixerOutput_t *output)
{
    /* 高度控制已启动后每路至少保持 1050us，避免姿态修正把单个电机压回停转区。 */
    if (output->motor_1_us < MOTOR_ATTITUDE_ENABLE_PWM_US) output->motor_1_us = MOTOR_ATTITUDE_ENABLE_PWM_US;
    if (output->motor_2_us < MOTOR_ATTITUDE_ENABLE_PWM_US) output->motor_2_us = MOTOR_ATTITUDE_ENABLE_PWM_US;
    if (output->motor_3_us < MOTOR_ATTITUDE_ENABLE_PWM_US) output->motor_3_us = MOTOR_ATTITUDE_ENABLE_PWM_US;
    if (output->motor_4_us < MOTOR_ATTITUDE_ENABLE_PWM_US) output->motor_4_us = MOTOR_ATTITUDE_ENABLE_PWM_US;
    Motor_SetPulseUs(MOTOR_M1, output->motor_1_us);
    Motor_SetPulseUs(MOTOR_M2, output->motor_2_us);
    Motor_SetPulseUs(MOTOR_M3, output->motor_3_us);
    Motor_SetPulseUs(MOTOR_M4, output->motor_4_us);
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

static float MotorTest_Clampf(float value, float minimum, float maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

/**
 * @brief 把遥控油门杆 0～1000 线性映射成基础油门。
 * @param stick 遥控油门杆量，0 表示最低，1000 表示满杆。
 * @return 基础油门，单位微秒。未使用的杆量对应 1000us，满杆对应当前最高油门。
 */
static uint16_t MotorTest_MapStickToThrottleUs(uint16_t stick)
{
    uint16_t stick_used;
    uint16_t max_throttle_us;
    uint32_t span_us;

    stick_used = (stick > 1000U) ? 1000U : stick;
    max_throttle_us = PidTable_GetMaxThrottleUs();
    if (max_throttle_us < MOTOR_PWM_MIN_US) max_throttle_us = MOTOR_PWM_MIN_US;
    if (max_throttle_us > MOTOR_PWM_MAX_US) max_throttle_us = MOTOR_PWM_MAX_US;
    span_us = (uint32_t)max_throttle_us - (uint32_t)MOTOR_PWM_MIN_US;
    return (uint16_t)(MOTOR_PWM_MIN_US + ((span_us * (uint32_t)stick_used) / 1000U));
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
    uint32_t last_imu_tick;
    uint32_t next_battery_tick;
    uint32_t imu_tick;
    float control_dt_s;
    float accel_bias_sum;
    uint16_t accel_bias_count;
    float roll_corr_us;
    float pitch_corr_us;
    float yaw_corr_us;
    float baro_altitude_m;
    float accel_up_mps2;
    bool baro_fresh;
    AltitudeEstimate_t alt_est;
    BaroSnapshot_t baro_snap;
    int32_t altitude_cm;
    int16_t vz_cms;

    (void)argument;
    PidTable_Init();
    Battery_Init();
    memset(&body, 0, sizeof(body));
    memset(&alt_est, 0, sizeof(alt_est));
    throttle_us = MOTOR_PWM_MIN_US;
    altitude_cm = 0;
    vz_cms = 0;

    if (!Motor_Init())
    {
        for (;;)
        {
            osDelay(1000U);
        }
    }

    /* 上电第一件事就是四路 1000us，避免电调把空 PWM 当成油门。 */
    Motor_Disarm();
    Radio_PublishFlightState(0.0f, 0.0f, 0.0f, MOTOR_PWM_MIN_US, 0, 0, false, true, false);
    osDelay(ESC_ARM_WAIT_MS);

    for (;;)
    {
        status = MPU6050_Init(&imu_device, &hi2c1);
        if (status != MPU6050_STATUS_OK)
        {
            Motor_Disarm();
            Radio_PublishFlightState(0.0f, 0.0f, 0.0f, MOTOR_PWM_MIN_US, 0, 0, false, true, false);
            osDelay(1000U);
            continue;
        }

        IMU_GyroBias_Start(&bias_collector, IMU_BIAS_SAMPLE_COUNT);
        IMU_Level_Start(&level_collector, IMU_LEVEL_SAMPLE_COUNT);
        IMU_Attitude_Init(&attitude);
        AttitudeControl_Reset();
        AltitudeEst_Init(0.0f);
        bias_ready = false;
        level_ready = false;
        was_armed = false;
        last_imu_tick = 0U;
        next_battery_tick = osKernelGetTickCount();
        control_dt_s = IMU_DELTA_TIME_S;
        accel_bias_sum = 0.0f;
        accel_bias_count = 0U;
    
        for (;;)
        {
            status = MPU6050_ReadData(&imu_device, &imu_data);
            if (status != MPU6050_STATUS_OK)
            {
                if (was_armed) Blackbox_NotifyDisarmed();
                Motor_Disarm();
                Radio_PublishFlightState(0.0f, 0.0f, 0.0f, MOTOR_PWM_MIN_US, 0, 0, false, true, false);
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
                continue;
            }

            imu_tick = osKernelGetTickCount();
            if (last_imu_tick != 0U) control_dt_s = MotorTest_Clampf((float)(imu_tick - last_imu_tick) * 0.001f, MOTOR_DT_MIN_S, MOTOR_DT_MAX_S);
            last_imu_tick = imu_tick;

            IMU_GyroBias_Apply(&bias_collector.bias, &imu_data);
            if (!IMU_Attitude_Update(&attitude, &imu_data, control_dt_s))
            {
                osDelay(IMU_SAMPLE_PERIOD_MS);
                continue;
            }
            IMU_Attitude_MapToBody(&attitude, &imu_data, &body);

            if (!level_ready)
            {
                /* 水平零点采样期间机体静止，顺便平均垂直线性加速度作为高度滤波初始 ba。 */
                accel_bias_sum += IMU_Attitude_VerticalAccelUp(&attitude, &imu_data);
                if (accel_bias_count < 0xFFFFU) accel_bias_count++;
                if (!IMU_Level_Update(&level_collector, &body))
                {
                    osDelay(IMU_SAMPLE_PERIOD_MS);
                    continue;
                }

                level_ready = true;
                Motor_Disarm();
                AttitudeControl_Reset();
                AltitudeEst_Init((accel_bias_count > 0U) ? (accel_bias_sum / (float)accel_bias_count) : 0.0f);
                continue;
            }

            IMU_Level_Apply(&level_collector.offset, &body);

            /* 气压 8 Hz，加速度 100 Hz：每拍都预测，有新气压才更新。 */
            baro_altitude_m = 0.0f;
            baro_fresh = Baro_TakeNewAltitude(&baro_altitude_m);
            accel_up_mps2 = IMU_Attitude_VerticalAccelUp(&attitude, &imu_data);
            AltitudeEst_Update(accel_up_mps2, control_dt_s, baro_altitude_m, baro_fresh);
            AltitudeEst_Get(&alt_est);
            /* 卡尔曼还没跟上时，遥测先发气压相对高度，避免遥控一直 0.0m。 */
            if (alt_est.healthy)
            {
                altitude_cm = MotorTest_ToI16(alt_est.z_m * 100.0f);
                vz_cms = MotorTest_ToCenti(alt_est.vz_mps);
            }
            else
            {
                /* 卡尔曼不健康时，遥控仍显示原始气压高度，但垂速清零且高度环禁用。 */
                if (baro_fresh) altitude_cm = MotorTest_ToI16(baro_altitude_m * 100.0f);
                else if (Baro_GetSnapshot(&baro_snap)) altitude_cm = MotorTest_ToI16(baro_snap.altitude_m * 100.0f);
                else altitude_cm = 0;
                vz_cms = 0;
            }

            if ((int32_t)(osKernelGetTickCount() - next_battery_tick) >= 0)
            {
                Battery_Update();
                next_battery_tick = osKernelGetTickCount() + MOTOR_BATTERY_UPDATE_MS;
            }
            ControlLink_Update((uint32_t)osKernelGetTickCount());
            ControlLink_GetSnapshot(&link);
            armed = link.connected && link.command_valid && ((link.command.flags & APEX_CONTROL_FLAG_ARM_REQUESTED) != 0U);
            Baro_SetGroundTrackingAllowed(!armed && (fabsf(body.roll_rate_dps) < MOTOR_GROUND_RATE_LIMIT_DPS) && (fabsf(body.pitch_rate_dps) < MOTOR_GROUND_RATE_LIMIT_DPS) && (fabsf(body.yaw_rate_dps) < MOTOR_GROUND_RATE_LIMIT_DPS) && (fabsf(accel_up_mps2 - alt_est.accel_bias_mps2) < MOTOR_GROUND_ACCEL_LIMIT_MPS2));

            if (!armed)
            {
                throttle_us = MOTOR_PWM_MIN_US;
                AttitudeControl_Reset();
                Motor_Disarm();
                if (was_armed) Blackbox_NotifyDisarmed();
            }
            else
            {
                if (!was_armed)
                {
                    Blackbox_NotifyArmed();
                }
                /* 油门杆直接映射成基础推力，不再经过垂速环。 */
                throttle_us = MotorTest_MapStickToThrottleUs(link.command.throttle);
                /* 低油门先保持四路等值，避免地面姿态误差让单路电机提前抬升。 */
                if (throttle_us <= MOTOR_ATTITUDE_ENABLE_PWM_US)
                {
                    AttitudeControl_Reset();
                    roll_corr_us = 0.0f;
                    pitch_corr_us = 0.0f;
                    yaw_corr_us = 0.0f;
                    mixer_output.motor_1_us = throttle_us;
                    mixer_output.motor_2_us = throttle_us;
                    mixer_output.motor_3_us = throttle_us;
                    mixer_output.motor_4_us = throttle_us;
                    Motor_SetAllPulseUs(throttle_us);
                }
                else if (AttitudeControl_Update((float)throttle_us, &body, control_dt_s, &mixer_output))
                {
                    MotorTest_ApplyMixer(&mixer_output);
                    AttitudeControl_GetLastCorrections(&roll_corr_us, &pitch_corr_us, &yaw_corr_us);
                }
                else
                {
                    roll_corr_us = 0.0f;
                    pitch_corr_us = 0.0f;
                    yaw_corr_us = 0.0f;
                    mixer_output.motor_1_us = throttle_us;
                    mixer_output.motor_2_us = throttle_us;
                    mixer_output.motor_3_us = throttle_us;
                    mixer_output.motor_4_us = throttle_us;
                    Motor_SetAllPulseUs(throttle_us);
                }

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
                sample.altitude_cm = altitude_cm;
                sample.vz_cms = vz_cms;
                sample.target_vz_cms = 0;
                sample.vz_correction_us = 0;
                sample.hover_pwm_us = PidTable_GetMaxThrottleUs();
                sample.altitude_state = 0U;
                sample.armed = 1U;
                sample.connected = link.connected ? 1U : 0U;
                Blackbox_SubmitSample(&sample);
            }
            was_armed = armed;

            Radio_PublishFlightState(body.roll_deg, body.pitch_deg, body.yaw_deg, throttle_us, altitude_cm, vz_cms, true, true, armed);

            osDelay(IMU_SAMPLE_PERIOD_MS);
        }
    }
}
