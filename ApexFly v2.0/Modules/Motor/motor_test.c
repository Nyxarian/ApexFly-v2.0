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
 * 气压计任务已停用，I2C2 只给磁力计。高度遥测先填 0，altitude_est 源码保留。
 * 解锁后黑匣子样本把磁航向写进 target_vz_cms；电台遥测另填 home_distance_m。
 * 偏航杆中位锁磁航向，出死区切速率。横滚俯仰杆映射成目标角，满杆约 ±10°，再加 Trim。YAW ANGLE 为 0 时偏航外环不出力。
 */

#include "motor_test.h"

#include "attitude_control.h"
#include "battery.h"
#include "blackbox.h"
#include "cmsis_os.h"
#include "control_link.h"
#include "i2c.h"
#include "imu_attitude.h"
#include "imu_bias.h"
#include "imu_level.h"
#include "mag.h"
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
#define MOTOR_BATTERY_UPDATE_MS 1000U
#define MOTOR_DT_MIN_S 0.005f
#define MOTOR_DT_MAX_S 0.03f
#define MOTOR_ATTITUDE_ENABLE_PWM_US 1050U

/* 微秒时基：用 DWT 周期计数器。170MHz 下 CYCCNT 约 25s 回绕，
   dt 的差值必须在原始计数器域做无符号减法（先除再减会在回绕处出错）。 */
static void MotorTest_TimebaseInit(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

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

static int16_t MotorTest_MagHeadingCdeg(bool *calibration_valid)
{
    MagSnapshot_t mag;
    float heading_deg;

    if (calibration_valid != NULL) *calibration_valid = false;
    if (!Mag_GetSnapshot(&mag) || !mag.valid) return -1;
    if (calibration_valid != NULL) *calibration_valid = mag.calibration_valid;
    heading_deg = mag.calibration_valid ? mag.cal_heading_deg : mag.body_heading_deg;
    if (heading_deg < 0.0f) heading_deg += 360.0f;
    if (heading_deg >= 360.0f) heading_deg -= 360.0f;
    return MotorTest_ToCenti(heading_deg);
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
    uint32_t last_imu_cycles;
    uint32_t next_battery_tick;
    uint32_t imu_cycles;
    uint32_t next_tick;
    uint32_t dt_us;
    float control_dt_s;
    float roll_corr_us;
    float pitch_corr_us;
    float yaw_corr_us;
    int32_t altitude_cm;
    int16_t vz_cms;
    int16_t mag_heading_cdeg;
    bool mag_calibration_valid;
    float roll_trim_deg;
    float pitch_trim_deg;

    (void)argument;
    MotorTest_TimebaseInit();
    PidTable_Init();
    Battery_Init();
    memset(&body, 0, sizeof(body));
    throttle_us = MOTOR_PWM_MIN_US;
    altitude_cm = 0;
    vz_cms = 0;
    mag_heading_cdeg = -1;
    mag_calibration_valid = false;
    roll_trim_deg = 0.0f;
    pitch_trim_deg = 0.0f;

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
        bias_ready = false;
        level_ready = false;
        was_armed = false;
        last_imu_cycles = 0U;
        next_battery_tick = osKernelGetTickCount();
        next_tick = osKernelGetTickCount() + IMU_SAMPLE_PERIOD_MS;
        control_dt_s = IMU_DELTA_TIME_S;
    
        for (;;)
        {
            /* 等到本拍的计划时刻，保证周期稳定、不受单拍耗时影响；
               落后超过一拍时重新对齐，避免积压后连续追赶。
               注意：这里必须用有符号差值判断，无符号相减在“尚未到点”时会被误判为落后。 */
            if ((int32_t)(osKernelGetTickCount() - next_tick) > (int32_t)IMU_SAMPLE_PERIOD_MS)
            {
                next_tick = osKernelGetTickCount();
            }
            (void)osDelayUntil(next_tick);
            next_tick += IMU_SAMPLE_PERIOD_MS;

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
                    continue;
                }

                bias_ready = true;
                Motor_Disarm();
                AttitudeControl_Reset();
                continue;
            }

            imu_cycles = DWT->CYCCNT;
            if (last_imu_cycles != 0U)
            {
                /* 先在原始计数器域做无符号减法（天然处理回绕），再换算成微秒。 */
                dt_us = (uint32_t)(imu_cycles - last_imu_cycles) / (SystemCoreClock / 1000000U);
                control_dt_s = MotorTest_Clampf((float)dt_us * 0.000001f, MOTOR_DT_MIN_S, MOTOR_DT_MAX_S);
            }
            last_imu_cycles = imu_cycles;

            /* 控制环内禁止任何阻塞打印：每拍约 6ms 的串口发送会把 100Hz 拖到 60Hz 且抖动。
               需要观察 IMU 数据时看黑匣子样本，或在本文件外临时挂调试代码。 */
            IMU_GyroBias_Apply(&bias_collector.bias, &imu_data);
            if (!IMU_Attitude_Update(&attitude, &imu_data, control_dt_s))
            {
                continue;
            }
            IMU_Attitude_MapToBody(&attitude, &imu_data, &body);

            if (!level_ready)
            {
                if (!IMU_Level_Update(&level_collector, &body))
                {
                    continue;
                }

                level_ready = true;
                Motor_Disarm();
                AttitudeControl_Reset();
                continue;
            }

            IMU_Level_Apply(&level_collector.offset, &body);

            /* 气压计停用后高度和垂速遥测固定为 0，不读 I2C2。 */
            altitude_cm = 0;
            vz_cms = 0;

            if ((int32_t)(osKernelGetTickCount() - next_battery_tick) >= 0)
            {
                Battery_Update();
                next_battery_tick = osKernelGetTickCount() + MOTOR_BATTERY_UPDATE_MS;
            }
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
                if (!was_armed)
                {
                    Blackbox_NotifyArmed();
                }
                /* 油门杆直接映射成基础推力，不再经过垂速环。 */
                mag_heading_cdeg = MotorTest_MagHeadingCdeg(&mag_calibration_valid);
                PidTable_GetAttitudeTrimDeg(&roll_trim_deg, &pitch_trim_deg);
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
                else if (AttitudeControl_Update((float)throttle_us, &body, (mag_heading_cdeg < 0) ? 0.0f : ((float)mag_heading_cdeg / 100.0f), mag_calibration_valid && (mag_heading_cdeg >= 0), link.command.yaw, link.command.roll, link.command.pitch, roll_trim_deg, pitch_trim_deg, control_dt_s, &mixer_output))
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
                sample.target_vz_cms = mag_heading_cdeg;
                sample.vz_correction_us = 0;
                sample.hover_pwm_us = PidTable_GetMaxThrottleUs();
                sample.altitude_state = 0U;
                sample.armed = 1U;
                sample.connected = link.connected ? 1U : 0U;
                Blackbox_SubmitSample(&sample);
            }
            was_armed = armed;

            Radio_PublishFlightState(body.roll_deg, body.pitch_deg, body.yaw_deg, throttle_us, altitude_cm, vz_cms, true, true, armed);
        }
    }
}
