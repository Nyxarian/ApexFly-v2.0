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
 * 6. 250Hz：刷新失联 → 读快照 → 已解锁则把油门杆映射成基础油门（再经速率限制），然后进串级混控
 *
 * 油门映射：最低固定 1000us，最高由 PID 表 APEX_PID_HOVER_PWM_US 决定，默认 1600us。
 * 映射结果再经一阶速率限制（不对称：上升 800us/s、下降 300us/s），消除摇杆快动造成的推力阶跃，
 * 让降落从"瞬间失去升力"变成"缓慢下沉"，给飞行员补油的反应时间。
 * 注意：速率限制只改变油门的"变化快慢"，不改变稳态映射；摇杆停在悬停点以下仍会稳态下坠，
 * 那属于定高的职责。将来接入定高（激光测距）时，需评估是否与本层斜率限制叠加。
 * 气压计模块已于 2026-09-11 清除，高度遥测先固定填 0，不读 I2C2。
 * 将来接入激光测距做定高时，在这里把高度接进来；届时注意本文件的油门速率限制
 * 会与新控制器自带的斜率限制叠加，需要评估是否删掉一层。
 * 解锁后黑匣子样本把磁航向写进 target_vz_cms；电台遥测另填 home_distance_m。
 * 偏航杆中位锁磁航向，出死区切速率。横滚俯仰杆映射成目标角，满杆约 ±10°，再加 Trim。YAW ANGLE 为 0 时偏航外环不出力。
 */

#include "motor_test.h"

#include "attitude_control.h"
#include "battery.h"
#include "blackbox.h"
#include "cmsis_os.h"
#include "control_link.h"
#include "FreeRTOS.h"
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
#include "semphr.h"
#include "task.h"
#include "tof.h"
#include <string.h>

#define ESC_ARM_WAIT_MS 3000U
#define IMU_BIAS_SAMPLE_COUNT 500U
#define IMU_LEVEL_SAMPLE_COUNT 200U
/* 控制环 250Hz（4ms），一次 DRDY = 一帧控制。本值只用于 DRDY 兜底与回退路径。 */
#define IMU_SAMPLE_PERIOD_MS 4U
/* 上电初值，实测 dt 之后会被真实周期覆盖（传感器 250Hz 实际约 4008us）。 */
#define IMU_DELTA_TIME_S 0.004f
#define MOTOR_BATTERY_UPDATE_MS 1000U
/* dt 合理性区间：正常 250Hz 约 4008us，DRDY 兜底约 8ms，两者都要落在区间内。 */
#define MOTOR_DT_MIN_S 0.002f
#define MOTOR_DT_MAX_S 0.012f
/* 姿态环启用门槛与混控有效下限统一，避免两处不一致。 */
#define MOTOR_ATTITUDE_ENABLE_PWM_US MIXER_PWM_EFFECTIVE_MIN_US
/* I2C 连续失败多少次才判定总线真故障并上锁重初始化（20 拍 = 80ms，250Hz）。 */
#define MOTOR_I2C_FAIL_LIMIT 20U
/* 等 DRDY 脉冲的超时，单位毫秒。取两倍周期：脉冲丢了也照常跑一拍，绝不死等。 */
#define MOTOR_DRDY_TIMEOUT_MS 8U
/* 黑匣子记录分频：1 = 每拍都记（250Hz，约 8.3 分钟/场次）；
   2 = 隔一拍记（125Hz，约 16 分钟/场次）。调参期间用 1 拿最密的数据。 */
#define MOTOR_BLACKBOX_SAMPLE_DIV 1U

/* 油门速率限制（不对称）：上升快、下降慢。
   上升 800us/s：满行程 1000->1600 约 0.75s，保留"爬升要跟手"的能力。
   下降 300us/s：满行程 1600->1000 约 2.0s，把"瞬间掉升力"变成"缓慢下沉"。
   依据：悬停附近推力斜率约 3.1gf/us，300us/s 下每拍(4ms)只变 1.2us，
   远小于"温和修正"所需的 16us 量级，不影响姿态环正常补油。 */
#define MOTOR_THROTTLE_SLEW_UP_US_PER_S   800.0f
#define MOTOR_THROTTLE_SLEW_DOWN_US_PER_S 300.0f

/* 微秒时基：用 DWT 周期计数器。170MHz 下 CYCCNT 约 25s 回绕，
   dt 的差值必须在原始计数器域做无符号减法（先除再减会在回绕处出错）。 */
static void MotorTest_TimebaseInit(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/* 数据就绪信号：EXTI0 中断里释放，控制任务里等。
   用静态分配的二值信号量，不占 FreeRTOS 堆；中断里只发信号，不做任何 I2C。 */
static StaticSemaphore_t imu_drdy_semaphore_storage;
static SemaphoreHandle_t imu_drdy_semaphore;

/* 油门速率限制器的当前值（跨拍保持）。因为是速率限制，本模块不再是无状态的。
   复位时机共三处，均与既有的 throttle_us = MOTOR_PWM_MIN_US 成对出现：
   ①任务初始化 ②外层循环重初始化 IMU ③未解锁分支。
   漏掉第 ③ 处会导致解锁瞬间从上一轮的残留值开始爬升。 */
static float throttle_slew_us = (float)MOTOR_PWM_MIN_US;

/**
 * @brief IMU 数据就绪中断回调，由 EXTI0 的 HAL_GPIO_EXTI_Callback 调用。
 *
 * 运行在中断上下文：只释放信号量，不做 I2C、不做任何阻塞操作。
 * 信号量尚未创建时直接返回，避免上电早期的中断打到空句柄上。
 */
void MotorTest_NotifyImuDrdyFromISR(void)
{
    BaseType_t higher_priority_task_woken = pdFALSE;

    if (imu_drdy_semaphore == NULL) return;

    xSemaphoreGiveFromISR(imu_drdy_semaphore, &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
}

/**
 * @brief 等一次数据就绪脉冲，一次 DRDY 对应一帧控制。
 *
 * 等不到就按超时返回，让控制环照常跑一拍：
 * INT 线断了或脉冲被吃掉时，飞机不会停在信号量上等死。
 * 台架实测（250Hz/8ms 超时）：DRDY 完全不来时循环稳定跑在 125Hz 不卡死；
 * 冷启动后第一次等待可能超时一次（芯片刚使能中断，第一个脉冲要几毫秒才来），之后不再发生。
 * @return true 表示由中断唤醒；false 表示超时或信号量不可用。
 */
static bool MotorTest_WaitImuDrdy(void)
{
    if (imu_drdy_semaphore == NULL)
    {
        /* 静态分配下不会失败，这里只是兜底：退回定时等待，别让循环停掉。 */
        osDelay(IMU_SAMPLE_PERIOD_MS);
        return false;
    }

    return xSemaphoreTake(imu_drdy_semaphore, pdMS_TO_TICKS(MOTOR_DRDY_TIMEOUT_MS)) == pdTRUE;
}

/**
 * @brief 把混控算出的四路脉宽写到对应电机。
 */
static void MotorTest_ApplyMixer(MixerOutput_t *output)
{
    /* 有效下限由 Mixer_Calculate 统一处理（MIXER_PWM_EFFECTIVE_MIN_US），
       这里不再逐路裁剪，避免破坏四路差速关系。 */
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

static float MotorTest_Absf(float value)
{
    return (value < 0.0f) ? -value : value;
}

/**
 * @brief 对基础油门做一阶速率限制（不对称），消除摇杆快动造成的推力阶跃。
 * @param current_us 上一拍已输出的油门，单位微秒。
 * @param target_us  本拍由摇杆映射得到的目标油门，单位微秒。
 * @param dt_s       本拍实际周期，单位秒（已由控制环限幅到 2~12ms）。
 * @return 限速后的油门，单位微秒，落在 [MOTOR_PWM_MIN_US, MOTOR_PWM_MAX_US]。
 * @note 上升用 MOTOR_THROTTLE_SLEW_UP_US_PER_S，下降用 _DOWN_。
 *       差值小于一步时直接到位，避免无谓地缓慢逼近。
 *       本函数不解决"摇杆停在悬停点以下"的稳态下坠——那属于定高的职责。
 */
static float MotorTest_ThrottleRampLimit(float current_us, float target_us, float dt_s)
{
    float max_step_us;
    float output_us;

    if (dt_s <= 0.0f)
    {
        return current_us;
    }

    max_step_us = (target_us >= current_us)
                      ? (MOTOR_THROTTLE_SLEW_UP_US_PER_S * dt_s)
                      : (MOTOR_THROTTLE_SLEW_DOWN_US_PER_S * dt_s);

    if (target_us > (current_us + max_step_us))
    {
        output_us = current_us + max_step_us;
    }
    else if (target_us < (current_us - max_step_us))
    {
        output_us = current_us - max_step_us;
    }
    else
    {
        output_us = target_us;
    }

    if (output_us < (float)MOTOR_PWM_MIN_US) output_us = (float)MOTOR_PWM_MIN_US;
    if (output_us > (float)MOTOR_PWM_MAX_US) output_us = (float)MOTOR_PWM_MAX_US;

    return output_us;
}

/* 水平零点采集门禁：机体必须静止、接近水平、加计模长正常。
   任一不满足就清空累加器重新开始，避免把斜放或晃动采成永久零点。 */
static bool MotorTest_LevelSampleValid(const MPU6050_Data_t *data, const IMU_BodyAttitude_t *body)
{
    float accel_norm_sq;

    accel_norm_sq = (data->accel_x_g * data->accel_x_g) + (data->accel_y_g * data->accel_y_g) + (data->accel_z_g * data->accel_z_g);
    if ((accel_norm_sq < 0.9025f) || (accel_norm_sq > 1.2100f)) return false;
    if ((MotorTest_Absf(body->roll_deg) > 8.0f) || (MotorTest_Absf(body->pitch_deg) > 8.0f)) return false;
    if ((MotorTest_Absf(data->gyro_x_dps) > 2.0f) || (MotorTest_Absf(data->gyro_y_dps) > 2.0f) || (MotorTest_Absf(data->gyro_z_dps) > 2.0f)) return false;
    return true;
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
    uint16_t throttle_target_us;
    uint32_t last_imu_cycles;
    uint32_t next_battery_tick;
    uint32_t imu_cycles;
    uint32_t dt_us;
    uint32_t blackbox_sample_divider;
    uint32_t i2c_fail_streak;
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
    /* 先建好 DRDY 信号量，之后 EXTI0 一来脉冲就能被接住。 */
    imu_drdy_semaphore = xSemaphoreCreateBinaryStatic(&imu_drdy_semaphore_storage);
    PidTable_Init();
    Battery_Init();
    memset(&body, 0, sizeof(body));
    throttle_us = MOTOR_PWM_MIN_US;
    throttle_slew_us = (float)MOTOR_PWM_MIN_US;
    throttle_target_us = MOTOR_PWM_MIN_US;
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
        dt_us = 0U;
        blackbox_sample_divider = 0U;
        next_battery_tick = osKernelGetTickCount();
        control_dt_s = IMU_DELTA_TIME_S;
        throttle_slew_us = (float)MOTOR_PWM_MIN_US;
        i2c_fail_streak = 0U;
    
        for (;;)
        {
            /* 等 MPU6050 的数据就绪脉冲：一次 DRDY = 一帧控制，每拍必然是新样本。
               等不到就超时兜底照常跑一拍，INT 线断了也不会死锁。 */
            (void)MotorTest_WaitImuDrdy();

            /* 本拍起点：既用来量周期，也用来量整拍耗时（读 IMU + 解算 + 混控 + 遥测）。
               在原始计数器域做无符号减法（天然处理回绕），再换算成微秒。 */
            imu_cycles = DWT->CYCCNT;
            if (last_imu_cycles != 0U)
            {
                dt_us = (uint32_t)(imu_cycles - last_imu_cycles) / (SystemCoreClock / 1000000U);
                control_dt_s = MotorTest_Clampf((float)dt_us * 0.000001f, MOTOR_DT_MIN_S, MOTOR_DT_MAX_S);
            }
            last_imu_cycles = imu_cycles;

            status = MPU6050_ReadData(&imu_device, &imu_data);
            if (status != MPU6050_STATUS_OK)
            {
                /* 单次 I2C 错误只丢本拍：电机保持上一拍输出，下一拍自动重试。
                   连续多次失败才认为总线真故障，走上锁保护并重新初始化 IMU。 */
                i2c_fail_streak++;
                if (i2c_fail_streak < MOTOR_I2C_FAIL_LIMIT)
                {
                    continue;
                }
                if (was_armed) Blackbox_NotifyDisarmed();
                Motor_Disarm();
                Radio_PublishFlightState(0.0f, 0.0f, 0.0f, MOTOR_PWM_MIN_US, 0, 0, false, true, false);
                break;
            }
            i2c_fail_streak = 0U;

            if (!bias_ready)
            {
                /* 采集期间也发布一次状态，让遥控器看到 IMU 尚未就绪。 */
                Radio_PublishFlightState(0.0f, 0.0f, 0.0f, MOTOR_PWM_MIN_US, 0, 0, false, true, false);

                if (!IMU_GyroBias_Update(&bias_collector, &imu_data))
                {
                    continue;
                }

                bias_ready = true;
                Motor_Disarm();
                AttitudeControl_Reset();
                continue;
            }

            /* 控制环内禁止任何阻塞打印：250Hz 下一拍只有 4ms，一行长日志就能吃掉采样、
               并把 dt 统计弄脏。需要观察姿态数据时看黑匣子样本，
               或在本文件外临时挂调试代码，验证完立即删除。 */
            IMU_GyroBias_Apply(&bias_collector.bias, &imu_data);
            if (!IMU_Attitude_Update(&attitude, &imu_data, control_dt_s))
            {
                continue;
            }
            IMU_Attitude_MapToBody(&attitude, &imu_data, &body);

            if (!level_ready)
            {
                if (!MotorTest_LevelSampleValid(&imu_data, &body))
                {
                    /* 门禁不通过：清空累加器重来，避免斜放/晃动污染零点。 */
                    IMU_Level_Start(&level_collector, IMU_LEVEL_SAMPLE_COUNT);
                    continue;
                }

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

            /* 高度直接读 ToF 任务算好的结果：只做一次 32 位整数读取，
               不碰 I2C2 锁，绝不会拖慢 250Hz 控制环。
               该值是实测距离换算的厘米数，不含零点相减，任何时候（含上锁）都有效；
               测不到时 ToF 内部会保持上一次的值。垂速暂不做，固定 0。 */
            altitude_cm = ToF_GetAltitudeCm();
            vz_cms = 0;

            if ((int32_t)(osKernelGetTickCount() - next_battery_tick) >= 0)
            {
                Battery_Update();
                next_battery_tick = osKernelGetTickCount() + MOTOR_BATTERY_UPDATE_MS;
            }
            ControlLink_Update((uint32_t)osKernelGetTickCount());
            ControlLink_GetSnapshot(&link);
            armed = link.connected && link.command_valid && ((link.command.flags & APEX_CONTROL_FLAG_ARM_REQUESTED) != 0U);

            /* 内环（角速度环）未启用时拒绝解锁：此时姿态环没有阻尼，起飞必然发散。 */
            if (armed && !AttitudeControl_IsInnerLoopEnabled())
            {
                armed = false;
            }

            if (!armed)
            {
                throttle_us = MOTOR_PWM_MIN_US;
                throttle_slew_us = (float)MOTOR_PWM_MIN_US;
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
                /* 油门杆先映射成基础推力（映射本身与改动前一致），再经速率限制，不经过垂速环。
                   速率不对称：上升 800、下降 300 us/s。
                   状态回喂的是限速后的【浮点值】，不是取整后的整数——若回喂整数，
                   每拍丢弃小数会系统性吃掉速率（实测 800->750、300->250）。 */
                mag_heading_cdeg = MotorTest_MagHeadingCdeg(&mag_calibration_valid);
                PidTable_GetAttitudeTrimDeg(&roll_trim_deg, &pitch_trim_deg);
                throttle_target_us = MotorTest_MapStickToThrottleUs(link.command.throttle);
                throttle_slew_us = MotorTest_ThrottleRampLimit(throttle_slew_us, (float)throttle_target_us, control_dt_s);
                throttle_us = (uint16_t)(throttle_slew_us + 0.5f);
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
                /* 记录分频：250Hz 全记约 8.3 分钟/场次，分频 2 可翻倍到约 16 分钟。 */
                blackbox_sample_divider++;
                if (blackbox_sample_divider >= MOTOR_BLACKBOX_SAMPLE_DIV)
                {
                    blackbox_sample_divider = 0U;
                    Blackbox_SubmitSample(&sample);
                }
            }
            was_armed = armed;

            Radio_PublishFlightState(body.roll_deg, body.pitch_deg, body.yaw_deg, throttle_us, altitude_cm, vz_cms, true, AttitudeControl_IsInnerLoopEnabled(), armed);
        }
    }
}
