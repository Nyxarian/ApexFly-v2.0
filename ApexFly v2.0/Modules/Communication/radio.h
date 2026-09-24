/**
 * @file radio.h
 * @brief 飞机端 LoRa 电台任务：收遥控控制帧，立刻回遥测。
 *
 * 本模块只读写 RA-01SC 和 control_link，不写 TIM8。
 * 电机任务读 control_link 快照，需要回传的姿态/油门用 Radio_PublishFlightState() 交给本模块。
 *
 * 空中时序必须和遥控器一致：
 * 收控制帧 → 立刻发遥测；收 PID 命令 → 立刻发 PID 应答；TX_DONE 后再 StartReceive。
 * 当前硬件：SPI1，NSS=PA4，SCK=PA5，MISO=PA6，MOSI=PA7，
 * DIO1=PB2，NRST=PE7，TXEN=PE8，RXEN=PE9，BUSY=PE10。
 */

#ifndef RADIO_H
#define RADIO_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    /* 机体系横滚角，单位度。 */
    float roll_deg;
    /* 机体系俯仰角，单位度。 */
    float pitch_deg;
    /* 机体系偏航角，单位度。 */
    float yaw_deg;
    /* 当前基础油门，单位微秒。 */
    uint16_t throttle_pwm_us;
    /* 当前高度，单位厘米。 */
    int32_t altitude_cm;
    /* 当前垂直速度，单位厘米每秒。 */
    int16_t vertical_speed_cms;
    /* true 表示姿态和陀螺仪数据已经准备好。 */
    bool imu_ready;
    /* true 表示电机 PWM 已经准备好。 */
    bool motor_ready;
    /* true 表示当前已解锁。 */
    bool armed;
    /* 实际生效的飞行模式，取 VerticalControlMode_t 的值；bit7 置位表示
       "遥控请求被拒绝或已自动退出"。注意是**实际生效**而不是遥控请求的那个：
       定高因故障自动退出时，遥控器上要立刻看到模式变回去，飞行员才有明确反馈。 */
    uint8_t flight_mode;
    /* 实际定点状态，编码见 apex_protocol.h。 */
    uint8_t position_state;
} RadioFlightStateSnapshot_t;

/**
 * @brief 读取最新飞控状态快照，供调试和遥测任务使用。
 * @param state 输出快照地址。
 * @return 成功复制返回 true，参数为空返回 false。
 */
bool Radio_GetFlightStateSnapshot(RadioFlightStateSnapshot_t *state);

/**
 * @brief 电机任务把最新飞控状态交给电台，用来填遥测。
 * @param roll_deg 机体系横滚，单位度。
 * @param pitch_deg 机体系俯仰，单位度。
 * @param yaw_deg 机体系偏航，单位度。
 * @param throttle_pwm_us 当前基础油门，单位微秒。
 * @param altitude_cm 卡尔曼离地高度，单位厘米，向上为正。
 * @param vertical_speed_cms 卡尔曼垂直速度，单位厘米每秒，向上为正。
 * @param imu_ready 陀螺零偏已采完。
 * @param motor_ready 电调 PWM 已启动。
 * @param armed 遥控已请求解锁且链路未失联。
 */
void Radio_PublishFlightState(float roll_deg, float pitch_deg, float yaw_deg, uint16_t throttle_pwm_us, int32_t altitude_cm, int16_t vertical_speed_cms, bool imu_ready, bool motor_ready, bool armed);

/**
 * @brief 电机任务把**实际生效**的飞行模式交给电台，用来填遥测的 flight_mode 字段。
 *
 * 与 Radio_PublishFlightState() 分开单独发布，是为了不改动那 5 处既有调用点。
 * 协议里这个字节原本是把遥控请求原样回显，改成回显实际模式之后，
 * "我按了但飞机没照做"这件事才在遥控器上看得见。
 *
 * @param flight_mode 实际模式；bit7 置位表示请求被拒绝或已自动退回普通模式。
 */
void Radio_PublishFlightMode(uint8_t flight_mode);
/* 发布实际定点状态，遥测复用 flight_mode 的 bit4~6。 */
void Radio_PublishPositionState(uint8_t position_state);

/**
 * @brief 电台任务入口。绑定 SPI1 后连续接收，失败则间隔重试。
 * @param argument 未使用。
 */
void Radio_Task(void *argument);

#ifdef __cplusplus
}
#endif

#endif
