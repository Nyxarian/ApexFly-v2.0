/**
 * @file mixer.h
 * @brief 四轴 X 机架混控。
 *
 * 把基础油门和横滚、俯仰、偏航修正量，换成四路电机 PWM 脉宽。
 * 本模块只做数学计算，不操作 TIM8，也不做 PID。
 *
 * 电机布局（机头朝上，从上往下看，转向已实测对齐）：
 * M4 前左，顺时针 CW
 * M2 前右，逆时针 CCW
 * M3 后左，逆时针 CCW
 * M1 后右，顺时针 CW
 *
 * 飞控正方向与当前 IMU 映射一致：
 * 正横滚 = 右倾 / 往右飞
 * 正俯仰 = 抬头 / 往后飞
 * 正偏航 = 顺时针 / 机头右转
 *
 * ---------- 实机联动已确认，改混控前先看这里 ----------
 * 拆桨、基础油门 1250us，用手倾斜机体，低侧应加快、高侧应减慢。
 *
 * 右倾（右边低）：M1 后右、M2 前右加快；M3 后左、M4 前左减慢。
 * 左倾（左边低）：M3 后左、M4 前左加快；M1、M2 减慢。
 * 前倾 / 低头（机头低）：M2 前右、M4 前左加快；M1、M3 减慢。
 * 后倾 / 抬头（机尾低）：M1 后右、M3 后左加快；M2、M4 减慢。
 *
 * 测试任务把姿态角送进混控时，必须用下面这组符号才和上面现象一致：
 * roll_correction_us  = -Kp * roll_deg
 * pitch_correction_us = +Kp * pitch_deg
 * yaw_correction_us   = 0
 *
 * 注意：俯仰是正号，横滚是负号。不要改回两边都用负号，
 * 否则前倾后倾反馈会再次相反，又要拆桨重测。
 * Mixer_Calculate() 内部公式不要为了“好记”去改；要改极性，改调用处的 Kp 符号。
 */

#ifndef MIXER_H
#define MIXER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 混控输入
typedef struct
{
    /* 四路共同的基础油门，单位微秒。 */
    float throttle_us;
    /* 横滚修正，正值产生右倾力矩，单位微秒。 */
    float roll_correction_us;
    /* 俯仰修正，正值产生抬头力矩，单位微秒。 */
    float pitch_correction_us;
    /* 偏航修正，正值产生顺时针力矩，单位微秒。 */
    float yaw_correction_us;
} MixerInput_t;

// 混控输出
typedef struct
{
    /* M1 后右，顺时针。 */
    uint16_t motor_1_us;
    /* M2 前右，逆时针。 */
    uint16_t motor_2_us;
    /* M3 后左，逆时针。 */
    uint16_t motor_3_us;
    /* M4 前左，顺时针。 */
    uint16_t motor_4_us;
    /* 三轴修正的统一缩放，正常为 1.0，饱和时小于 1.0。 */
    float correction_scale;
    /* 基础油门被裁切，或三轴修正被缩小。 */
    bool saturated;
} MixerOutput_t;

/**
 * @brief 根据基础油门和三轴修正计算四路电机脉宽。
 * @param input 混控输入。
 * @param minimum_pulse_us 允许的最小脉宽，通常 1000。
 * @param maximum_pulse_us 允许的最大脉宽，通常 2000。
 * @param output 四路结果。
 * @return 参数合法并计算完成返回 true。
 */
bool Mixer_Calculate(const MixerInput_t *input,
                     uint16_t minimum_pulse_us,
                     uint16_t maximum_pulse_us,
                     MixerOutput_t *output);

#ifdef __cplusplus
}
#endif

#endif
