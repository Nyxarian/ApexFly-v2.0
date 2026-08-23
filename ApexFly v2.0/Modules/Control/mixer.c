/**
 * @file mixer.c
 * @brief 四轴 X 机架混控实现。
 *
 * 公式与老工程 quad_mixer 一致。
 * 正 Roll 输入：右倾力矩；正 Pitch 输入：抬头力矩；正 Yaw 输入：顺时针力矩。
 *
 * M1 后右 CW：正 Roll 减速，正 Pitch 加速，正 Yaw 减速
 * M2 前右 CCW：正 Roll 减速，正 Pitch 减速，正 Yaw 加速
 * M3 后左 CCW：正 Roll 加速，正 Pitch 加速，正 Yaw 加速
 * M4 前左 CW：正 Roll 加速，正 Pitch 减速，正 Yaw 减速
 *
 * 实机用手倾斜时的正确反馈（已测过，不要随便改公式）：
 * 右倾：M1/M2 加快，M3/M4 减慢
 * 左倾：M3/M4 加快，M1/M2 减慢
 * 前倾低头：M2/M4 加快，M1/M3 减慢
 * 后倾抬头：M1/M3 加快，M2/M4 减慢
 *
 * 要得到上面这组现象，调用处必须是：
 * roll_correction  = -Kp * roll_deg
 * pitch_correction = +Kp * pitch_deg
 * 俯仰正号、横滚负号是实测结果。改极性请改调用处，不要改本文件里的加减号。
 */

#include "mixer.h"

#include <stddef.h>

/**
 * @brief 把浮点脉宽限制到范围内并四舍五入成整数微秒。
 */
static uint16_t Mixer_ConvertPulse(float pulse_us, uint16_t minimum_pulse_us, uint16_t maximum_pulse_us)
{
    if (pulse_us < (float)minimum_pulse_us)
    {
        return minimum_pulse_us;
    }
    if (pulse_us > (float)maximum_pulse_us)
    {
        return maximum_pulse_us;
    }
    return (uint16_t)(pulse_us + 0.5f);
}

/**
 * @brief 按一只电机的修正量，收紧四路共用的缩放比例，避免某一路先顶满而把力矩方向打歪。
 */
static void Mixer_UpdateScale(float correction_us, float upward_room_us, float downward_room_us, float *scale)
{
    float candidate_scale;

    if (scale == NULL)
    {
        return;
    }

    if (correction_us > 0.0f)
    {
        candidate_scale = upward_room_us / correction_us;
        if (candidate_scale < *scale)
        {
            *scale = candidate_scale;
        }
    }

    if (correction_us < 0.0f)
    {
        candidate_scale = downward_room_us / (-correction_us);
        if (candidate_scale < *scale)
        {
            *scale = candidate_scale;
        }
    }
}

bool Mixer_Calculate(const MixerInput_t *input, uint16_t minimum_pulse_us, uint16_t maximum_pulse_us, MixerOutput_t *output)
{
    float throttle_us;
    float motor_1_correction_us;
    float motor_2_correction_us;
    float motor_3_correction_us;
    float motor_4_correction_us;
    float correction_scale;
    float upward_room_us;
    float downward_room_us;
    float motor_1_us;
    float motor_2_us;
    float motor_3_us;
    float motor_4_us;
    bool throttle_limited;

    if ((input == NULL) || (output == NULL))
    {
        return false;
    }
    if (maximum_pulse_us <= minimum_pulse_us)
    {
        return false;
    }

    throttle_limited = false;
    throttle_us = input->throttle_us;

    if (throttle_us < (float)minimum_pulse_us)
    {
        throttle_us = (float)minimum_pulse_us;
        throttle_limited = true;
    }
    if (throttle_us > (float)maximum_pulse_us)
    {
        throttle_us = (float)maximum_pulse_us;
        throttle_limited = true;
    }

    /* M1 后右顺时针。 */
    motor_1_correction_us = -input->roll_correction_us + input->pitch_correction_us - input->yaw_correction_us;
    /* M2 前右逆时针。 */
    motor_2_correction_us = -input->roll_correction_us - input->pitch_correction_us + input->yaw_correction_us;
    /* M3 后左逆时针。 */
    motor_3_correction_us = input->roll_correction_us + input->pitch_correction_us + input->yaw_correction_us;
    /* M4 前左顺时针。 */
    motor_4_correction_us = input->roll_correction_us - input->pitch_correction_us - input->yaw_correction_us;

    upward_room_us = (float)maximum_pulse_us - throttle_us;
    downward_room_us = throttle_us - (float)minimum_pulse_us;
    correction_scale = 1.0f;

    Mixer_UpdateScale(motor_1_correction_us, upward_room_us, downward_room_us, &correction_scale);
    Mixer_UpdateScale(motor_2_correction_us, upward_room_us, downward_room_us, &correction_scale);
    Mixer_UpdateScale(motor_3_correction_us, upward_room_us, downward_room_us, &correction_scale);
    Mixer_UpdateScale(motor_4_correction_us, upward_room_us, downward_room_us, &correction_scale);

    if (correction_scale < 0.0f)
    {
        correction_scale = 0.0f;
    }
    if (correction_scale > 1.0f)
    {
        correction_scale = 1.0f;
    }

    motor_1_us = throttle_us + motor_1_correction_us * correction_scale;
    motor_2_us = throttle_us + motor_2_correction_us * correction_scale;
    motor_3_us = throttle_us + motor_3_correction_us * correction_scale;
    motor_4_us = throttle_us + motor_4_correction_us * correction_scale;

    output->motor_1_us = Mixer_ConvertPulse(motor_1_us, minimum_pulse_us, maximum_pulse_us);
    output->motor_2_us = Mixer_ConvertPulse(motor_2_us, minimum_pulse_us, maximum_pulse_us);
    output->motor_3_us = Mixer_ConvertPulse(motor_3_us, minimum_pulse_us, maximum_pulse_us);
    output->motor_4_us = Mixer_ConvertPulse(motor_4_us, minimum_pulse_us, maximum_pulse_us);
    output->correction_scale = correction_scale;
    output->saturated = throttle_limited || (correction_scale < 0.999f);
    return true;
}
