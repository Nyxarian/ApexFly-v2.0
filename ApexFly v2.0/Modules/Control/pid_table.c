/**
 * @file pid_table.c
 * @brief RAM 参数表实现：30 个协议 ID 与姿态串级、满杆油门上限互转。
 *
 * 增益和限幅改完后只刷新 Pid_t 的数字并 Reset 积分，不重新 Init 整轴以外的状态机。
 * 限幅语义（#26）：遥控只能设 out_max（MAX_RATE_DPS / MAX_CORRECTION_US）；
 * 积分限幅由 attitude_control.c 按 out_max 的固定比例推导，本文件不再写 i_limit。
 * trim 先存着给 GET/SET 显示，不送进控制目标角。
 * APEX_PID_HOVER_PWM_US 这个枚举名是历史遗留：它表示的是【满杆对应的最高油门】，
 * 不是"悬停需要多少油门"。内部字段已改名成 max_throttle_us，
 * 只有协议枚举名保留旧写法，免得飞机与遥控两端错位。
 * 垂速 PID 项（ID 25~28）当前只保留读写通道，不参与电机输出；接入定高后它们就是垂速内环的三增益。
 * MAG_CAL_STOP 成功后由黑匣子把硬铁偏移写入 0x002000；失败不覆盖旧值。
 */

#include "pid_table.h"

#include "attitude_control.h"
#include "blackbox.h"
#include "FreeRTOS.h"
#include "mag.h"
#include "task.h"
#include <stddef.h>
#include <string.h>

/* 参数集版本号，随 PID 回包发给遥控器（遥控端当前不判它，仅作记录/排障线索）。
   ⚠️ 2026-09-15 由 2 抬到 3：参数个数 30 → 33（新增高度环三项），
      抬上去之后回包里就能看出"这一版参数表是哪一代"。 */
#define PID_TABLE_PARAMETER_VERSION 3U
#define PID_TABLE_GAIN_MAX 50.0f
#define PID_TABLE_VZ_KP_MAX 500.0f
#define PID_TABLE_VZ_KI_MAX 100.0f
#define PID_TABLE_VZ_KD_MAX 50.0f
/* 高度环（外环）三增益量程。外环输入是高度误差（cm）、输出是目标垂速（cm/s），
   所以它的数量级比垂速环小得多（方案 §3.3 定的 Kp_h = 0.6），量程也按此收紧：
   给 5 / 2 / 5 已经足够宽，同时让"手滑多按一位"更容易被 OUT_OF_RANGE 挡住。 */
#define PID_TABLE_H_KP_MAX 5.0f
#define PID_TABLE_H_KI_MAX 2.0f
#define PID_TABLE_H_KD_MAX 5.0f
#define PID_TABLE_MAX_THROTTLE_MIN_US 1200.0f
#define PID_TABLE_MAX_THROTTLE_MAX_US 2000.0f
#define PID_TABLE_RATE_MIN 20.0f
#define PID_TABLE_RATE_MAX 300.0f
#define PID_TABLE_CORR_MIN 20.0f
#define PID_TABLE_CORR_MAX 300.0f
#define PID_TABLE_TRIM_ABS_MAX 10.0f

/* 高度轴参数组：**两个环** + 满杆最高油门。
   ⚠️⚠️ 两个环不要混（这是 2026-09-15 加外环时特意分开命名的原因）：
       vz_*  = **垂速环（内环）**：输入垂速误差 cm/s → 输出油门修正量 µs。H9 已接入。
       h_*   = **高度环（外环）**：输入高度误差 cm   → 输出目标垂速 cm/s。H13 才接入。
   在 H13 之前 h_* 只是"协议 / 遥控可读写、可存 Flash、可进黑匣子"的参数，
   **不参与任何控制** ⇒ 现在改它们不影响飞行，也不会被任何代码读到。
   满杆油门 max_throttle_us 不属于任何一个环，它是两个模式共用的油门上界。 */
typedef struct
{
    float vz_kp;
    float vz_ki;
    float vz_kd;
    float h_kp;
    float h_ki;
    float h_kd;
    /* 满杆对应的最高油门，单位微秒。注意：不是"悬停需要多少油门"。 */
    float max_throttle_us;
} PidTableAltitudeParams_t;

#define PID_TABLE_ALTITUDE_DEFAULT_VZ_KP            100.0f
#define PID_TABLE_ALTITUDE_DEFAULT_VZ_KI            0.0f
#define PID_TABLE_ALTITUDE_DEFAULT_VZ_KD            0.0f
/* 外环 Kp 默认值。量纲：高度误差 cm → 目标垂速 cm/s，所以单位是 1/s。
   ⚠️ 2026-09-15（H13）由方案 §3.3 的设计值 **0.6 降到 0.3**，理由是**回路延迟**：
      ToF 一步 20ms、4 步一个测量周期 = 83ms，再加差分垂速、低通、限速器
      （±50µs 满幅修正要走 125ms）；§3.8 的滞后预算是 60ms。
      手算相位裕度：悬停点 d(推力比)/du = 2/415 ⇒ 修正量每变 1µs，加速度变 4.7cm/s²；
      内环 corr = −Kp_v·vz 的闭环时间常数 = 1/(Kp_v × 4.7)，Kp_v=20 时只有 11ms，
      比延迟快 8~15 倍 ⇒ 相位裕度必然为负。
      仿真（`sim_h13_takeoff.js`）确认：60~80ms 延迟下 `vz_kp=20 / h_kp=0.6`
      **进入极限环**（末段垂速 ±27cm/s）；延迟 0 时同一组参数完全稳定。
   ⚠️⚠️ **但后来把 vz_kp 单独扫一遍才发现：真正的瓶颈是内环，不是外环**
      （延迟 80/100/120ms 下 `vz_kp ≤ 2` 稳、`≥ 3` 就振荡；而 `vz_kp=2` 时
      `h_kp` 取 0.2~0.8 全都稳）。所以**这一条不是因为"延迟"才取 0.3**，
      而是"首飞先保守、之后往上加"—— 结论不变、理由更正，详见
      `vertical_control.h` 外环常量段里的那段更正说明。
   ⇒ **默认值必须是"安全值"而不是"设计值"**：擦 Flash / 上电默认时，
     飞机会用这个值起飞。往保守方向错的代价只是"反应迟钝"，往激进方向错是炸机。
   ⚠️ 与 MAX THROTTLE 同一个坑：**Flash 里存着旧值的话 `LOAD_SAVED` 会把 0.6 带回来**，
      首飞前要在遥控器上确认这一项的显示值（或者手动设成 0.3）。
   ⚠️ 默认值**不是 0**：0 的含义是"外环关闭"，那时飞机只靠内环阻尼、
      以一个恒定速度缓慢下沉 —— 比"迟钝"危险得多。
   KI/KD 默认 0（方案里外环就是 P）。 */
#define PID_TABLE_ALTITUDE_DEFAULT_H_KP             0.3f
#define PID_TABLE_ALTITUDE_DEFAULT_H_KI             0.0f
#define PID_TABLE_ALTITUDE_DEFAULT_H_KD             0.0f
/* 满杆对应的最高油门，单位微秒。这是**普通模式**杆推到底给的值。
   ⚠️ 2026-09-15 由 1500 改成 **2000**（用户决定）：当年取 1500 是因为
      "1450 左右就能起飞，再高用不到，压窄区间精度"——但那个精度是花不出去的：
      映射 1000~1500 时一格杆 = 0.5µs，而 TIM8 计数 1MHz（`motor.c` 文件头），
      电调只能分辨 1µs。⇒ 老映射的 0.5µs 分辨率有一半是浪费。
      改成 1000~2000 后一格杆 = 1µs，正好等于电调分辨率，没有损失。
   ⚠️ 副作用两条（台架首推时要注意，不是缺陷）：
      ① 悬停点从杆量约 **900** 挪到约 **450**（1450µs 在 1000~2000 里的 45%）。
         这是常规手感（多数机器悬停就在中位附近），且往上留了大力气。
      ② 软启动时间翻倍：满杆行程 1000µs，上升速率 800µs/s ⇒ 约 1.25s（原 0.625s）。
   ⚠️ 它**只影响普通模式**。定高模式的油门权限是 vertical_control.h 里
      自己的 [1150, 1650] 常量，**刻意不跟这个值联动**，避免"调手动手感意外改定高"。 */
#define PID_TABLE_ALTITUDE_DEFAULT_MAX_THROTTLE_US  2000.0f

static void PidTable_AltitudeResetDefaults(PidTableAltitudeParams_t *altitude)
{
    altitude->vz_kp = PID_TABLE_ALTITUDE_DEFAULT_VZ_KP;
    altitude->vz_ki = PID_TABLE_ALTITUDE_DEFAULT_VZ_KI;
    altitude->vz_kd = PID_TABLE_ALTITUDE_DEFAULT_VZ_KD;
    altitude->h_kp = PID_TABLE_ALTITUDE_DEFAULT_H_KP;
    altitude->h_ki = PID_TABLE_ALTITUDE_DEFAULT_H_KI;
    altitude->h_kd = PID_TABLE_ALTITUDE_DEFAULT_H_KD;
    altitude->max_throttle_us = PID_TABLE_ALTITUDE_DEFAULT_MAX_THROTTLE_US;
}

typedef struct
{
    AttitudePidParams_t pid;
    PidTableAltitudeParams_t altitude;
    float roll_trim_deg;
    float pitch_trim_deg;
} PidTableState_t;

static PidTableState_t pid_table;
static bool pid_table_ready;

static int32_t PidTable_FloatToQ10000(float value)
{
    if (value >= 0.0f) return (int32_t)(value * (float)APEX_PID_VALUE_SCALE + 0.5f);
    return (int32_t)(value * (float)APEX_PID_VALUE_SCALE - 0.5f);
}

static float PidTable_Q10000ToFloat(int32_t value_q10000)
{
    return (float)value_q10000 / (float)APEX_PID_VALUE_SCALE;
}

static float PidTable_Absf(float value)
{
    if (value < 0.0f) return -value;
    return value;
}

static void PidTable_ExportValues(int32_t *values_q10000)
{
    const AttitudePidParams_t *pid = &pid_table.pid;

    if (values_q10000 == NULL) return;

    values_q10000[APEX_PID_ROLL_OUTER_KP] = PidTable_FloatToQ10000(pid->roll_angle.kp);
    values_q10000[APEX_PID_ROLL_OUTER_KI] = PidTable_FloatToQ10000(pid->roll_angle.ki);
    values_q10000[APEX_PID_ROLL_OUTER_KD] = PidTable_FloatToQ10000(pid->roll_angle.kd);
    values_q10000[APEX_PID_ROLL_INNER_KP] = PidTable_FloatToQ10000(pid->roll_rate.kp);
    values_q10000[APEX_PID_ROLL_INNER_KI] = PidTable_FloatToQ10000(pid->roll_rate.ki);
    values_q10000[APEX_PID_ROLL_INNER_KD] = PidTable_FloatToQ10000(pid->roll_rate.kd);
    values_q10000[APEX_PID_PITCH_OUTER_KP] = PidTable_FloatToQ10000(pid->pitch_angle.kp);
    values_q10000[APEX_PID_PITCH_OUTER_KI] = PidTable_FloatToQ10000(pid->pitch_angle.ki);
    values_q10000[APEX_PID_PITCH_OUTER_KD] = PidTable_FloatToQ10000(pid->pitch_angle.kd);
    values_q10000[APEX_PID_PITCH_INNER_KP] = PidTable_FloatToQ10000(pid->pitch_rate.kp);
    values_q10000[APEX_PID_PITCH_INNER_KI] = PidTable_FloatToQ10000(pid->pitch_rate.ki);
    values_q10000[APEX_PID_PITCH_INNER_KD] = PidTable_FloatToQ10000(pid->pitch_rate.kd);
    values_q10000[APEX_PID_YAW_RATE_KP] = PidTable_FloatToQ10000(pid->yaw_rate.kp);
    values_q10000[APEX_PID_YAW_RATE_KI] = PidTable_FloatToQ10000(pid->yaw_rate.ki);
    values_q10000[APEX_PID_YAW_RATE_KD] = PidTable_FloatToQ10000(pid->yaw_rate.kd);
    values_q10000[APEX_PID_ROLL_MAX_RATE_DPS] = PidTable_FloatToQ10000(pid->roll_angle.out_max);
    values_q10000[APEX_PID_ROLL_MAX_CORRECTION_US] = PidTable_FloatToQ10000(pid->roll_rate.out_max);
    values_q10000[APEX_PID_PITCH_MAX_RATE_DPS] = PidTable_FloatToQ10000(pid->pitch_angle.out_max);
    values_q10000[APEX_PID_PITCH_MAX_CORRECTION_US] = PidTable_FloatToQ10000(pid->pitch_rate.out_max);
    values_q10000[APEX_PID_YAW_MAX_RATE_DPS] = PidTable_FloatToQ10000(pid->yaw_angle.out_max);
    values_q10000[APEX_PID_YAW_MAX_CORRECTION_US] = PidTable_FloatToQ10000(pid->yaw_rate.out_max);
    values_q10000[APEX_PID_ROLL_TRIM_DEG] = PidTable_FloatToQ10000(pid_table.roll_trim_deg);
    values_q10000[APEX_PID_PITCH_TRIM_DEG] = PidTable_FloatToQ10000(pid_table.pitch_trim_deg);
    values_q10000[APEX_PID_YAW_ANGLE_KP] = PidTable_FloatToQ10000(pid->yaw_angle.kp);
    values_q10000[APEX_PID_YAW_ANGLE_KI] = PidTable_FloatToQ10000(pid->yaw_angle.ki);
    values_q10000[APEX_PID_YAW_ANGLE_KD] = PidTable_FloatToQ10000(pid->yaw_angle.kd);
    values_q10000[APEX_PID_VERTICAL_SPEED_KP] = PidTable_FloatToQ10000(pid_table.altitude.vz_kp);
    values_q10000[APEX_PID_VERTICAL_SPEED_KI] = PidTable_FloatToQ10000(pid_table.altitude.vz_ki);
    values_q10000[APEX_PID_VERTICAL_SPEED_KD] = PidTable_FloatToQ10000(pid_table.altitude.vz_kd);
    values_q10000[APEX_PID_HOVER_PWM_US] = PidTable_FloatToQ10000(pid_table.altitude.max_throttle_us);
    /* 高度环（外环）。必须写在最后：追加在旧编号之后，旧 30 项的位置一个都不动。 */
    values_q10000[APEX_PID_HEIGHT_KP] = PidTable_FloatToQ10000(pid_table.altitude.h_kp);
    values_q10000[APEX_PID_HEIGHT_KI] = PidTable_FloatToQ10000(pid_table.altitude.h_ki);
    values_q10000[APEX_PID_HEIGHT_KD] = PidTable_FloatToQ10000(pid_table.altitude.h_kd);
}

static ApexPidStatus_t PidTable_SetValue(uint8_t parameter_id, float value)
{
    AttitudePidParams_t *pid = &pid_table.pid;

    switch (parameter_id)
    {
    case APEX_PID_ROLL_OUTER_KP:
    case APEX_PID_ROLL_OUTER_KI:
    case APEX_PID_ROLL_OUTER_KD:
    case APEX_PID_ROLL_INNER_KP:
    case APEX_PID_ROLL_INNER_KI:
    case APEX_PID_ROLL_INNER_KD:
    case APEX_PID_PITCH_OUTER_KP:
    case APEX_PID_PITCH_OUTER_KI:
    case APEX_PID_PITCH_OUTER_KD:
    case APEX_PID_PITCH_INNER_KP:
    case APEX_PID_PITCH_INNER_KI:
    case APEX_PID_PITCH_INNER_KD:
    case APEX_PID_YAW_RATE_KP:
    case APEX_PID_YAW_RATE_KI:
    case APEX_PID_YAW_RATE_KD:
    case APEX_PID_YAW_ANGLE_KP:
    case APEX_PID_YAW_ANGLE_KI:
    case APEX_PID_YAW_ANGLE_KD:
        if ((value < 0.0f) || (value > PID_TABLE_GAIN_MAX)) return APEX_PID_STATUS_OUT_OF_RANGE;
        break;
    case APEX_PID_ROLL_MAX_RATE_DPS:
    case APEX_PID_PITCH_MAX_RATE_DPS:
    case APEX_PID_YAW_MAX_RATE_DPS:
        if ((value < PID_TABLE_RATE_MIN) || (value > PID_TABLE_RATE_MAX)) return APEX_PID_STATUS_OUT_OF_RANGE;
        break;
    case APEX_PID_ROLL_MAX_CORRECTION_US:
    case APEX_PID_PITCH_MAX_CORRECTION_US:
    case APEX_PID_YAW_MAX_CORRECTION_US:
        if ((value < PID_TABLE_CORR_MIN) || (value > PID_TABLE_CORR_MAX)) return APEX_PID_STATUS_OUT_OF_RANGE;
        break;
    case APEX_PID_ROLL_TRIM_DEG:
    case APEX_PID_PITCH_TRIM_DEG:
        if (PidTable_Absf(value) > PID_TABLE_TRIM_ABS_MAX) return APEX_PID_STATUS_OUT_OF_RANGE;
        break;
    case APEX_PID_VERTICAL_SPEED_KP:
        if ((value < 0.0f) || (value > PID_TABLE_VZ_KP_MAX)) return APEX_PID_STATUS_OUT_OF_RANGE;
        break;
    case APEX_PID_VERTICAL_SPEED_KI:
        if ((value < 0.0f) || (value > PID_TABLE_VZ_KI_MAX)) return APEX_PID_STATUS_OUT_OF_RANGE;
        break;
    case APEX_PID_VERTICAL_SPEED_KD:
        if ((value < 0.0f) || (value > PID_TABLE_VZ_KD_MAX)) return APEX_PID_STATUS_OUT_OF_RANGE;
        break;
    case APEX_PID_HOVER_PWM_US:
        if ((value < PID_TABLE_MAX_THROTTLE_MIN_US) || (value > PID_TABLE_MAX_THROTTLE_MAX_US)) return APEX_PID_STATUS_OUT_OF_RANGE;
        break;
    case APEX_PID_HEIGHT_KP:
        if ((value < 0.0f) || (value > PID_TABLE_H_KP_MAX)) return APEX_PID_STATUS_OUT_OF_RANGE;
        break;
    case APEX_PID_HEIGHT_KI:
        if ((value < 0.0f) || (value > PID_TABLE_H_KI_MAX)) return APEX_PID_STATUS_OUT_OF_RANGE;
        break;
    case APEX_PID_HEIGHT_KD:
        if ((value < 0.0f) || (value > PID_TABLE_H_KD_MAX)) return APEX_PID_STATUS_OUT_OF_RANGE;
        break;
    default:
        return APEX_PID_STATUS_INVALID_PARAMETER;
    }

    switch (parameter_id)
    {
    case APEX_PID_ROLL_OUTER_KP: pid->roll_angle.kp = value; break;
    case APEX_PID_ROLL_OUTER_KI: pid->roll_angle.ki = value; break;
    case APEX_PID_ROLL_OUTER_KD: pid->roll_angle.kd = value; break;
    case APEX_PID_ROLL_INNER_KP: pid->roll_rate.kp = value; break;
    case APEX_PID_ROLL_INNER_KI: pid->roll_rate.ki = value; break;
    case APEX_PID_ROLL_INNER_KD: pid->roll_rate.kd = value; break;
    case APEX_PID_PITCH_OUTER_KP: pid->pitch_angle.kp = value; break;
    case APEX_PID_PITCH_OUTER_KI: pid->pitch_angle.ki = value; break;
    case APEX_PID_PITCH_OUTER_KD: pid->pitch_angle.kd = value; break;
    case APEX_PID_PITCH_INNER_KP: pid->pitch_rate.kp = value; break;
    case APEX_PID_PITCH_INNER_KI: pid->pitch_rate.ki = value; break;
    case APEX_PID_PITCH_INNER_KD: pid->pitch_rate.kd = value; break;
    case APEX_PID_YAW_RATE_KP: pid->yaw_rate.kp = value; break;
    case APEX_PID_YAW_RATE_KI: pid->yaw_rate.ki = value; break;
    case APEX_PID_YAW_RATE_KD: pid->yaw_rate.kd = value; break;
    case APEX_PID_ROLL_MAX_RATE_DPS:
        pid->roll_angle.out_max = value;
        pid->roll_angle.out_min = -value;
        break;
    case APEX_PID_ROLL_MAX_CORRECTION_US:
        pid->roll_rate.out_max = value;
        pid->roll_rate.out_min = -value;
        break;
    case APEX_PID_PITCH_MAX_RATE_DPS:
        pid->pitch_angle.out_max = value;
        pid->pitch_angle.out_min = -value;
        break;
    case APEX_PID_PITCH_MAX_CORRECTION_US:
        pid->pitch_rate.out_max = value;
        pid->pitch_rate.out_min = -value;
        break;
    case APEX_PID_YAW_MAX_RATE_DPS:
        pid->yaw_angle.out_max = value;
        pid->yaw_angle.out_min = -value;
        break;
    case APEX_PID_YAW_MAX_CORRECTION_US:
        pid->yaw_rate.out_max = value;
        pid->yaw_rate.out_min = -value;
        break;
    case APEX_PID_ROLL_TRIM_DEG: pid_table.roll_trim_deg = value; break;
    case APEX_PID_PITCH_TRIM_DEG: pid_table.pitch_trim_deg = value; break;
    case APEX_PID_YAW_ANGLE_KP: pid->yaw_angle.kp = value; break;
    case APEX_PID_YAW_ANGLE_KI: pid->yaw_angle.ki = value; break;
    case APEX_PID_YAW_ANGLE_KD: pid->yaw_angle.kd = value; break;
    case APEX_PID_VERTICAL_SPEED_KP: pid_table.altitude.vz_kp = value; break;
    case APEX_PID_VERTICAL_SPEED_KI: pid_table.altitude.vz_ki = value; break;
    case APEX_PID_VERTICAL_SPEED_KD: pid_table.altitude.vz_kd = value; break;
    case APEX_PID_HOVER_PWM_US: pid_table.altitude.max_throttle_us = value; break;
    case APEX_PID_HEIGHT_KP: pid_table.altitude.h_kp = value; break;
    case APEX_PID_HEIGHT_KI: pid_table.altitude.h_ki = value; break;
    case APEX_PID_HEIGHT_KD: pid_table.altitude.h_kd = value; break;
    default:
        return APEX_PID_STATUS_INVALID_PARAMETER;
    }

    return APEX_PID_STATUS_OK;
}

static void PidTable_FillResponse(ApexPidResponse_t *response, uint16_t sequence, ApexPidOperation_t operation, ApexPidStatus_t status)
{
    if (response == NULL) return;
    memset(response, 0, sizeof(*response));
    response->sequence = sequence;
    response->status = status;
    response->operation = operation;
    response->parameter_version = PID_TABLE_PARAMETER_VERSION;
    PidTable_ExportValues(response->values_q10000);
}

void PidTable_ExportSnapshot(int32_t *values_q10000)
{
    taskENTER_CRITICAL();
    PidTable_ExportValues(values_q10000);
    taskEXIT_CRITICAL();
}

bool PidTable_ImportSnapshot(const int32_t *values_q10000)
{
    uint8_t index;
    float value;
    ApexPidStatus_t status;
    PidTableState_t backup;

    if (values_q10000 == NULL) return false;

    taskENTER_CRITICAL();
    backup = pid_table;
    for (index = 0U; index < APEX_PID_PARAMETER_COUNT; index++)
    {
        value = PidTable_Q10000ToFloat(values_q10000[index]);
        status = PidTable_SetValue(index, value);
        if (status != APEX_PID_STATUS_OK)
        {
            pid_table = backup;
            taskEXIT_CRITICAL();
            return false;
        }
    }
    taskEXIT_CRITICAL();
    AttitudeControl_ApplyParams(&pid_table.pid);
    return true;
}

uint16_t PidTable_GetMaxThrottleUs(void)
{
    float max_throttle_us;

    taskENTER_CRITICAL();
    max_throttle_us = pid_table.altitude.max_throttle_us;
    taskEXIT_CRITICAL();
    if (max_throttle_us < PID_TABLE_MAX_THROTTLE_MIN_US) max_throttle_us = PID_TABLE_MAX_THROTTLE_MIN_US;
    if (max_throttle_us > PID_TABLE_MAX_THROTTLE_MAX_US) max_throttle_us = PID_TABLE_MAX_THROTTLE_MAX_US;
    return (uint16_t)(max_throttle_us + 0.5f);
}

void PidTable_GetHeightParams(float *h_kp, float *h_ki, float *h_kd)
{
    float kp;
    float ki;
    float kd;

    taskENTER_CRITICAL();
    kp = pid_table.altitude.h_kp;
    ki = pid_table.altitude.h_ki;
    kd = pid_table.altitude.h_kd;
    taskEXIT_CRITICAL();
    if (h_kp != NULL) *h_kp = kp;
    if (h_ki != NULL) *h_ki = ki;
    if (h_kd != NULL) *h_kd = kd;
}

void PidTable_GetAltitudeParams(float *vz_kp, float *vz_ki, float *vz_kd)
{
    float kp;
    float ki;
    float kd;

    taskENTER_CRITICAL();
    kp = pid_table.altitude.vz_kp;
    ki = pid_table.altitude.vz_ki;
    kd = pid_table.altitude.vz_kd;
    taskEXIT_CRITICAL();
    if (vz_kp != NULL) *vz_kp = kp;
    if (vz_ki != NULL) *vz_ki = ki;
    if (vz_kd != NULL) *vz_kd = kd;
}

void PidTable_GetAttitudeTrimDeg(float *roll_trim_deg, float *pitch_trim_deg)
{
    float roll_trim;
    float pitch_trim;

    taskENTER_CRITICAL();
    roll_trim = pid_table.roll_trim_deg;
    pitch_trim = pid_table.pitch_trim_deg;
    taskEXIT_CRITICAL();
    if (roll_trim_deg != NULL) *roll_trim_deg = roll_trim;
    if (pitch_trim_deg != NULL) *pitch_trim_deg = pitch_trim;
}

static void PidTable_LoadCompiledDefaults(void)
{
    taskENTER_CRITICAL();
    pid_table.pid = *AttitudeControl_GetDefaultParams();
    PidTable_AltitudeResetDefaults(&pid_table.altitude);
    pid_table.roll_trim_deg = 0.0f;
    pid_table.pitch_trim_deg = 0.0f;
    taskEXIT_CRITICAL();
    AttitudeControl_ApplyParams(&pid_table.pid);
}

void PidTable_Init(void)
{
    taskENTER_CRITICAL();
    if (!pid_table_ready)
    {
        memset(&pid_table, 0, sizeof(pid_table));
        pid_table.pid = *AttitudeControl_GetDefaultParams();
        PidTable_AltitudeResetDefaults(&pid_table.altitude);
        pid_table_ready = true;
        taskEXIT_CRITICAL();
        AttitudeControl_ApplyParams(&pid_table.pid);
        return;
    }
    taskEXIT_CRITICAL();
}

void PidTable_HandleCommand(const ApexPidCommand_t *command, bool armed, ApexPidResponse_t *response)
{
    ApexPidStatus_t status;
    float value;

    if ((command == NULL) || (response == NULL)) return;

    switch (command->operation)
    {
    case APEX_PID_OPERATION_GET:
        taskENTER_CRITICAL();
        PidTable_FillResponse(response, command->sequence, command->operation, APEX_PID_STATUS_OK);
        taskEXIT_CRITICAL();
        return;

    case APEX_PID_OPERATION_SET:
        if (armed)
        {
            PidTable_FillResponse(response, command->sequence, command->operation, APEX_PID_STATUS_LOCKED_REQUIRED);
            return;
        }
        if (command->parameter_id >= APEX_PID_PARAMETER_COUNT)
        {
            PidTable_FillResponse(response, command->sequence, command->operation, APEX_PID_STATUS_INVALID_PARAMETER);
            return;
        }
        value = PidTable_Q10000ToFloat(command->value_q10000);
        taskENTER_CRITICAL();
        status = PidTable_SetValue(command->parameter_id, value);
        taskEXIT_CRITICAL();
        if (status == APEX_PID_STATUS_OK)
        {
            AttitudeControl_ApplyParams(&pid_table.pid);
        }
        taskENTER_CRITICAL();
        PidTable_FillResponse(response, command->sequence, command->operation, status);
        taskEXIT_CRITICAL();
        return;

    case APEX_PID_OPERATION_LOAD_DEFAULTS:
        if (armed)
        {
            PidTable_FillResponse(response, command->sequence, command->operation, APEX_PID_STATUS_LOCKED_REQUIRED);
            return;
        }
        PidTable_LoadCompiledDefaults();
        PidTable_FillResponse(response, command->sequence, command->operation, APEX_PID_STATUS_OK);
        return;

    case APEX_PID_OPERATION_SAVE:
        if (armed)
        {
            PidTable_FillResponse(response, command->sequence, command->operation, APEX_PID_STATUS_LOCKED_REQUIRED);
            return;
        }
        if (!Blackbox_IsReady())
        {
            PidTable_FillResponse(response, command->sequence, command->operation, APEX_PID_STATUS_STORAGE_NOT_READY);
            return;
        }
        if (!Blackbox_RequestSave())
        {
            PidTable_FillResponse(response, command->sequence, command->operation, APEX_PID_STATUS_STORAGE_ERROR);
            return;
        }
        PidTable_FillResponse(response, command->sequence, command->operation, APEX_PID_STATUS_OK);
        return;

    case APEX_PID_OPERATION_LOAD_SAVED:
        if (armed)
        {
            PidTable_FillResponse(response, command->sequence, command->operation, APEX_PID_STATUS_LOCKED_REQUIRED);
            return;
        }
        if (!Blackbox_IsReady())
        {
            PidTable_FillResponse(response, command->sequence, command->operation, APEX_PID_STATUS_STORAGE_NOT_READY);
            return;
        }
        if (!Blackbox_LoadSavedParameters())
        {
            PidTable_FillResponse(response, command->sequence, command->operation, APEX_PID_STATUS_STORAGE_ERROR);
            return;
        }
        PidTable_FillResponse(response, command->sequence, command->operation, APEX_PID_STATUS_OK);
        return;

    case APEX_PID_OPERATION_MAG_CAL_START:
        if (armed)
        {
            PidTable_FillResponse(response, command->sequence, command->operation, APEX_PID_STATUS_LOCKED_REQUIRED);
            return;
        }
        /* 开始校准不写 Flash。这里借用 STORAGE_NOT_READY 表示磁力计还没识别到芯片。 */
        status = Mag_CalibrationStart() ? APEX_PID_STATUS_OK : APEX_PID_STATUS_STORAGE_NOT_READY;
        PidTable_FillResponse(response, command->sequence, command->operation, status);
        return;

    case APEX_PID_OPERATION_MAG_CAL_STOP:
        if (armed)
        {
            PidTable_FillResponse(response, command->sequence, command->operation, APEX_PID_STATUS_LOCKED_REQUIRED);
            return;
        }
        /* 覆盖不足时回超出范围，遥控据此提示继续转圈；成功后再排队写 Flash。 */
        if (!Mag_CalibrationStop(NULL))
        {
            PidTable_FillResponse(response, command->sequence, command->operation, APEX_PID_STATUS_OUT_OF_RANGE);
            return;
        }
        if (!Blackbox_IsReady())
        {
            PidTable_FillResponse(response, command->sequence, command->operation, APEX_PID_STATUS_STORAGE_NOT_READY);
            return;
        }
        status = Blackbox_RequestMagCalibrationSave() ? APEX_PID_STATUS_OK : APEX_PID_STATUS_STORAGE_ERROR;
        PidTable_FillResponse(response, command->sequence, command->operation, status);
        return;

    case APEX_PID_OPERATION_MAG_CAL_CANCEL:
        Mag_CalibrationCancel();
        PidTable_FillResponse(response, command->sequence, command->operation, APEX_PID_STATUS_OK);
        return;

    case APEX_PID_OPERATION_RESERVED_5:
    case APEX_PID_OPERATION_RESERVED_6:
    case APEX_PID_OPERATION_RESERVED_7:
    default:
        PidTable_FillResponse(response, command->sequence, command->operation, APEX_PID_STATUS_INVALID_OPERATION);
        return;
    }
}
