/**
 * @file position_control.h
 * @brief 光流机体系转换、定点状态机和位置/速度两级环。
 *
 * 本模块只把光流的模块坐标转换成机体系，并算出将来可接入姿态环的
 * roll/pitch 目标角，由 motor_test.c 在普通模式接入姿态环；本模块不直接写电机或油门。
 */

#ifndef POSITION_CONTROL_H
#define POSITION_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    POSITION_CONTROL_STATE_DISABLED = 0U,
    POSITION_CONTROL_STATE_WAITING = 1U,
    POSITION_CONTROL_STATE_ACTIVE = 2U,
    POSITION_CONTROL_STATE_LATCHED = 3U,
    POSITION_CONTROL_STATE_PILOT_MOVE = 4U
} PositionControlState_t;

typedef enum
{
    POSITION_CONTROL_FAULT_NONE = 0U,
    POSITION_CONTROL_FAULT_FLOW = 1U,
    POSITION_CONTROL_FAULT_LINK = 2U,
    POSITION_CONTROL_FAULT_ATTITUDE = 3U,
    POSITION_CONTROL_FAULT_HEIGHT = 4U,
    POSITION_CONTROL_FAULT_PERMISSION = 5U
} PositionControlFaultReason_t;

#define POSITION_CONTROL_GATE_REQUEST 0x01U
#define POSITION_CONTROL_GATE_LINK    0x02U
#define POSITION_CONTROL_GATE_FLOW    0x04U
#define POSITION_CONTROL_GATE_STICKS  0x08U
#define POSITION_CONTROL_GATE_PERMISSION 0x10U

typedef struct
{
    /* true：已完成零点标定且光流当前有效。 */
    bool valid;
    /* 当前请求、门禁位图和状态，供 OF7 台架打印。 */
    bool request;
    uint8_t gate_mask;
    uint8_t state;
    /* 每个合格标定后帧加一；位置环应约为它的一半。 */
    uint32_t velocity_update_count;
    uint32_t position_update_count;
    /* 机体系位置：前、右为正，单位计数。 */
    float position_forward_counts;
    float position_right_counts;
    /* 机体系速度：前、右为正，单位计数/秒。 */
    float velocity_forward_cps;
    float velocity_right_cps;
    /* 位置环给速度环的目标速度，单位计数/秒。 */
    float velocity_forward_sp_cps;
    float velocity_right_sp_cps;
    /* 速度环原始输出与 2.5Hz 低通后的最终输出，单位度。 */
    float raw_roll_deg;
    float raw_pitch_deg;
    float roll_deg;
    float pitch_deg;
    /* 当前高度补偿比例，1.0 表示参考高度。 */
    float height_scale;
    /* true 表示因机体角速度过大而暂时冻结光流控制。 */
    bool motion_suppressed;
    /* 本次使能捕获的光流累计原点，模块坐标，单位计数。 */
    float anchor_x_counts;
    float anchor_y_counts;
    /* 最近一次阻止/退出定点的原因，取 PositionControlFaultReason_t。 */
    uint8_t fault_reason;
} PositionControlSnapshot_t;

/* 清空计数器与全部输出。光流任务启动时调用一次。 */
void PositionControl_Init(void);

/* OF10 独立定点请求接口。上电默认关闭。 */
void PositionControl_SetRequest(bool requested);

/* OF8 外部门禁接口：链路与摇杆状态，由 Flow 任务更新。 */
void PositionControl_SetPilotContext(bool link_ok, bool sticks_centered);

/* 安全状态由控制环更新。pilot_disarmed 仅表示链路正常且有效遥控帧明确请求上锁；
   失联、内环禁用和 IMU 故障均传 false，避免把异常许可失效当作正常上锁。
   例：PositionControl_SetSafetyContext(false, true, true, true) -> 定点等待。 */
void PositionControl_SetSafetyContext(bool control_permission, bool attitude_safe, bool height_safe, bool pilot_disarmed);

/** @brief 发布 ToF 高度快照，供光流位置/速度量按参考高度归一化。 */
void PositionControl_SetHeightContext(bool valid, float height_cm);

/** @brief 旋转期间冻结位置环；旋转结束后稳定一段时间再重新锚定。 */
void PositionControl_SetMotionSuppressed(bool suppressed);

/*
 * 输入一帧已减零点的光流状态并更新两级环。
 * sum_x/sum_y 为模块坐标的累积位移，vx/vy 为模块坐标的瞬时速度。
 * dt_s 为本帧实际间隔；非正值会被忽略，输出保持上一拍。
 */
void PositionControl_Update(float sum_x, float sum_y, float vx_cps, float vy_cps, float dt_s);

/* 光流失效或仍在标定时置 false：立刻把输出归零并复位滤波/分频状态。 */
void PositionControl_SetFlowAvailable(bool available);

/* 复制当前快照，给低频打印任务读取。 */
void PositionControl_GetSnapshot(PositionControlSnapshot_t *snapshot);

#ifdef __cplusplus
}
#endif

#endif /* POSITION_CONTROL_H */
