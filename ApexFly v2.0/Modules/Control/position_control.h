/**
 * @file position_control.h
 * @brief 光流定点 OF6：坐标旋转和位置/速度两级环，只计算、不下发。
 *
 * 本模块只把光流的模块坐标转换成机体系，并算出将来可接入姿态环的
 * roll/pitch 目标角。当前没有任何接口会把这两个角写到姿态环、电机或油门。
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
    /* 本次使能捕获的光流累计原点，模块坐标，单位计数。 */
    float anchor_x_counts;
    float anchor_y_counts;
} PositionControlSnapshot_t;

/* 清空计数器与全部输出。光流任务启动时调用一次。 */
void PositionControl_Init(void);

/* OF7 请求接口。真实遥控请求留到 OF10 接入；当前默认请求仅用于只打印诊断。 */
void PositionControl_SetRequest(bool requested);

/* OF7 外部门禁接口：链路、摇杆中心、实际控制许可。 */
void PositionControl_SetGateContext(bool link_ok, bool sticks_centered, bool control_permission);

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
