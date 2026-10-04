/**
 * @file imu_accel_calibration.h
 * @brief 六面加速度计 bias/scale 校准的采样与计算。
 *
 * 本模块只负责校准数据的采集和计算，不负责 Flash、不负责串口命令，
 * 也不自动接入姿态解算。六个姿态完成并通过检查后，调用方再决定何时保存和应用。
 */

#ifndef IMU_ACCEL_CALIBRATION_H
#define IMU_ACCEL_CALIBRATION_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define IMU_ACCEL_CAL_POSE_COUNT 6U

typedef enum
{
    IMU_ACCEL_CAL_POSE_X_POS = 0U,
    IMU_ACCEL_CAL_POSE_X_NEG = 1U,
    IMU_ACCEL_CAL_POSE_Y_POS = 2U,
    IMU_ACCEL_CAL_POSE_Y_NEG = 3U,
    IMU_ACCEL_CAL_POSE_Z_POS = 4U,
    IMU_ACCEL_CAL_POSE_Z_NEG = 5U
} IMU_AccelCalPose_t;

/** @brief 返回姿态名称：X+、X-、Y+、Y-、Z+、Z-。 */
const char *IMU_AccelCalibration_PoseName(uint8_t pose);

typedef struct
{
    float bias_x_g;
    float bias_y_g;
    float bias_z_g;
    float scale_x;
    float scale_y;
    float scale_z;
    uint32_t version;
    uint32_t crc;
    bool valid;
} IMU_AccelCalibration_t;

typedef struct
{
    bool active;
    uint8_t current_pose;
    uint32_t target_sample_count;
    uint32_t sample_count;
    float sum_x_g;
    float sum_y_g;
    float sum_z_g;
    float sum_sq_x_g;
    float sum_sq_y_g;
    float sum_sq_z_g;
    float pose_mean_g[IMU_ACCEL_CAL_POSE_COUNT][3];
    bool pose_valid[IMU_ACCEL_CAL_POSE_COUNT];
} IMU_AccelCalibrationSession_t;

typedef enum
{
    IMU_ACCEL_CAL_CONTROLLER_IDLE = 0U,
    IMU_ACCEL_CAL_CONTROLLER_WAIT_NEXT = 1U,
    IMU_ACCEL_CAL_CONTROLLER_SAMPLING = 2U,
    IMU_ACCEL_CAL_CONTROLLER_COMPLETE = 3U,
    IMU_ACCEL_CAL_CONTROLLER_ERROR = 4U
} IMU_AccelCalibrationControllerState_t;

typedef struct
{
    uint8_t state;
    uint8_t current_pose;
    uint8_t completed_pose_count;
    uint8_t last_error;
    uint32_t target_sample_count;
    uint32_t accepted_sample_count;
    bool active_calibration_valid;
} IMU_AccelCalibrationControllerStatus_t;

/** @brief 清空会话和校准结果。 */
void IMU_AccelCalibration_Init(IMU_AccelCalibrationSession_t *session);

/**
 * @brief 开始一轮六面采样。
 * @param target_sample_count 每个姿态需要的样本数。
 */
bool IMU_AccelCalibration_Start(IMU_AccelCalibrationSession_t *session, uint32_t target_sample_count);

/** @brief 选择当前姿态并清空当前姿态的累加器。 */
bool IMU_AccelCalibration_BeginPose(IMU_AccelCalibrationSession_t *session, IMU_AccelCalPose_t pose);

/**
 * @brief 添加一个原始加速度样本，单位 g。
 * @return 样本被接受返回 true；当前姿态已收满或会话无效返回 false。
 */
bool IMU_AccelCalibration_AddSample(IMU_AccelCalibrationSession_t *session, float accel_x_g, float accel_y_g, float accel_z_g);

/** @brief 完成当前姿态，检查重力方向和模长。 */
bool IMU_AccelCalibration_FinishPose(IMU_AccelCalibrationSession_t *session);

/** @brief 六个姿态齐全后计算 bias/scale；失败时不产生有效结果。 */
bool IMU_AccelCalibration_Finalize(const IMU_AccelCalibrationSession_t *session, IMU_AccelCalibration_t *calibration);

/** @brief 用校准结果修正一组加速度，单位仍为 g。 */
bool IMU_AccelCalibration_Apply(const IMU_AccelCalibration_t *calibration, float accel_x_g, float accel_y_g, float accel_z_g, float *corrected_x_g, float *corrected_y_g, float *corrected_z_g);

/** @brief 初始化命令控制器和当前活动校准。 */
void IMU_AccelCalibration_ControllerInit(void);
/** @brief 请求开始一轮校准，采样仅由控制环消费。 */
bool IMU_AccelCalibration_RequestStart(uint32_t target_sample_count);
/** @brief 请求开始当前姿态采样。 */
bool IMU_AccelCalibration_RequestNext(void);
/** @brief 请求取消当前校准。 */
void IMU_AccelCalibration_RequestCancel(void);
/** @brief 控制环在未解锁时喂一拍原始加速度和角速度。 */
void IMU_AccelCalibration_ProcessSample(bool disarmed, float accel_x_g, float accel_y_g, float accel_z_g, float gyro_x_dps, float gyro_y_dps, float gyro_z_dps);
/** @brief 读取命令控制器状态。 */
void IMU_AccelCalibration_GetControllerStatus(IMU_AccelCalibrationControllerStatus_t *status);
/** @brief 复制已经完成、尚未保存的结果。 */
bool IMU_AccelCalibration_GetCompletedResult(IMU_AccelCalibration_t *calibration);
/** @brief 设置/清除上电加载的活动校准。 */
void IMU_AccelCalibration_SetActive(const IMU_AccelCalibration_t *calibration);
/** @brief 对当前原始加速度应用活动校准；无有效校准时保持原值。 */
void IMU_AccelCalibration_ApplyActive(float *accel_x_g, float *accel_y_g, float *accel_z_g);

#ifdef __cplusplus
}
#endif

#endif
