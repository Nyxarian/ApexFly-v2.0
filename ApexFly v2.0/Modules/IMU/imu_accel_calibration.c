/**
 * @file imu_accel_calibration.c
 * @brief 六面加速度计 bias/scale 校准实现。
 */

#include "imu_accel_calibration.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define IMU_ACCEL_CAL_GRAVITY_MIN_G 0.80f
#define IMU_ACCEL_CAL_GRAVITY_MAX_G 1.20f
#define IMU_ACCEL_CAL_DOMINANT_AXIS_MIN_G 0.75f
#define IMU_ACCEL_CAL_VARIANCE_MAX_G2 0.0025f
#define IMU_ACCEL_CAL_BIAS_MAX_G 0.30f
#define IMU_ACCEL_CAL_SCALE_MIN 0.50f
#define IMU_ACCEL_CAL_SCALE_MAX 1.50f
#define IMU_ACCEL_CAL_GYRO_STILL_MAX_DPS 2.0f

static IMU_AccelCalibrationSession_t controller_session;
static IMU_AccelCalibration_t controller_result;
static IMU_AccelCalibration_t active_calibration;
static volatile bool controller_start_pending;
static volatile bool controller_next_pending;
static volatile bool controller_cancel_pending;
static volatile uint32_t controller_requested_samples;
static IMU_AccelCalibrationControllerState_t controller_state;
static uint8_t controller_next_pose;
static uint8_t controller_last_error;

static float IMU_AccelCalibration_Absf(float value)
{
    return (value < 0.0f) ? -value : value;
}

static bool IMU_AccelCalibration_IsPoseValid(IMU_AccelCalPose_t pose)
{
    return (uint32_t)pose < IMU_ACCEL_CAL_POSE_COUNT;
}

static uint8_t IMU_AccelCalibration_Axis(IMU_AccelCalPose_t pose)
{
    return ((uint8_t)pose < 2U) ? 0U : (((uint8_t)pose < 4U) ? 1U : 2U);
}

static float IMU_AccelCalibration_ExpectedSign(IMU_AccelCalPose_t pose)
{
    return (((uint8_t)pose & 1U) == 0U) ? 1.0f : -1.0f;
}

const char *IMU_AccelCalibration_PoseName(uint8_t pose)
{
    static const char *const names[IMU_ACCEL_CAL_POSE_COUNT] = {"X+", "X-", "Y+", "Y-", "Z+", "Z-"};

    if (pose >= IMU_ACCEL_CAL_POSE_COUNT) return "DONE";
    return names[pose];
}

void IMU_AccelCalibration_Init(IMU_AccelCalibrationSession_t *session)
{
    if (session == NULL) return;
    memset(session, 0, sizeof(*session));
}

bool IMU_AccelCalibration_Start(IMU_AccelCalibrationSession_t *session, uint32_t target_sample_count)
{
    if (session == NULL) return false;
    IMU_AccelCalibration_Init(session);
    session->active = true;
    session->target_sample_count = (target_sample_count == 0U) ? 1U : target_sample_count;
    session->current_pose = 0xFFU;
    return true;
}

bool IMU_AccelCalibration_BeginPose(IMU_AccelCalibrationSession_t *session, IMU_AccelCalPose_t pose)
{
    if ((session == NULL) || !session->active || !IMU_AccelCalibration_IsPoseValid(pose)) return false;

    session->current_pose = (uint8_t)pose;
    session->sample_count = 0U;
    session->sum_x_g = 0.0f;
    session->sum_y_g = 0.0f;
    session->sum_z_g = 0.0f;
    session->sum_sq_x_g = 0.0f;
    session->sum_sq_y_g = 0.0f;
    session->sum_sq_z_g = 0.0f;
    return true;
}

bool IMU_AccelCalibration_AddSample(IMU_AccelCalibrationSession_t *session, float accel_x_g, float accel_y_g, float accel_z_g)
{
    if ((session == NULL) || !session->active || (session->current_pose >= IMU_ACCEL_CAL_POSE_COUNT)) return false;
    if (session->sample_count >= session->target_sample_count) return false;
    if (!isfinite(accel_x_g) || !isfinite(accel_y_g) || !isfinite(accel_z_g)) return false;
    {
        float norm_g = sqrtf((accel_x_g * accel_x_g) + (accel_y_g * accel_y_g) + (accel_z_g * accel_z_g));
        if ((norm_g < IMU_ACCEL_CAL_GRAVITY_MIN_G) || (norm_g > IMU_ACCEL_CAL_GRAVITY_MAX_G)) return false;
    }

    session->sum_x_g += accel_x_g;
    session->sum_y_g += accel_y_g;
    session->sum_z_g += accel_z_g;
    session->sum_sq_x_g += accel_x_g * accel_x_g;
    session->sum_sq_y_g += accel_y_g * accel_y_g;
    session->sum_sq_z_g += accel_z_g * accel_z_g;
    session->sample_count++;
    return true;
}

bool IMU_AccelCalibration_FinishPose(IMU_AccelCalibrationSession_t *session)
{
    IMU_AccelCalPose_t pose;
    float mean_x;
    float mean_y;
    float mean_z;
    float norm_g;
    uint8_t axis;
    float expected;
    float dominant;
    float variance_x;
    float variance_y;
    float variance_z;

    if ((session == NULL) || !session->active || (session->current_pose >= IMU_ACCEL_CAL_POSE_COUNT) ||
        (session->sample_count < session->target_sample_count))
    {
        return false;
    }

    pose = (IMU_AccelCalPose_t)session->current_pose;
    mean_x = session->sum_x_g / (float)session->sample_count;
    mean_y = session->sum_y_g / (float)session->sample_count;
    mean_z = session->sum_z_g / (float)session->sample_count;
    variance_x = (session->sum_sq_x_g / (float)session->sample_count) - (mean_x * mean_x);
    variance_y = (session->sum_sq_y_g / (float)session->sample_count) - (mean_y * mean_y);
    variance_z = (session->sum_sq_z_g / (float)session->sample_count) - (mean_z * mean_z);
    if ((variance_x < 0.0f) && (variance_x > -0.00001f)) variance_x = 0.0f;
    if ((variance_y < 0.0f) && (variance_y > -0.00001f)) variance_y = 0.0f;
    if ((variance_z < 0.0f) && (variance_z > -0.00001f)) variance_z = 0.0f;
    norm_g = sqrtf((mean_x * mean_x) + (mean_y * mean_y) + (mean_z * mean_z));
    axis = IMU_AccelCalibration_Axis(pose);
    expected = IMU_AccelCalibration_ExpectedSign(pose);
    dominant = (axis == 0U) ? mean_x : ((axis == 1U) ? mean_y : mean_z);

    if ((variance_x < 0.0f) || (variance_y < 0.0f) || (variance_z < 0.0f) ||
        (variance_x > IMU_ACCEL_CAL_VARIANCE_MAX_G2) || (variance_y > IMU_ACCEL_CAL_VARIANCE_MAX_G2) || (variance_z > IMU_ACCEL_CAL_VARIANCE_MAX_G2) ||
        (norm_g < IMU_ACCEL_CAL_GRAVITY_MIN_G) || (norm_g > IMU_ACCEL_CAL_GRAVITY_MAX_G) ||
        ((dominant * expected) < IMU_ACCEL_CAL_DOMINANT_AXIS_MIN_G))
    {
        return false;
    }

    session->pose_mean_g[(uint8_t)pose][0] = mean_x;
    session->pose_mean_g[(uint8_t)pose][1] = mean_y;
    session->pose_mean_g[(uint8_t)pose][2] = mean_z;
    session->pose_valid[(uint8_t)pose] = true;
    session->current_pose = 0xFFU;
    session->sample_count = 0U;
    {
        uint8_t index;
        bool all_poses_valid = true;
        for (index = 0U; index < IMU_ACCEL_CAL_POSE_COUNT; index++)
        {
            if (!session->pose_valid[index])
            {
                all_poses_valid = false;
                break;
            }
        }
        if (all_poses_valid) session->active = false;
    }
    return true;
}

bool IMU_AccelCalibration_Finalize(const IMU_AccelCalibrationSession_t *session, IMU_AccelCalibration_t *calibration)
{
    uint8_t axis;
    float positive;
    float negative;
    float difference;
    float bias;
    float scale;

    if ((session == NULL) || (calibration == NULL) || session->active) return false;
    for (axis = 0U; axis < IMU_ACCEL_CAL_POSE_COUNT; axis++)
    {
        if (!session->pose_valid[axis]) return false;
    }

    memset(calibration, 0, sizeof(*calibration));
    for (axis = 0U; axis < 3U; axis++)
    {
        positive = session->pose_mean_g[axis * 2U][axis];
        negative = session->pose_mean_g[(axis * 2U) + 1U][axis];
        difference = positive - negative;
        bias = 0.5f * (positive + negative);
        if (IMU_AccelCalibration_Absf(difference) < 0.2f) return false;
        scale = 2.0f / difference;
        if ((IMU_AccelCalibration_Absf(bias) > IMU_ACCEL_CAL_BIAS_MAX_G) ||
            (scale < IMU_ACCEL_CAL_SCALE_MIN) || (scale > IMU_ACCEL_CAL_SCALE_MAX))
        {
            return false;
        }
        if (axis == 0U) { calibration->bias_x_g = bias; calibration->scale_x = scale; }
        else if (axis == 1U) { calibration->bias_y_g = bias; calibration->scale_y = scale; }
        else { calibration->bias_z_g = bias; calibration->scale_z = scale; }
    }
    calibration->version = 1U;
    calibration->valid = true;
    return true;
}

bool IMU_AccelCalibration_Apply(const IMU_AccelCalibration_t *calibration, float accel_x_g, float accel_y_g, float accel_z_g, float *corrected_x_g, float *corrected_y_g, float *corrected_z_g)
{
    if ((calibration == NULL) || !calibration->valid || (corrected_x_g == NULL) || (corrected_y_g == NULL) || (corrected_z_g == NULL)) return false;
    *corrected_x_g = (accel_x_g - calibration->bias_x_g) * calibration->scale_x;
    *corrected_y_g = (accel_y_g - calibration->bias_y_g) * calibration->scale_y;
    *corrected_z_g = (accel_z_g - calibration->bias_z_g) * calibration->scale_z;
    return true;
}

void IMU_AccelCalibration_ControllerInit(void)
{
    IMU_AccelCalibration_Init(&controller_session);
    memset(&controller_result, 0, sizeof(controller_result));
    memset(&active_calibration, 0, sizeof(active_calibration));
    controller_start_pending = false;
    controller_next_pending = false;
    controller_cancel_pending = false;
    controller_requested_samples = 200U;
    controller_state = IMU_ACCEL_CAL_CONTROLLER_IDLE;
    controller_next_pose = 0U;
    controller_last_error = 0U;
}

bool IMU_AccelCalibration_RequestStart(uint32_t target_sample_count)
{
    if ((controller_state == IMU_ACCEL_CAL_CONTROLLER_SAMPLING) ||
        (controller_state == IMU_ACCEL_CAL_CONTROLLER_WAIT_NEXT)) return false;
    controller_requested_samples = (target_sample_count == 0U) ? 200U : target_sample_count;
    controller_start_pending = true;
    return true;
}

bool IMU_AccelCalibration_RequestNext(void)
{
    if (controller_state != IMU_ACCEL_CAL_CONTROLLER_WAIT_NEXT) return false;
    controller_next_pending = true;
    return true;
}

void IMU_AccelCalibration_RequestCancel(void)
{
    controller_cancel_pending = true;
}

static void IMU_AccelCalibration_ControllerSetError(uint8_t error)
{
    controller_last_error = error;
    controller_state = IMU_ACCEL_CAL_CONTROLLER_ERROR;
    controller_session.active = false;
}

void IMU_AccelCalibration_ProcessSample(bool disarmed, float accel_x_g, float accel_y_g, float accel_z_g, float gyro_x_dps, float gyro_y_dps, float gyro_z_dps)
{
    if (controller_cancel_pending)
    {
        controller_cancel_pending = false;
        IMU_AccelCalibration_Init(&controller_session);
        controller_state = IMU_ACCEL_CAL_CONTROLLER_IDLE;
        controller_next_pose = 0U;
        controller_last_error = 0U;
        return;
    }
    if (!disarmed) return;

    if (controller_start_pending)
    {
        controller_start_pending = false;
        if (!IMU_AccelCalibration_Start(&controller_session, controller_requested_samples))
        {
            IMU_AccelCalibration_ControllerSetError(1U);
            return;
        }
        controller_next_pose = 0U;
        controller_last_error = 0U;
        controller_state = IMU_ACCEL_CAL_CONTROLLER_WAIT_NEXT;
    }

    if ((controller_state == IMU_ACCEL_CAL_CONTROLLER_WAIT_NEXT) && controller_next_pending)
    {
        controller_next_pending = false;
        if (controller_next_pose >= IMU_ACCEL_CAL_POSE_COUNT ||
            !IMU_AccelCalibration_BeginPose(&controller_session, (IMU_AccelCalPose_t)controller_next_pose))
        {
            IMU_AccelCalibration_ControllerSetError(2U);
            return;
        }
        controller_state = IMU_ACCEL_CAL_CONTROLLER_SAMPLING;
    }

    if (controller_state != IMU_ACCEL_CAL_CONTROLLER_SAMPLING) return;
    if ((IMU_AccelCalibration_Absf(gyro_x_dps) > IMU_ACCEL_CAL_GYRO_STILL_MAX_DPS) ||
        (IMU_AccelCalibration_Absf(gyro_y_dps) > IMU_ACCEL_CAL_GYRO_STILL_MAX_DPS) ||
        (IMU_AccelCalibration_Absf(gyro_z_dps) > IMU_ACCEL_CAL_GYRO_STILL_MAX_DPS)) return;
    (void)IMU_AccelCalibration_AddSample(&controller_session, accel_x_g, accel_y_g, accel_z_g);
    if (controller_session.sample_count < controller_session.target_sample_count) return;

    if (!IMU_AccelCalibration_FinishPose(&controller_session))
    {
        IMU_AccelCalibration_ControllerSetError(3U);
        return;
    }
    controller_next_pose++;
    if (controller_next_pose >= IMU_ACCEL_CAL_POSE_COUNT)
    {
        if (!IMU_AccelCalibration_Finalize(&controller_session, &controller_result))
        {
            IMU_AccelCalibration_ControllerSetError(4U);
            return;
        }
        controller_state = IMU_ACCEL_CAL_CONTROLLER_COMPLETE;
    }
    else
    {
        controller_state = IMU_ACCEL_CAL_CONTROLLER_WAIT_NEXT;
    }
}

void IMU_AccelCalibration_GetControllerStatus(IMU_AccelCalibrationControllerStatus_t *status)
{
    uint8_t index;
    uint8_t completed = 0U;

    if (status == NULL) return;
    for (index = 0U; index < IMU_ACCEL_CAL_POSE_COUNT; index++)
    {
        if (controller_session.pose_valid[index]) completed++;
    }
    status->state = (uint8_t)controller_state;
    status->current_pose = controller_next_pose;
    status->completed_pose_count = completed;
    status->last_error = controller_last_error;
    status->target_sample_count = controller_session.target_sample_count;
    status->accepted_sample_count = controller_session.sample_count;
    status->active_calibration_valid = active_calibration.valid;
}

bool IMU_AccelCalibration_GetCompletedResult(IMU_AccelCalibration_t *calibration)
{
    if ((calibration == NULL) || (controller_state != IMU_ACCEL_CAL_CONTROLLER_COMPLETE) || !controller_result.valid) return false;
    *calibration = controller_result;
    return true;
}

void IMU_AccelCalibration_SetActive(const IMU_AccelCalibration_t *calibration)
{
    if ((calibration == NULL) || !calibration->valid)
    {
        memset(&active_calibration, 0, sizeof(active_calibration));
        return;
    }
    active_calibration = *calibration;
}

void IMU_AccelCalibration_ApplyActive(float *accel_x_g, float *accel_y_g, float *accel_z_g)
{
    float corrected_x;
    float corrected_y;
    float corrected_z;

    if ((accel_x_g == NULL) || (accel_y_g == NULL) || (accel_z_g == NULL) || !active_calibration.valid) return;
    if (!IMU_AccelCalibration_Apply(&active_calibration, *accel_x_g, *accel_y_g, *accel_z_g, &corrected_x, &corrected_y, &corrected_z)) return;
    *accel_x_g = corrected_x;
    *accel_y_g = corrected_y;
    *accel_z_g = corrected_z;
}
