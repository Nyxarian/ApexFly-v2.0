/**
 * @file imu_attitude.c
 * @brief MPU6050 六轴 Mahony4 风格四元数姿态解算实现。
 *
 * 内部只维护芯片坐标系四元数，使用归一化加速度提供重力方向反馈，
 * 不使用磁力计，因此偏航只能相对积分。对外输出继续沿用 ApexFly 已验证的
 * 芯片系到机体系映射，避免同时改变姿态 PID 和混控符号。
 */

#include "imu_attitude.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define IMU_ATTITUDE_RAD_TO_DEG 57.2957795f
#define IMU_ATTITUDE_DEG_TO_RAD 0.0174532925f
#define IMU_ATTITUDE_GRAVITY_MPS2 9.80665f
/* 加计模长门限：只在接近 1g 时认为测到的是重力，否则冻结重力修正，
   避免机动或振动时把"比力"当成重力方向拉偏姿态。
   注：加计尚未做零偏/尺度标定，静置实测约 1.046g，因此上限放宽到 1.10；
   完成 #13 加计标定后可收紧到 0.95~1.05。 */
#define IMU_ATTITUDE_ACCEL_NORM_MIN_G 0.95f
#define IMU_ATTITUDE_ACCEL_NORM_MAX_G 1.10f
/* 加计一阶低通系数：截止约 30Hz（100Hz 采样，alpha = 1 - e^(-2*pi*fc*dt) 约 0.85）。 */
#define IMU_ATTITUDE_ACCEL_LPF_ALPHA 0.85f
#define IMU_ATTITUDE_DELTA_TIME_MIN_S 0.0005f
#define IMU_ATTITUDE_DELTA_TIME_MAX_S 0.030f
#define IMU_ATTITUDE_MAHONY_KP 0.80f
/* 0.0003/样本按当前 100Hz 换算为每秒积分增益。 */
#define IMU_ATTITUDE_MAHONY_KI 0.030f
#define IMU_ATTITUDE_INTEGRAL_LIMIT_RAD_S 0.20f
#define IMU_ATTITUDE_QUATERNION_NORM_MIN 0.000001f
#define IMU_ATTITUDE_FINITE_ABS_LIMIT 1000000.0f
/* 阶段 2A：机架确认水平后测得的固定姿态偏置。
   约定为 corrected = raw + trim；不随每次上电重新采样。后续再迁移到持久化校准参数。 */
#define IMU_ATTITUDE_MOUNT_ROLL_TRIM_DEG  (-0.14f)
#define IMU_ATTITUDE_MOUNT_PITCH_TRIM_DEG (-0.87f)

static bool IMU_Attitude_IsFinite(float value)
{
    return (value == value) && (fabsf(value) < IMU_ATTITUDE_FINITE_ABS_LIMIT);
}

static float IMU_Attitude_Clamp(float value, float minimum, float maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static float IMU_Attitude_NormalizeAngle(float angle_deg)
{
    while (angle_deg > 180.0f) angle_deg -= 360.0f;
    while (angle_deg < -180.0f) angle_deg += 360.0f;
    return angle_deg;
}

static bool IMU_Attitude_NormalizeQuaternion(float *q0, float *q1, float *q2, float *q3)
{
    float norm_square;
    float inverse_norm;

    if ((q0 == NULL) || (q1 == NULL) || (q2 == NULL) || (q3 == NULL)) return false;
    if (!IMU_Attitude_IsFinite(*q0) || !IMU_Attitude_IsFinite(*q1) || !IMU_Attitude_IsFinite(*q2) || !IMU_Attitude_IsFinite(*q3)) return false;

    norm_square = (*q0 * *q0) + (*q1 * *q1) + (*q2 * *q2) + (*q3 * *q3);
    if (!IMU_Attitude_IsFinite(norm_square) || (norm_square < IMU_ATTITUDE_QUATERNION_NORM_MIN)) return false;

    inverse_norm = 1.0f / sqrtf(norm_square);
    if (!IMU_Attitude_IsFinite(inverse_norm)) return false;

    *q0 *= inverse_norm;
    *q1 *= inverse_norm;
    *q2 *= inverse_norm;
    *q3 *= inverse_norm;
    return true;
}

static bool IMU_Attitude_GetAccelUnit(float accel_x_g, float accel_y_g, float accel_z_g, float *ax, float *ay, float *az, float *norm_g)
{
    float norm_square;
    float inverse_norm;

    if ((ax == NULL) || (ay == NULL) || (az == NULL) || (norm_g == NULL)) return false;
    if (!IMU_Attitude_IsFinite(accel_x_g) || !IMU_Attitude_IsFinite(accel_y_g) || !IMU_Attitude_IsFinite(accel_z_g)) return false;

    norm_square = (accel_x_g * accel_x_g) + (accel_y_g * accel_y_g) + (accel_z_g * accel_z_g);
    if (!IMU_Attitude_IsFinite(norm_square) || (norm_square < IMU_ATTITUDE_QUATERNION_NORM_MIN)) return false;

    *norm_g = sqrtf(norm_square);
    if ((norm_square < IMU_ATTITUDE_ACCEL_NORM_MIN_G * IMU_ATTITUDE_ACCEL_NORM_MIN_G) || (norm_square > IMU_ATTITUDE_ACCEL_NORM_MAX_G * IMU_ATTITUDE_ACCEL_NORM_MAX_G)) return false;
    inverse_norm = 1.0f / *norm_g;
    if (!IMU_Attitude_IsFinite(inverse_norm)) return false;

    *ax = accel_x_g * inverse_norm;
    *ay = accel_y_g * inverse_norm;
    *az = accel_z_g * inverse_norm;
    return IMU_Attitude_IsFinite(*ax) && IMU_Attitude_IsFinite(*ay) && IMU_Attitude_IsFinite(*az);
}

static void IMU_Attitude_GetGravityVector(const IMU_Attitude_t *attitude, float *gx, float *gy, float *gz)
{
    *gx = 2.0f * (attitude->q1 * attitude->q3 - attitude->q0 * attitude->q2);
    *gy = 2.0f * (attitude->q0 * attitude->q1 + attitude->q2 * attitude->q3);
    *gz = 1.0f - 2.0f * (attitude->q1 * attitude->q1 + attitude->q2 * attitude->q2);
}

static void IMU_Attitude_SetInitialQuaternion(IMU_Attitude_t *attitude, float roll_rad, float pitch_rad)
{
    float half_roll;
    float half_pitch;
    float sin_half_roll;
    float cos_half_roll;
    float sin_half_pitch;
    float cos_half_pitch;

    half_roll = roll_rad * 0.5f;
    half_pitch = pitch_rad * 0.5f;
    sin_half_roll = sinf(half_roll);
    cos_half_roll = cosf(half_roll);
    sin_half_pitch = sinf(half_pitch);
    cos_half_pitch = cosf(half_pitch);

    /* 芯片系初始偏航置零，Roll/Pitch 由静止重力方向建立。 */
    attitude->q0 = cos_half_roll * cos_half_pitch;
    attitude->q1 = sin_half_roll * cos_half_pitch;
    attitude->q2 = cos_half_roll * sin_half_pitch;
    attitude->q3 = -sin_half_roll * sin_half_pitch;
    (void)IMU_Attitude_NormalizeQuaternion(&attitude->q0, &attitude->q1, &attitude->q2, &attitude->q3);
}

static void IMU_Attitude_GetChipEuler(const IMU_Attitude_t *attitude, float *roll_deg, float *pitch_deg, float *yaw_deg)
{
    float gravity_x;
    float gravity_y;
    float gravity_z;
    float pitch_argument;

    IMU_Attitude_GetGravityVector(attitude, &gravity_x, &gravity_y, &gravity_z);
    pitch_argument = IMU_Attitude_Clamp(-gravity_x, -1.0f, 1.0f);
    *roll_deg = IMU_Attitude_NormalizeAngle(atan2f(gravity_y, gravity_z) * IMU_ATTITUDE_RAD_TO_DEG);
    *pitch_deg = IMU_Attitude_NormalizeAngle(asinf(pitch_argument) * IMU_ATTITUDE_RAD_TO_DEG);
    *yaw_deg = IMU_Attitude_NormalizeAngle(atan2f(2.0f * (attitude->q0 * attitude->q3 + attitude->q1 * attitude->q2), 1.0f - 2.0f * (attitude->q2 * attitude->q2 + attitude->q3 * attitude->q3)) * IMU_ATTITUDE_RAD_TO_DEG);
}

void IMU_Attitude_Init(IMU_Attitude_t *attitude)
{
    if (attitude == NULL) return;
    memset(attitude, 0, sizeof(*attitude));
    attitude->q0 = 1.0f;
}

bool IMU_Attitude_Update(IMU_Attitude_t *attitude, const MPU6050_Data_t *data, float delta_time_s)
{
    float ax;
    float ay;
    float az;
    float accel_norm_g = 0.0f;
    float gravity_x;
    float gravity_y;
    float gravity_z;
    float error_x;
    float error_y;
    float error_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float q0_new;
    float q1_new;
    float q2_new;
    float q3_new;
    float roll_rad;
    float pitch_rad;

    if ((attitude == NULL) || (data == NULL)) return false;
    /* dt 异常时不再放弃本拍，而是用限幅值继续积分：
       极端延迟下"用偏差一点的 dt 继续控制"比"完全不控制"安全。 */
    if (!IMU_Attitude_IsFinite(delta_time_s)) return false;
    if (delta_time_s < IMU_ATTITUDE_DELTA_TIME_MIN_S) delta_time_s = IMU_ATTITUDE_DELTA_TIME_MIN_S;
    if (delta_time_s > IMU_ATTITUDE_DELTA_TIME_MAX_S) delta_time_s = IMU_ATTITUDE_DELTA_TIME_MAX_S;
    if (!IMU_Attitude_IsFinite(data->gyro_x_dps) || !IMU_Attitude_IsFinite(data->gyro_y_dps) || !IMU_Attitude_IsFinite(data->gyro_z_dps)) return false;

    /* 加计一阶低通：抑制电机振动残留进入重力修正。首拍直接用原始值初始化，避免从 0 缓慢爬升。 */
    if (!attitude->accel_lpf_ready)
    {
        attitude->accel_x_lpf = data->accel_x_g;
        attitude->accel_y_lpf = data->accel_y_g;
        attitude->accel_z_lpf = data->accel_z_g;
        attitude->accel_lpf_ready = true;
    }
    else
    {
        attitude->accel_x_lpf = (IMU_ATTITUDE_ACCEL_LPF_ALPHA * data->accel_x_g) + ((1.0f - IMU_ATTITUDE_ACCEL_LPF_ALPHA) * attitude->accel_x_lpf);
        attitude->accel_y_lpf = (IMU_ATTITUDE_ACCEL_LPF_ALPHA * data->accel_y_g) + ((1.0f - IMU_ATTITUDE_ACCEL_LPF_ALPHA) * attitude->accel_y_lpf);
        attitude->accel_z_lpf = (IMU_ATTITUDE_ACCEL_LPF_ALPHA * data->accel_z_g) + ((1.0f - IMU_ATTITUDE_ACCEL_LPF_ALPHA) * attitude->accel_z_lpf);
    }

    attitude->accel_correction_valid = IMU_Attitude_GetAccelUnit(attitude->accel_x_lpf, attitude->accel_y_lpf, attitude->accel_z_lpf, &ax, &ay, &az, &accel_norm_g);
    attitude->accel_norm_g = accel_norm_g;
    if (!attitude->initialized)
    {
        if (!attitude->accel_correction_valid) return false;
        roll_rad = atan2f(attitude->accel_y_lpf, attitude->accel_z_lpf);
        pitch_rad = atan2f(-attitude->accel_x_lpf, sqrtf(attitude->accel_y_lpf * attitude->accel_y_lpf + attitude->accel_z_lpf * attitude->accel_z_lpf));
        IMU_Attitude_SetInitialQuaternion(attitude, roll_rad, pitch_rad);
        attitude->fusion_error_x = 0.0f;
        attitude->fusion_error_y = 0.0f;
        attitude->fusion_error_z = 0.0f;
        attitude->initialized = true;
        return true;
    }

    IMU_Attitude_GetGravityVector(attitude, &gravity_x, &gravity_y, &gravity_z);
    if (attitude->accel_correction_valid)
    {
        /* 加速度观测与当前重力方向的叉乘，是 Mahony 的姿态误差。 */
        error_x = ay * gravity_z - az * gravity_y;
        error_y = az * gravity_x - ax * gravity_z;
        error_z = ax * gravity_y - ay * gravity_x;
        attitude->fusion_error_x = error_x;
        attitude->fusion_error_y = error_y;
        attitude->fusion_error_z = error_z;

        attitude->integral_x += IMU_ATTITUDE_MAHONY_KI * error_x * delta_time_s;
        attitude->integral_y += IMU_ATTITUDE_MAHONY_KI * error_y * delta_time_s;
        attitude->integral_z += IMU_ATTITUDE_MAHONY_KI * error_z * delta_time_s;
        attitude->integral_x = IMU_Attitude_Clamp(attitude->integral_x, -IMU_ATTITUDE_INTEGRAL_LIMIT_RAD_S, IMU_ATTITUDE_INTEGRAL_LIMIT_RAD_S);
        attitude->integral_y = IMU_Attitude_Clamp(attitude->integral_y, -IMU_ATTITUDE_INTEGRAL_LIMIT_RAD_S, IMU_ATTITUDE_INTEGRAL_LIMIT_RAD_S);
        attitude->integral_z = IMU_Attitude_Clamp(attitude->integral_z, -IMU_ATTITUDE_INTEGRAL_LIMIT_RAD_S, IMU_ATTITUDE_INTEGRAL_LIMIT_RAD_S);

        gyro_x = data->gyro_x_dps * IMU_ATTITUDE_DEG_TO_RAD + IMU_ATTITUDE_MAHONY_KP * error_x + attitude->integral_x;
        gyro_y = data->gyro_y_dps * IMU_ATTITUDE_DEG_TO_RAD + IMU_ATTITUDE_MAHONY_KP * error_y + attitude->integral_y;
        gyro_z = data->gyro_z_dps * IMU_ATTITUDE_DEG_TO_RAD + IMU_ATTITUDE_MAHONY_KP * error_z + attitude->integral_z;
    }
    else
    {
        /* 高动态或异常数据时只做陀螺预测，禁止继续累积重力误差。 */
        attitude->fusion_error_x = 0.0f;
        attitude->fusion_error_y = 0.0f;
        attitude->fusion_error_z = 0.0f;
        gyro_x = data->gyro_x_dps * IMU_ATTITUDE_DEG_TO_RAD;
        gyro_y = data->gyro_y_dps * IMU_ATTITUDE_DEG_TO_RAD;
        gyro_z = data->gyro_z_dps * IMU_ATTITUDE_DEG_TO_RAD;
    }

    /* 中值积分：用上一拍与本拍修正后角速度的平均值，把积分误差从 O(dt^2) 降到 O(dt^3)。
       首拍没有历史，退化为欧拉法。注意先取中值再保存本拍值。 */
    if (attitude->gyro_history_valid)
    {
        float gyro_mid_x = 0.5f * (gyro_x + attitude->last_gyro_x_rad);
        float gyro_mid_y = 0.5f * (gyro_y + attitude->last_gyro_y_rad);
        float gyro_mid_z = 0.5f * (gyro_z + attitude->last_gyro_z_rad);
        attitude->last_gyro_x_rad = gyro_x;
        attitude->last_gyro_y_rad = gyro_y;
        attitude->last_gyro_z_rad = gyro_z;
        gyro_x = gyro_mid_x;
        gyro_y = gyro_mid_y;
        gyro_z = gyro_mid_z;
    }
    else
    {
        attitude->last_gyro_x_rad = gyro_x;
        attitude->last_gyro_y_rad = gyro_y;
        attitude->last_gyro_z_rad = gyro_z;
        attitude->gyro_history_valid = true;
    }

    q0_new = attitude->q0 + 0.5f * (-attitude->q1 * gyro_x - attitude->q2 * gyro_y - attitude->q3 * gyro_z) * delta_time_s;
    q1_new = attitude->q1 + 0.5f * ( attitude->q0 * gyro_x - attitude->q3 * gyro_y + attitude->q2 * gyro_z) * delta_time_s;
    q2_new = attitude->q2 + 0.5f * ( attitude->q3 * gyro_x + attitude->q0 * gyro_y - attitude->q1 * gyro_z) * delta_time_s;
    q3_new = attitude->q3 + 0.5f * (-attitude->q2 * gyro_x + attitude->q1 * gyro_y + attitude->q0 * gyro_z) * delta_time_s;

    if (!IMU_Attitude_NormalizeQuaternion(&q0_new, &q1_new, &q2_new, &q3_new)) return false;
    attitude->q0 = q0_new;
    attitude->q1 = q1_new;
    attitude->q2 = q2_new;
    attitude->q3 = q3_new;
    return true;
}

void IMU_Attitude_MapToBody(const IMU_Attitude_t *attitude, const MPU6050_Data_t *data, IMU_BodyAttitude_t *body)
{
    float chip_roll_deg;
    float chip_pitch_deg;
    float chip_yaw_deg;

    if ((attitude == NULL) || (data == NULL) || (body == NULL) || !attitude->initialized) return;
    IMU_Attitude_GetChipEuler(attitude, &chip_roll_deg, &chip_pitch_deg, &chip_yaw_deg);

    /* 保持当前已实测的芯片系到机体系映射，不改变姿态 PID 极性。 */
    body->roll_deg = chip_pitch_deg + IMU_ATTITUDE_MOUNT_ROLL_TRIM_DEG;
    body->pitch_deg = chip_roll_deg + IMU_ATTITUDE_MOUNT_PITCH_TRIM_DEG;
    body->yaw_deg = -chip_yaw_deg;
    body->roll_rate_dps = data->gyro_y_dps;
    body->pitch_rate_dps = data->gyro_x_dps;
    body->yaw_rate_dps = -data->gyro_z_dps;
}

float IMU_Attitude_VerticalAccelUp(const IMU_Attitude_t *attitude, const MPU6050_Data_t *data)
{
    float gravity_x;
    float gravity_y;
    float gravity_z;
    float specific_up_g;

    if ((attitude == NULL) || (data == NULL) || !attitude->initialized) return 0.0f;
    IMU_Attitude_GetGravityVector(attitude, &gravity_x, &gravity_y, &gravity_z);
    /* 用软件低通后的加计值，避免振动直接进入垂向加速度。 */
    specific_up_g = attitude->accel_x_lpf * gravity_x + attitude->accel_y_lpf * gravity_y + attitude->accel_z_lpf * gravity_z;
    return (specific_up_g - 1.0f) * IMU_ATTITUDE_GRAVITY_MPS2;
}

float IMU_Attitude_GetAccelNormG(const IMU_Attitude_t *attitude)
{
    if (attitude == NULL) return 0.0f;
    return attitude->accel_norm_g;
}

bool IMU_Attitude_IsAccelCorrectionValid(const IMU_Attitude_t *attitude)
{
    if (attitude == NULL) return false;
    return attitude->accel_correction_valid;
}

void IMU_Attitude_GetFusionError(const IMU_Attitude_t *attitude, float *x, float *y, float *z)
{
    if (attitude == NULL) return;
    if (x != NULL) *x = attitude->fusion_error_x;
    if (y != NULL) *y = attitude->fusion_error_y;
    if (z != NULL) *z = attitude->fusion_error_z;
}

void IMU_Attitude_GetQuaternion(const IMU_Attitude_t *attitude, float q[4])
{
    if ((attitude == NULL) || (q == NULL)) return;
    q[0] = attitude->q0;
    q[1] = attitude->q1;
    q[2] = attitude->q2;
    q[3] = attitude->q3;
}
