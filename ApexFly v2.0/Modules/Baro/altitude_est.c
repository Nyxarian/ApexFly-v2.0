/**
 * @file altitude_est.c
 * @brief 三状态卡尔曼：气压钉高度，加速度补短时速度，并估计剩余 az 零偏。
 *
 * 状态为 [z, vz, ba]。初始 ba 来自解锁前静置平均；单次气压创新超过 2 m 时拒绝，
 * 连续三次异常则用气压重置高度和垂速，避免估计长期发散到几十米。
 */

#include "altitude_est.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define ALTITUDE_EST_Q_ACC 0.08f
#define ALTITUDE_EST_Q_BIAS 0.0002f
#define ALTITUDE_EST_R_BARO 0.09f
#define ALTITUDE_EST_DT_MIN_S 0.005f
#define ALTITUDE_EST_DT_MAX_S 0.03f
#define ALTITUDE_EST_VZ_LIMIT_MPS 2.0f
#define ALTITUDE_EST_BIAS_LIMIT_MPS2 1.5f
#define ALTITUDE_EST_INNOVATION_LIMIT_M 2.0f
#define ALTITUDE_EST_BAD_BARO_RESET_COUNT 3U
#define ALTITUDE_EST_BARO_TIMEOUT_S 1.0f

static float est_z;
static float est_vz;
static float est_ba;
static float est_p[3][3];
static float est_baro_age_s;
static float est_last_innovation_m;
static bool est_initialized;
static bool est_healthy;
static uint8_t est_bad_baro_count;

static float AltitudeEst_Clampf(float value, float minimum, float maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static void AltitudeEst_ResetCovariance(void)
{
    memset(est_p, 0, sizeof(est_p));
    est_p[0][0] = 0.25f;
    est_p[1][1] = 0.25f;
    est_p[2][2] = 0.04f;
}

static void AltitudeEst_ResetFromBaro(float baro_altitude_m)
{
    est_z = baro_altitude_m;
    est_vz = 0.0f;
    est_baro_age_s = 0.0f;
    est_last_innovation_m = 0.0f;
    est_bad_baro_count = 0U;
    AltitudeEst_ResetCovariance();
    est_initialized = true;
    est_healthy = true;
}

void AltitudeEst_Init(float initial_accel_bias_mps2)
{
    est_z = 0.0f;
    est_vz = 0.0f;
    est_ba = AltitudeEst_Clampf(initial_accel_bias_mps2, -ALTITUDE_EST_BIAS_LIMIT_MPS2, ALTITUDE_EST_BIAS_LIMIT_MPS2);
    est_baro_age_s = ALTITUDE_EST_BARO_TIMEOUT_S;
    est_last_innovation_m = 0.0f;
    est_bad_baro_count = 0U;
    AltitudeEst_ResetCovariance();
    est_initialized = false;
    est_healthy = false;
}

void AltitudeEst_Update(float accel_up_mps2, float dt_s, float baro_altitude_m, bool baro_fresh)
{
    float dt;
    float dt2;
    float az;
    float f_p[3][3];
    float s;
    float k0;
    float k1;
    float k2;
    uint8_t r;
    uint8_t c;

    if (!isfinite(accel_up_mps2) || !isfinite(dt_s))
    {
        est_healthy = false;
        return;
    }
    dt = AltitudeEst_Clampf(dt_s, ALTITUDE_EST_DT_MIN_S, ALTITUDE_EST_DT_MAX_S);
    dt2 = dt * dt;

    if (!est_initialized)
    {
        if (!baro_fresh || !isfinite(baro_altitude_m)) return;
        AltitudeEst_ResetFromBaro(baro_altitude_m);
        return;
    }

    est_baro_age_s += dt;
    az = accel_up_mps2 - est_ba;
    est_z += est_vz * dt + 0.5f * az * dt2;
    est_vz += az * dt;

    /* F = [[1, dt, -0.5 dt^2], [0, 1, -dt], [0, 0, 1]]。 */
    memcpy(f_p, est_p, sizeof(f_p));
    for (r = 0U; r < 3U; r++)
    {
        est_p[0][r] = f_p[0][r] + dt * f_p[1][r] - 0.5f * dt2 * f_p[2][r];
        est_p[1][r] = f_p[1][r] - dt * f_p[2][r];
        est_p[2][r] = f_p[2][r];
    }
    memcpy(f_p, est_p, sizeof(f_p));
    for (c = 0U; c < 3U; c++)
    {
        est_p[c][0] = f_p[c][0] + dt * f_p[c][1] - 0.5f * dt2 * f_p[c][2];
        est_p[c][1] = f_p[c][1] - dt * f_p[c][2];
        est_p[c][2] = f_p[c][2];
    }
    est_p[0][0] += ALTITUDE_EST_Q_ACC * 0.25f * dt2 * dt2;
    est_p[0][1] += ALTITUDE_EST_Q_ACC * 0.5f * dt2 * dt;
    est_p[1][0] += ALTITUDE_EST_Q_ACC * 0.5f * dt2 * dt;
    est_p[1][1] += ALTITUDE_EST_Q_ACC * dt2;
    est_p[2][2] += ALTITUDE_EST_Q_BIAS * dt;

    if (baro_fresh && isfinite(baro_altitude_m))
    {
        est_last_innovation_m = baro_altitude_m - est_z;
        if (fabsf(est_last_innovation_m) > ALTITUDE_EST_INNOVATION_LIMIT_M)
        {
            if (est_bad_baro_count < 0xFFU) est_bad_baro_count++;
            if (est_bad_baro_count >= ALTITUDE_EST_BAD_BARO_RESET_COUNT) AltitudeEst_ResetFromBaro(baro_altitude_m);
        }
        else
        {
            est_bad_baro_count = 0U;
            est_baro_age_s = 0.0f;
            s = est_p[0][0] + ALTITUDE_EST_R_BARO;
            if (s > 1.0e-6f)
            {
                k0 = est_p[0][0] / s;
                k1 = est_p[1][0] / s;
                k2 = est_p[2][0] / s;
                est_z += k0 * est_last_innovation_m;
                est_vz += k1 * est_last_innovation_m;
                est_ba += k2 * est_last_innovation_m;

                memcpy(f_p, est_p, sizeof(f_p));
                for (c = 0U; c < 3U; c++)
                {
                    est_p[0][c] = (1.0f - k0) * f_p[0][c];
                    est_p[1][c] = f_p[1][c] - k1 * f_p[0][c];
                    est_p[2][c] = f_p[2][c] - k2 * f_p[0][c];
                }
                est_p[0][1] = 0.5f * (est_p[0][1] + est_p[1][0]);
                est_p[1][0] = est_p[0][1];
                est_p[0][2] = 0.5f * (est_p[0][2] + est_p[2][0]);
                est_p[2][0] = est_p[0][2];
                est_p[1][2] = 0.5f * (est_p[1][2] + est_p[2][1]);
                est_p[2][1] = est_p[1][2];
            }
        }
    }

    est_vz = AltitudeEst_Clampf(est_vz, -ALTITUDE_EST_VZ_LIMIT_MPS, ALTITUDE_EST_VZ_LIMIT_MPS);
    est_ba = AltitudeEst_Clampf(est_ba, -ALTITUDE_EST_BIAS_LIMIT_MPS2, ALTITUDE_EST_BIAS_LIMIT_MPS2);
    est_healthy = isfinite(est_z) && isfinite(est_vz) && isfinite(est_ba) && (est_baro_age_s < ALTITUDE_EST_BARO_TIMEOUT_S);
}

void AltitudeEst_Get(AltitudeEstimate_t *estimate)
{
    if (estimate == NULL) return;
    estimate->z_m = est_z;
    estimate->vz_mps = est_vz;
    estimate->accel_bias_mps2 = est_ba;
    estimate->baro_innovation_m = est_last_innovation_m;
    estimate->initialized = est_initialized;
    estimate->healthy = est_healthy;
}
