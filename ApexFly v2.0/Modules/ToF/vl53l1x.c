/**
 * @file vl53l1x.c
 * @brief VL53L1X 飞行时间激光测距底层驱动实现（单次测量版）。
 *
 * ── 实现依据（重要，请勿随意改动本文件里的常量）────────────────────────
 * 初始化序列、寄存器地址、短距档参数与时间预算换算式，依据 ST 的 VL53L1X API
 * （STSW-IMG007）公开的寄存器映射与初始化流程，并参照 Pololu 的 vl53l1x-arduino
 * 库（MIT 许可，作者 Pololu）的等价实现整理而成。
 * 对应文档：VL53L1X 数据手册（DS12385）、VL53L1X API 用户手册（UM2356）。
 *
 * ── 工作流程（单次测量）──────────────────────────────────────────
 *   上电：复位 → 等启动 → 读振荡器频率 → 静态配置 → 动态配置 →
 *         设短距档 → 写时间预算(20ms) → 触发第一次测量
 *   之后：上层每次 StartSingleShot() 触发一次，IsMeasurementDone() 查询完成，
 *         ReadMeasurement() 取结果。首次读结果时做一次手动校准。
 *
 * ── 为什么不用连续模式 ─────────────────────────────────────────
 *   2026-09-11 实测：本芯片连续模式（MODE_START=0x40）写入成功、寄存器读回 0x40，
 *   但不再产生新结果；单次模式（0x10）每次都能正常完成（mode 自动回 0x00，
 *   测距状态读回 0x09）。故改用单次模式。
 *
 * ── I2C 约定 ─────────────────────────────────────────────────
 *   每一次 I2C 访问都先拿 I2C2 锁，与 hmc5883l.c 一致（两设备共用一条总线）。
 *   寄存器索引为 2 字节大端；多字节数据也是大端（MSB 在前）。
 */

#include "vl53l1x.h"

#include "i2c2_lock.h"
#include <stddef.h>

/* ------------------------------------------------------------------ *
 *  寄存器地址
 * ------------------------------------------------------------------ */
#define REG_SOFT_RESET                          0x0000U
#define REG_OSC_MEASURED__FAST_OSC__FREQUENCY   0x0006U
#define REG_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND 0x0008U
#define REG_VHV_CONFIG__INIT                    0x000BU
#define REG_ALGO__PART_TO_PART_RANGE_OFFSET_MM  0x001EU
#define REG_MM_CONFIG__OUTER_OFFSET_MM          0x0022U
#define REG_DSS_CONFIG__TARGET_TOTAL_RATE_MCPS  0x0024U
#define REG_PAD_I2C_HV__EXTSUP_CONFIG           0x002EU
#define REG_GPIO__TIO_HV_STATUS                 0x0031U
#define REG_SIGMA_ESTIMATOR__EFFECTIVE_PULSE_WIDTH_NS   0x0036U
#define REG_SIGMA_ESTIMATOR__EFFECTIVE_AMBIENT_WIDTH_NS 0x0037U
#define REG_ALGO__CROSSTALK_COMPENSATION_VALID_HEIGHT_MM 0x0039U
#define REG_ALGO__RANGE_IGNORE_VALID_HEIGHT_MM  0x003EU
#define REG_ALGO__RANGE_MIN_CLIP                0x003FU
#define REG_ALGO__CONSISTENCY_CHECK__TOLERANCE  0x0040U
#define REG_SYSTEM__INTERRUPT_CONFIG_GPIO       0x0046U
#define REG_CAL_CONFIG__VCSEL_START             0x0047U
#define REG_PHASECAL_CONFIG__TIMEOUT_MACROP     0x004BU
#define REG_PHASECAL_CONFIG__OVERRIDE           0x004DU
#define REG_DSS_CONFIG__ROI_MODE_CONTROL        0x004FU
#define REG_SYSTEM__THRESH_RATE_HIGH            0x0050U
#define REG_SYSTEM__THRESH_RATE_LOW             0x0052U
#define REG_DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT 0x0054U
#define REG_DSS_CONFIG__APERTURE_ATTENUATION    0x0057U
#define REG_MM_CONFIG__TIMEOUT_MACROP_A         0x005AU
#define REG_MM_CONFIG__TIMEOUT_MACROP_B         0x005CU
#define REG_RANGE_CONFIG__TIMEOUT_MACROP_A      0x005EU
#define REG_RANGE_CONFIG__VCSEL_PERIOD_A        0x0060U
#define REG_RANGE_CONFIG__TIMEOUT_MACROP_B      0x0061U
#define REG_RANGE_CONFIG__VCSEL_PERIOD_B        0x0063U
#define REG_RANGE_CONFIG__SIGMA_THRESH          0x0064U
#define REG_RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS 0x0066U
#define REG_RANGE_CONFIG__VALID_PHASE_HIGH      0x0069U
#define REG_SYSTEM__INTERMEASUREMENT_PERIOD     0x006CU
#define REG_SYSTEM__GROUPED_PARAMETER_HOLD_0    0x0071U
#define REG_SYSTEM__SEED_CONFIG                 0x0077U
#define REG_SD_CONFIG__WOI_SD0                  0x0078U
#define REG_SD_CONFIG__WOI_SD1                  0x0079U
#define REG_SD_CONFIG__INITIAL_PHASE_SD0        0x007AU
#define REG_SD_CONFIG__INITIAL_PHASE_SD1        0x007BU
#define REG_SYSTEM__GROUPED_PARAMETER_HOLD_1    0x007CU
#define REG_SD_CONFIG__QUANTIFIER               0x007EU
#define REG_SYSTEM__SEQUENCE_CONFIG             0x0081U
#define REG_SYSTEM__GROUPED_PARAMETER_HOLD      0x0082U
#define REG_SYSTEM__INTERRUPT_CLEAR             0x0086U
#define REG_SYSTEM__MODE_START                  0x0087U
#define REG_RESULT__INTERRUPT_STATUS            0x0088U
#define REG_RESULT__RANGE_STATUS                0x0089U
#define REG_RESULT__STREAM_COUNT                0x008BU
#define REG_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0 0x008CU
#define REG_RESULT__AMBIENT_COUNT_RATE_MCPS_SD0 0x0090U
#define REG_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0 0x0096U
#define REG_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0 0x0098U
#define REG_PHASECAL_RESULT__VCSEL_START        0x00D8U
#define REG_FIRMWARE__SYSTEM_STATUS             0x00E5U
#define REG_IDENTIFICATION__MODEL_ID            0x010FU

/* 期望的型号 ID（数据手册 Table 8：0x010F=0xEA，0x0110=0xCC）。 */
#define VL53L1X_EXPECTED_MODEL_ID 0xEACCU

/* DSS 目标回波率，8.8 定点，默认 0.25 MCPS。 */
#define VL53L1X_TARGET_RATE_MCPS 0x0A00U

/* 时间预算的保护间隔，单位微秒。 */
#define VL53L1X_TIMING_GUARD_US 4528U
/* 固定 20ms 时间预算（短距档）。 */
#define VL53L1X_TIMING_BUDGET_US 20000U

/* 单次测量命令值。 */
#define VL53L1X_MODE_SINGLE_SHOT 0x10U
/* 中止测量命令值。 */
#define VL53L1X_MODE_ABORT       0x80U
/* 测距完成时的 RANGE_STATUS 原始码（参考实现认定为有效）。 */
#define VL53L1X_RANGE_STATUS_COMPLETE 0x09U

#define VL53L1X_I2C_TIMEOUT_MS  50U
#define VL53L1X_LOCK_TIMEOUT_MS 200U

/* 一次测量的原始字段（读 17 字节得到）。 */
typedef struct
{
    uint8_t range_status;
    uint8_t stream_count;
    uint16_t dss_spads_sd0;
    uint16_t ambient_mcps_sd0;
    uint16_t range_mm_sd0;
    uint16_t peak_signal_mcps_sd0;
} VL53L1X_RawResults_t;

static VL53L1X_RawResults_t vl53l1x_raw;
static uint32_t vl53l1x_fast_osc_frequency;

/* ------------------------------------------------------------------ *
 *  I2C 底层：每次访问都加 I2C2 锁
 * ------------------------------------------------------------------ */

static bool VL53L1X_WriteBytes(VL53L1X_Device_t *device, uint16_t index, const uint8_t *data, uint8_t length)
{
    uint8_t buffer[6];
    uint8_t i;
    bool ok;

    if ((length == 0U) || ((uint16_t)length + 2U > sizeof(buffer))) return false;

    buffer[0] = (uint8_t)(index >> 8);
    buffer[1] = (uint8_t)(index & 0x00FFU);
    for (i = 0U; i < length; i++)
    {
        buffer[2U + i] = data[i];
    }

    if (!I2C2_Lock(VL53L1X_LOCK_TIMEOUT_MS)) return false;
    ok = (HAL_I2C_Master_Transmit(device->i2c, device->address, buffer, (uint16_t)(2U + length), VL53L1X_I2C_TIMEOUT_MS) == HAL_OK);
    I2C2_Unlock();

    if (!ok) device->error_count++;
    return ok;
}

static bool VL53L1X_ReadBytes(VL53L1X_Device_t *device, uint16_t index, uint8_t *data, uint8_t length)
{
    uint8_t reg[2];
    bool ok;

    reg[0] = (uint8_t)(index >> 8);
    reg[1] = (uint8_t)(index & 0x00FFU);

    if (!I2C2_Lock(VL53L1X_LOCK_TIMEOUT_MS)) return false;
    ok = (HAL_I2C_Master_Transmit(device->i2c, device->address, reg, 2U, VL53L1X_I2C_TIMEOUT_MS) == HAL_OK);
    if (ok)
    {
        ok = (HAL_I2C_Master_Receive(device->i2c, device->address, data, length, VL53L1X_I2C_TIMEOUT_MS) == HAL_OK);
    }
    I2C2_Unlock();

    if (!ok) device->error_count++;
    return ok;
}

static bool VL53L1X_WriteByte(VL53L1X_Device_t *device, uint16_t index, uint8_t value)
{
    return VL53L1X_WriteBytes(device, index, &value, 1U);
}

static bool VL53L1X_ReadByte(VL53L1X_Device_t *device, uint16_t index, uint8_t *value)
{
    return VL53L1X_ReadBytes(device, index, value, 1U);
}

static bool VL53L1X_WriteWord(VL53L1X_Device_t *device, uint16_t index, uint16_t value)
{
    uint8_t buffer[2];

    buffer[0] = (uint8_t)(value >> 8);
    buffer[1] = (uint8_t)(value & 0x00FFU);
    return VL53L1X_WriteBytes(device, index, buffer, 2U);
}

static bool VL53L1X_ReadWord(VL53L1X_Device_t *device, uint16_t index, uint16_t *value)
{
    uint8_t buffer[2];

    if (!VL53L1X_ReadBytes(device, index, buffer, 2U)) return false;
    *value = (uint16_t)(((uint16_t)buffer[0] << 8) | buffer[1]);
    return true;
}

/* ------------------------------------------------------------------ *
 *  时间预算换算："(低字节 × 2^高字节) + 1"
 * ------------------------------------------------------------------ */

static uint16_t VL53L1X_EncodeTimeout(uint32_t timeout_mclks)
{
    uint32_t ls_byte = 0U;
    uint16_t ms_byte = 0U;

    if (timeout_mclks == 0U) return 0U;

    ls_byte = timeout_mclks - 1U;
    while ((ls_byte & 0xFFFFFF00UL) != 0U)
    {
        ls_byte >>= 1;
        ms_byte++;
    }
    return (uint16_t)((ms_byte << 8) | (ls_byte & 0xFFU));
}

static uint32_t VL53L1X_TimeoutMicrosecondsToMclks(uint32_t timeout_us, uint32_t macro_period_us)
{
    return (((uint32_t)timeout_us << 12) + (macro_period_us >> 1)) / macro_period_us;
}

/* 由 VCSEL 周期算"宏周期"，单位微秒，12.12 定点。依赖已读到的振荡器频率。 */
static uint32_t VL53L1X_CalcMacroPeriod(uint8_t vcsel_period)
{
    uint32_t pll_period_us;
    uint8_t vcsel_period_pclks;
    uint32_t macro_period_us;

    if (vl53l1x_fast_osc_frequency == 0U) return 0U;

    pll_period_us = ((uint32_t)1U << 30) / vl53l1x_fast_osc_frequency;
    vcsel_period_pclks = (uint8_t)(((uint32_t)vcsel_period + 1U) << 1);

    macro_period_us = (uint32_t)2304U * pll_period_us;   /* MACRO_PERIOD_VCSEL_PERIODS */
    macro_period_us >>= 6;
    macro_period_us *= vcsel_period_pclks;
    macro_period_us >>= 6;

    return macro_period_us;
}

static bool VL53L1X_SetTimingBudget(VL53L1X_Device_t *device, uint32_t budget_us)
{
    uint32_t range_config_timeout_us;
    uint32_t macro_period_us;
    uint32_t phasecal_timeout_mclks;
    uint8_t vcsel_period;

    if (budget_us <= VL53L1X_TIMING_GUARD_US) return false;
    range_config_timeout_us = budget_us - VL53L1X_TIMING_GUARD_US;
    if (range_config_timeout_us > 1100000U) return false;
    range_config_timeout_us /= 2U;

    if (!VL53L1X_ReadByte(device, REG_RANGE_CONFIG__VCSEL_PERIOD_A, &vcsel_period)) return false;
    macro_period_us = VL53L1X_CalcMacroPeriod(vcsel_period);
    if (macro_period_us == 0U) return false;

    phasecal_timeout_mclks = VL53L1X_TimeoutMicrosecondsToMclks(1000U, macro_period_us);
    if (phasecal_timeout_mclks > 0xFFU) phasecal_timeout_mclks = 0xFFU;
    if (!VL53L1X_WriteByte(device, REG_PHASECAL_CONFIG__TIMEOUT_MACROP, (uint8_t)phasecal_timeout_mclks)) return false;

    if (!VL53L1X_WriteWord(device, REG_MM_CONFIG__TIMEOUT_MACROP_A,
                           VL53L1X_EncodeTimeout(VL53L1X_TimeoutMicrosecondsToMclks(1U, macro_period_us)))) return false;
    if (!VL53L1X_WriteWord(device, REG_RANGE_CONFIG__TIMEOUT_MACROP_A,
                           VL53L1X_EncodeTimeout(VL53L1X_TimeoutMicrosecondsToMclks(range_config_timeout_us, macro_period_us)))) return false;

    if (!VL53L1X_ReadByte(device, REG_RANGE_CONFIG__VCSEL_PERIOD_B, &vcsel_period)) return false;
    macro_period_us = VL53L1X_CalcMacroPeriod(vcsel_period);
    if (macro_period_us == 0U) return false;

    if (!VL53L1X_WriteWord(device, REG_MM_CONFIG__TIMEOUT_MACROP_B,
                           VL53L1X_EncodeTimeout(VL53L1X_TimeoutMicrosecondsToMclks(1U, macro_period_us)))) return false;
    if (!VL53L1X_WriteWord(device, REG_RANGE_CONFIG__TIMEOUT_MACROP_B,
                           VL53L1X_EncodeTimeout(VL53L1X_TimeoutMicrosecondsToMclks(range_config_timeout_us, macro_period_us)))) return false;

    return true;
}

/* 配置为短距档。 */
static bool VL53L1X_SetShortRangeMode(VL53L1X_Device_t *device)
{
    if (!VL53L1X_WriteByte(device, REG_RANGE_CONFIG__VCSEL_PERIOD_A, 0x07U)) return false;
    if (!VL53L1X_WriteByte(device, REG_RANGE_CONFIG__VCSEL_PERIOD_B, 0x05U)) return false;
    if (!VL53L1X_WriteByte(device, REG_RANGE_CONFIG__VALID_PHASE_HIGH, 0x38U)) return false;
    if (!VL53L1X_WriteByte(device, REG_SD_CONFIG__WOI_SD0, 0x07U)) return false;
    if (!VL53L1X_WriteByte(device, REG_SD_CONFIG__WOI_SD1, 0x05U)) return false;
    if (!VL53L1X_WriteByte(device, REG_SD_CONFIG__INITIAL_PHASE_SD0, 6U)) return false;
    if (!VL53L1X_WriteByte(device, REG_SD_CONFIG__INITIAL_PHASE_SD1, 6U)) return false;
    return true;
}

/* 等启动完成：轮询固件状态寄存器的 bit0。 */
static bool VL53L1X_PollBoot(VL53L1X_Device_t *device)
{
    uint32_t attempt;
    uint8_t status;

    for (attempt = 0U; attempt < 100U; attempt++)
    {
        if (VL53L1X_ReadByte(device, REG_FIRMWARE__SYSTEM_STATUS, &status))
        {
            if ((status & 0x01U) != 0U) return true;
        }
        HAL_Delay(2U);
    }
    return false;
}

/* 首次测距后做手动校准：关掉固件校准步骤、改写 VHV 与相位校准。
   依据 API 的 low_power_auto_setup_manual_calibration()。 */
static void VL53L1X_SetupManualCalibration(VL53L1X_Device_t *device)
{
    uint8_t vcsel_start;

    (void)VL53L1X_ReadByte(device, REG_VHV_CONFIG__INIT, &device->saved_vhv_init);
    (void)VL53L1X_ReadByte(device, REG_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, &device->saved_vhv_timeout);

    (void)VL53L1X_WriteByte(device, REG_VHV_CONFIG__INIT, (uint8_t)(device->saved_vhv_init & 0x7FU));
    (void)VL53L1X_WriteByte(device, REG_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND,
                            (uint8_t)((device->saved_vhv_timeout & 0x03U) + (3U << 2)));

    (void)VL53L1X_WriteByte(device, REG_PHASECAL_CONFIG__OVERRIDE, 0x01U);
    if (VL53L1X_ReadByte(device, REG_PHASECAL_RESULT__VCSEL_START, &vcsel_start))
    {
        (void)VL53L1X_WriteByte(device, REG_CAL_CONFIG__VCSEL_START, vcsel_start);
    }

    device->calibrated = true;
}

/* 按实测信号强度动态更新 DSS（有效 SPAD 数）。依据 API 的 low_power_auto_update_DSS()。 */
static void VL53L1X_UpdateDss(VL53L1X_Device_t *device)
{
    uint16_t spad_count = vl53l1x_raw.dss_spads_sd0;

    if (spad_count != 0U)
    {
        uint32_t total_rate_per_spad = (uint32_t)vl53l1x_raw.peak_signal_mcps_sd0 + vl53l1x_raw.ambient_mcps_sd0;

        if (total_rate_per_spad > 0xFFFFU) total_rate_per_spad = 0xFFFFU;
        total_rate_per_spad <<= 16;
        total_rate_per_spad /= spad_count;

        if (total_rate_per_spad != 0U)
        {
            uint32_t required_spads = ((uint32_t)VL53L1X_TARGET_RATE_MCPS << 16) / total_rate_per_spad;

            if (required_spads > 0xFFFFU) required_spads = 0xFFFFU;
            (void)VL53L1X_WriteWord(device, REG_DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, (uint16_t)required_spads);
            return;
        }
    }

    /* 除零保护：把目标设到中间值。 */
    (void)VL53L1X_WriteWord(device, REG_DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, 0x8000U);
}

/* 读一次 17 字节结果块（从 RANGE_STATUS 起）。 */
static bool VL53L1X_ReadRawResults(VL53L1X_Device_t *device)
{
    uint8_t buffer[17];

    if (!VL53L1X_ReadBytes(device, REG_RESULT__RANGE_STATUS, buffer, (uint8_t)sizeof(buffer))) return false;

    vl53l1x_raw.range_status = buffer[0];
    /* buffer[1] = report status，不用 */
    vl53l1x_raw.stream_count = buffer[2];
    vl53l1x_raw.dss_spads_sd0 = (uint16_t)(((uint16_t)buffer[3] << 8) | buffer[4]);
    /* buffer[5..6] = peak signal，不用 */
    vl53l1x_raw.ambient_mcps_sd0 = (uint16_t)(((uint16_t)buffer[7] << 8) | buffer[8]);
    /* buffer[9..12] = sigma / phase，不用 */
    vl53l1x_raw.range_mm_sd0 = (uint16_t)(((uint16_t)buffer[13] << 8) | buffer[14]);
    vl53l1x_raw.peak_signal_mcps_sd0 = (uint16_t)(((uint16_t)buffer[15] << 8) | buffer[16]);

    return true;
}

/* ------------------------------------------------------------------ *
 *  对外接口
 * ------------------------------------------------------------------ */

const char *VL53L1X_InitStatusString(VL53L1X_InitStatus_t status)
{
    switch (status)
    {
    case VL53L1X_INIT_NOT_STARTED:        return "not started";
    case VL53L1X_INIT_OK:                 return "ok";
    case VL53L1X_INIT_NO_ACK:             return "no ack (check VCC/GND/SCL/SDA/XSHUT)";
    case VL53L1X_INIT_BAD_MODEL_ID:       return "bad model id (expect 0xEACC)";
    case VL53L1X_INIT_CONFIG_WRITE_FAIL:  return "config write failed";
    case VL53L1X_INIT_CONFIG_VERIFY_FAIL: return "boot poll failed";
    case VL53L1X_INIT_START_FAIL:         return "start single shot failed";
    default:                              return "unknown";
    }
}

VL53L1X_InitStatus_t VL53L1X_GetLastInitStatus(const VL53L1X_Device_t *device)
{
    if (device == NULL) return VL53L1X_INIT_NOT_STARTED;
    return device->last_init_status;
}

uint16_t VL53L1X_ReadModelId(VL53L1X_Device_t *device)
{
    uint16_t model_id = 0U;

    if ((device == NULL) || (device->i2c == NULL)) return 0U;
    if (!VL53L1X_ReadWord(device, REG_IDENTIFICATION__MODEL_ID, &model_id)) return 0U;
    return model_id;
}

bool VL53L1X_Init(VL53L1X_Device_t *device, I2C_HandleTypeDef *i2c)
{
    uint8_t byte_value;
    uint16_t outer_offset;

    if ((device == NULL) || (i2c == NULL)) return false;

    device->i2c = i2c;
    device->address = (uint16_t)(VL53L1X_I2C_ADDRESS << 1);
    device->initialized = false;
    device->error_count = 0U;
    device->last_init_status = VL53L1X_INIT_NOT_STARTED;
    device->calibrated = false;
    device->saved_vhv_init = 0U;
    device->saved_vhv_timeout = 0U;

    /* ── 1. 确认芯片在总线上、型号正确（数据手册 Table 8）。 ── */
    if (VL53L1X_ReadModelId(device) != VL53L1X_EXPECTED_MODEL_ID)
    {
        bool ack = false;
        if (I2C2_Lock(VL53L1X_LOCK_TIMEOUT_MS))
        {
            ack = (HAL_I2C_IsDeviceReady(i2c, device->address, 3U, VL53L1X_I2C_TIMEOUT_MS) == HAL_OK);
            I2C2_Unlock();
        }
        device->last_init_status = ack ? VL53L1X_INIT_BAD_MODEL_ID : VL53L1X_INIT_NO_ACK;
        return false;
    }

    /* ── 2. 软复位。 ── */
    if (!VL53L1X_WriteByte(device, REG_SOFT_RESET, 0x00U)) goto fail_write;
    HAL_Delay(1U);
    if (!VL53L1X_WriteByte(device, REG_SOFT_RESET, 0x01U)) goto fail_write;
    HAL_Delay(1U);

    /* ── 3. 等启动完成。 ── */
    if (!VL53L1X_PollBoot(device))
    {
        device->last_init_status = VL53L1X_INIT_CONFIG_VERIFY_FAIL;
        return false;
    }

    /* ── 4. 切到 2V8 模式（本模块 I/O 供电 3.3V）。 ── */
    if (!VL53L1X_ReadByte(device, REG_PAD_I2C_HV__EXTSUP_CONFIG, &byte_value)) goto fail_write;
    if (!VL53L1X_WriteByte(device, REG_PAD_I2C_HV__EXTSUP_CONFIG, (uint8_t)(byte_value | 0x01U))) goto fail_write;

    /* ── 5. 读振荡器频率，后面算时间预算要用。 ── */
    {
        uint16_t osc_frequency = 0U;
        if (!VL53L1X_ReadWord(device, REG_OSC_MEASURED__FAST_OSC__FREQUENCY, &osc_frequency)) goto fail_write;
        vl53l1x_fast_osc_frequency = (uint32_t)osc_frequency;
    }

    /* ── 6. 静态配置（对应 API 的 standard ranging 预设）。 ── */
    if (!VL53L1X_WriteWord(device, REG_DSS_CONFIG__TARGET_TOTAL_RATE_MCPS, VL53L1X_TARGET_RATE_MCPS)) goto fail_write;
    if (!VL53L1X_WriteByte(device, REG_GPIO__TIO_HV_STATUS, 0x02U)) goto fail_write;
    if (!VL53L1X_WriteByte(device, REG_SIGMA_ESTIMATOR__EFFECTIVE_PULSE_WIDTH_NS, 8U)) goto fail_write;
    if (!VL53L1X_WriteByte(device, REG_SIGMA_ESTIMATOR__EFFECTIVE_AMBIENT_WIDTH_NS, 16U)) goto fail_write;
    if (!VL53L1X_WriteByte(device, REG_ALGO__CROSSTALK_COMPENSATION_VALID_HEIGHT_MM, 0x01U)) goto fail_write;
    if (!VL53L1X_WriteByte(device, REG_ALGO__RANGE_IGNORE_VALID_HEIGHT_MM, 0xFFU)) goto fail_write;
    if (!VL53L1X_WriteByte(device, REG_ALGO__RANGE_MIN_CLIP, 0U)) goto fail_write;
    if (!VL53L1X_WriteByte(device, REG_ALGO__CONSISTENCY_CHECK__TOLERANCE, 2U)) goto fail_write;

    /* ── 7. 通用配置。 ── */
    if (!VL53L1X_WriteWord(device, REG_SYSTEM__THRESH_RATE_HIGH, 0x0000U)) goto fail_write;
    if (!VL53L1X_WriteWord(device, REG_SYSTEM__THRESH_RATE_LOW, 0x0000U)) goto fail_write;
    if (!VL53L1X_WriteByte(device, REG_DSS_CONFIG__APERTURE_ATTENUATION, 0x38U)) goto fail_write;

    /* ── 8. 时序配置（随后由"档位 + 时间预算"重算）。 ── */
    if (!VL53L1X_WriteWord(device, REG_RANGE_CONFIG__SIGMA_THRESH, 360U)) goto fail_write;
    if (!VL53L1X_WriteWord(device, REG_RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS, 192U)) goto fail_write;

    /* ── 9. 动态配置。 ── */
    if (!VL53L1X_WriteByte(device, REG_SYSTEM__GROUPED_PARAMETER_HOLD_0, 0x01U)) goto fail_write;
    if (!VL53L1X_WriteByte(device, REG_SYSTEM__GROUPED_PARAMETER_HOLD_1, 0x01U)) goto fail_write;
    if (!VL53L1X_WriteByte(device, REG_SD_CONFIG__QUANTIFIER, 2U)) goto fail_write;

    if (!VL53L1X_WriteByte(device, REG_SYSTEM__GROUPED_PARAMETER_HOLD, 0x00U)) goto fail_write;
    if (!VL53L1X_WriteByte(device, REG_SYSTEM__SEED_CONFIG, 1U)) goto fail_write;

    /* ── 10. 低功耗自动模式的序列配置。 ── */
    if (!VL53L1X_WriteByte(device, REG_SYSTEM__SEQUENCE_CONFIG, 0x8BU)) goto fail_write;   /* VHV | PHASECAL | DSS1 | RANGE */
    if (!VL53L1X_WriteWord(device, REG_DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, (uint16_t)(200U << 8))) goto fail_write;
    if (!VL53L1X_WriteByte(device, REG_DSS_CONFIG__ROI_MODE_CONTROL, 2U)) goto fail_write;  /* 请求式有效 SPAD 数 */
    /* 显式钉死中断配置：bit5 = 测距完成时置位。 */
    if (!VL53L1X_WriteByte(device, REG_SYSTEM__INTERRUPT_CONFIG_GPIO, 0x20U)) goto fail_write;

    /* ── 11. 设为短距档，并写 20ms 时间预算。 ── */
    if (!VL53L1X_SetShortRangeMode(device)) goto fail_write;
    if (!VL53L1X_SetTimingBudget(device, VL53L1X_TIMING_BUDGET_US)) goto fail_start;

    /* ── 12. 器件间距离偏移按外层偏移的 4 倍写入（假设 MM1/MM2 已关闭）。 ── */
    if (!VL53L1X_ReadWord(device, REG_MM_CONFIG__OUTER_OFFSET_MM, &outer_offset)) goto fail_write;
    if (!VL53L1X_WriteWord(device, REG_ALGO__PART_TO_PART_RANGE_OFFSET_MM, (uint16_t)(outer_offset * 4U))) goto fail_write;

    device->last_init_status = VL53L1X_INIT_OK;

    /* ── 13. 触发第一次单次测量；上层按节奏反复触发。
             注意：触发失败必须把 initialized 收回去，否则上层会以为设备可用
             却永远触发不了。 ── */
    device->initialized = true;
    if (!VL53L1X_StartSingleShot(device))
    {
        device->initialized = false;
        device->last_init_status = VL53L1X_INIT_START_FAIL;
        return false;
    }
    return true;

fail_write:
    device->last_init_status = VL53L1X_INIT_CONFIG_WRITE_FAIL;
    return false;

fail_start:
    device->last_init_status = VL53L1X_INIT_START_FAIL;
    return false;
}

bool VL53L1X_StartSingleShot(VL53L1X_Device_t *device)
{
    if ((device == NULL) || !device->initialized) return false;

    /* 先清中断，避免把上一拍的老结果当成新的。 */
    (void)VL53L1X_WriteByte(device, REG_SYSTEM__INTERRUPT_CLEAR, 0x01U);
    if (!VL53L1X_WriteByte(device, REG_SYSTEM__MODE_START, VL53L1X_MODE_SINGLE_SHOT))
    {
        device->last_init_status = VL53L1X_INIT_START_FAIL;
        return false;
    }
    return true;
}

bool VL53L1X_IsMeasurementDone(VL53L1X_Device_t *device)
{
    uint8_t range_status = 0U;

    if ((device == NULL) || !device->initialized) return false;
    if (!VL53L1X_ReadByte(device, REG_RESULT__RANGE_STATUS, &range_status)) return false;
    return (range_status == VL53L1X_RANGE_STATUS_COMPLETE);
}

bool VL53L1X_ReadMeasurement(VL53L1X_Device_t *device, VL53L1X_Result_t *result)
{
    if ((device == NULL) || (result == NULL) || !device->initialized) return false;

    if (!VL53L1X_ReadRawResults(device)) return false;

    if (!device->calibrated) VL53L1X_SetupManualCalibration(device);
    VL53L1X_UpdateDss(device);

    /* 距离换算：乘 2011 再除以 0x0800（含 +0x400 四舍五入），约等于 ×0.982。 */
    result->distance_mm = (uint16_t)((((uint32_t)vl53l1x_raw.range_mm_sd0 * 2011U) + 0x0400U) / 0x0800U);

    /* 把芯片原始状态码映射成"0 = 有效"。 */
    switch (vl53l1x_raw.range_status)
    {
    case 9U:  /* RANGECOMPLETE */
        result->range_status = (vl53l1x_raw.stream_count == 0U) ? 8U : 0U;
        break;
    case 8U:  result->range_status = 4U; break;  /* MINCLIP             -> 距离过近 */
    case 4U:  result->range_status = 2U; break;  /* MSRCNOTARGET        -> 信号太弱 */
    case 6U:  result->range_status = 1U; break;  /* SIGMATHRESHOLDCHECK -> 噪声过大 */
    case 5U:  result->range_status = 3U; break;  /* RANGEPHASECHECK     -> 超量程 */
    case 7U:  result->range_status = 5U; break;  /* PHASECONSISTENCY    -> 相位异常 */
    case 12U: result->range_status = 6U; break;  /* RANGEIGNORETHRESHOLD */
    case 13U: result->range_status = 7U; break;  /* USERROICLIP         -> 近距离裁剪 */
    case 18U: result->range_status = 9U; break;  /* GPHSTREAMCOUNT0READY */
    case 1U:
    case 2U:
    case 3U:
    case 17U: result->range_status = 10U; break; /* 硬件故障 */
    default:  result->range_status = 11U; break; /* 其它 */
    }

    result->valid = (result->range_status == 0U);
    result->error_count = device->error_count;

    /* 清中断，准备下一次。 */
    (void)VL53L1X_WriteByte(device, REG_SYSTEM__INTERRUPT_CLEAR, 0x01U);

    return true;
}
