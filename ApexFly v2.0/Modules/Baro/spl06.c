/**
 * @file spl06.c
 * @brief SPL06-001 驱动实现。寄存器布局和补偿公式沿用可飞老板，Init 里用 osDelay 让出 CPU。
 */

#include "spl06.h"

#include "cmsis_os.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

#define SPL06_REG_PSR_B2 0x00U
#define SPL06_REG_PRS_CFG 0x06U
#define SPL06_REG_TMP_CFG 0x07U
#define SPL06_REG_MEAS_CFG 0x08U
#define SPL06_REG_CFG_REG 0x09U
#define SPL06_REG_RESET 0x0CU
#define SPL06_REG_PRODUCT_ID 0x0DU
#define SPL06_REG_COEF_START 0x10U
#define SPL06_REG_COEF_SOURCE 0x28U

#define SPL06_RESET_COMMAND 0x09U
#define SPL06_READY_SENSOR 0x40U
#define SPL06_READY_COEFFICIENT 0x80U
#define SPL06_READY_PRESSURE 0x20U
#define SPL06_READY_TEMPERATURE 0x10U

/* 压力用 8 Hz / OSR32 降噪；温度只参与内部补偿，8 Hz / OSR8 足够。 */
#define SPL06_PRESSURE_8HZ_OSR_32 0x35U
#define SPL06_TEMPERATURE_8HZ_OSR_8 0x33U
#define SPL06_CFG_PRESSURE_SHIFT 0x04U
#define SPL06_MEASURE_CONTINUOUS_PRESSURE_TEMPERATURE 0x07U
#define SPL06_SCALE_OSR_32 516096.0f
#define SPL06_SCALE_OSR_8 7864320.0f

static bool SPL06_ReadRegisters(SPL06_Device_t *device, uint8_t register_address, uint8_t *data, uint16_t length)
{
    return HAL_I2C_Mem_Read(device->i2c, device->address, register_address, I2C_MEMADD_SIZE_8BIT, data, length, 100U) == HAL_OK;
}

static bool SPL06_WriteRegister(SPL06_Device_t *device, uint8_t register_address, uint8_t value)
{
    return HAL_I2C_Mem_Write(device->i2c, device->address, register_address, I2C_MEMADD_SIZE_8BIT, &value, 1U, 100U) == HAL_OK;
}

static int32_t SPL06_SignExtend(uint32_t value, uint8_t bits)
{
    uint32_t sign_mask = 1UL << (bits - 1U);
    uint32_t value_mask = (1UL << bits) - 1UL;

    value &= value_mask;
    if ((value & sign_mask) != 0U) value |= ~value_mask;
    return (int32_t)value;
}

static int32_t SPL06_DecodeSigned24(const uint8_t *bytes)
{
    uint32_t value = ((uint32_t)bytes[0] << 16) | ((uint32_t)bytes[1] << 8) | bytes[2];
    return SPL06_SignExtend(value, 24U);
}

/* 18 字节校准区按手册拼接 12/16/20 位有符号系数，直接当 int16 会把负系数读错。 */
static bool SPL06_ReadCalibration(SPL06_Device_t *device)
{
    uint8_t buffer[18];
    SPL06_Calibration_t *coefficient = &device->calibration;

    if (!SPL06_ReadRegisters(device, SPL06_REG_COEF_START, buffer, sizeof(buffer))) return false;
    coefficient->c0 = (int16_t)SPL06_SignExtend(((uint32_t)buffer[0] << 4) | (buffer[1] >> 4), 12U);
    coefficient->c1 = (int16_t)SPL06_SignExtend(((uint32_t)(buffer[1] & 0x0FU) << 8) | buffer[2], 12U);
    coefficient->c00 = SPL06_SignExtend(((uint32_t)buffer[3] << 12) | ((uint32_t)buffer[4] << 4) | (buffer[5] >> 4), 20U);
    coefficient->c10 = SPL06_SignExtend(((uint32_t)(buffer[5] & 0x0FU) << 16) | ((uint32_t)buffer[6] << 8) | buffer[7], 20U);
    coefficient->c01 = (int16_t)(((uint16_t)buffer[8] << 8) | buffer[9]);
    coefficient->c11 = (int16_t)(((uint16_t)buffer[10] << 8) | buffer[11]);
    coefficient->c20 = (int16_t)(((uint16_t)buffer[12] << 8) | buffer[13]);
    coefficient->c21 = (int16_t)(((uint16_t)buffer[14] << 8) | buffer[15]);
    coefficient->c30 = (int16_t)(((uint16_t)buffer[16] << 8) | buffer[17]);
    return true;
}

bool SPL06_InitAt(SPL06_Device_t *device, I2C_HandleTypeDef *i2c, uint8_t address_7bit)
{
    uint8_t status = 0U;
    uint8_t coefficient_source;
    uint8_t temperature_config;
    uint32_t waited_ms;

    if ((device == NULL) || (i2c == NULL)) return false;
    memset(device, 0, sizeof(*device));
    device->i2c = i2c;
    device->address = (uint16_t)(address_7bit << 1);
    device->pressure_scale = SPL06_SCALE_OSR_32;
    device->temperature_scale = SPL06_SCALE_OSR_8;

    if (HAL_I2C_IsDeviceReady(device->i2c, device->address, 3U, 100U) != HAL_OK) return false;
    if (!SPL06_WriteRegister(device, SPL06_REG_RESET, SPL06_RESET_COMMAND)) return false;
    osDelay(50U);

    waited_ms = 0U;
    do
    {
        if (!SPL06_ReadRegisters(device, SPL06_REG_MEAS_CFG, &status, 1U)) return false;
        if ((status & (SPL06_READY_SENSOR | SPL06_READY_COEFFICIENT)) == (SPL06_READY_SENSOR | SPL06_READY_COEFFICIENT)) break;
        osDelay(10U);
        waited_ms += 10U;
    } while (waited_ms < 500U);

    if ((status & (SPL06_READY_SENSOR | SPL06_READY_COEFFICIENT)) != (SPL06_READY_SENSOR | SPL06_READY_COEFFICIENT)) return false;
    if (!SPL06_ReadRegisters(device, SPL06_REG_PRODUCT_ID, &device->product_id, 1U)) return false;
    if ((device->product_id & 0xF0U) != 0x10U) return false;
    if (!SPL06_ReadCalibration(device)) return false;
    if (!SPL06_ReadRegisters(device, SPL06_REG_COEF_SOURCE, &coefficient_source, 1U)) return false;

    temperature_config = SPL06_TEMPERATURE_8HZ_OSR_8;
    if ((coefficient_source & 0x80U) != 0U) temperature_config |= 0x80U;

    if (!SPL06_WriteRegister(device, SPL06_REG_PRS_CFG, SPL06_PRESSURE_8HZ_OSR_32)) return false;
    if (!SPL06_WriteRegister(device, SPL06_REG_TMP_CFG, temperature_config)) return false;
    if (!SPL06_WriteRegister(device, SPL06_REG_CFG_REG, SPL06_CFG_PRESSURE_SHIFT)) return false;
    if (!SPL06_WriteRegister(device, SPL06_REG_MEAS_CFG, SPL06_MEASURE_CONTINUOUS_PRESSURE_TEMPERATURE)) return false;
    osDelay(150U);
    device->initialized = true;
    return true;
}

bool SPL06_Init(SPL06_Device_t *device, I2C_HandleTypeDef *i2c)
{
    if (SPL06_InitAt(device, i2c, SPL06_I2C_ADDRESS)) return true;
    return SPL06_InitAt(device, i2c, SPL06_I2C_ADDRESS_ALT);
}

bool SPL06_ReadData(SPL06_Device_t *device, SPL06_Data_t *data)
{
    uint8_t status;
    uint8_t raw[6];
    int32_t raw_pressure;
    int32_t raw_temperature;
    float pressure_scaled;
    float temperature_scaled;
    SPL06_Calibration_t *coefficient;

    if ((device == NULL) || (data == NULL) || !device->initialized) return false;
    if (!SPL06_ReadRegisters(device, SPL06_REG_MEAS_CFG, &status, 1U)) return false;
    if ((status & (SPL06_READY_PRESSURE | SPL06_READY_TEMPERATURE)) != (SPL06_READY_PRESSURE | SPL06_READY_TEMPERATURE)) return false;
    if (!SPL06_ReadRegisters(device, SPL06_REG_PSR_B2, raw, sizeof(raw))) return false;

    raw_pressure = SPL06_DecodeSigned24(&raw[0]);
    raw_temperature = SPL06_DecodeSigned24(&raw[3]);
    /* ready 位可能保持为 1，同一组寄存器结果只能发布一次。 */
    if ((device->data.sample_count > 0U) && (raw_pressure == device->data.raw_pressure)) return false;
    device->data.raw_pressure = raw_pressure;
    device->data.raw_temperature = raw_temperature;
    pressure_scaled = (float)raw_pressure / device->pressure_scale;
    temperature_scaled = (float)raw_temperature / device->temperature_scale;
    coefficient = &device->calibration;

    device->data.temperature_c = (float)coefficient->c0 * 0.5f + (float)coefficient->c1 * temperature_scaled;
    device->data.pressure_pa = (float)coefficient->c00 + pressure_scaled * ((float)coefficient->c10 + pressure_scaled * ((float)coefficient->c20 + pressure_scaled * (float)coefficient->c30)) + temperature_scaled * (float)coefficient->c01 + temperature_scaled * pressure_scaled * ((float)coefficient->c11 + pressure_scaled * (float)coefficient->c21);

    if (device->data.pressure_pa > 1000.0f) device->data.altitude_m = 44330.0f * (1.0f - powf(device->data.pressure_pa / 101325.0f, 0.19029495f));
    else device->data.altitude_m = 0.0f;

    device->data.sample_count++;
    *data = device->data;
    return true;
}
