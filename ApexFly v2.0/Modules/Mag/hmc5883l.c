/**
 * @file hmc5883l.c
 * @brief GY-273 磁力计底层实现。DRDY 不接，靠状态位或定时读。
 *
 * HMC5883L 数据顺序是 X、Z、Y，必须一次读 6 字节，否则 LOCK。
 * QMC5883L 数据顺序是 X、Y、Z，小端。
 * QMC5883P 地址 0x2C，CHIP_ID 在 0x00，数据从 0x01 起，也是小端 XYZ。
 */

#include "hmc5883l.h"

#include "i2c2_lock.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

#define MAG_I2C_TIMEOUT_MS 20U
#define MAG_LOCK_TIMEOUT_MS 200U
#define MAG_SCAN_LOCK_TIMEOUT_MS 500U

#define HMC5883L_REG_CRA 0x00U
#define HMC5883L_REG_CRB 0x01U
#define HMC5883L_REG_MODE 0x02U
#define HMC5883L_REG_DATA_X_MSB 0x03U
#define HMC5883L_REG_STATUS 0x09U
#define HMC5883L_REG_IDA 0x0AU
#define HMC5883L_REG_IDB 0x0BU
#define HMC5883L_REG_IDC 0x0CU
/* 8 次平均、75 Hz、正常测量。 */
#define HMC5883L_CRA_75HZ_8AVG 0x78U
/* 增益 ±1.9 Ga，靠近电机时比默认 ±1.3 更不容易饱和。 */
#define HMC5883L_CRB_GAIN_1P9GA 0x20U
#define HMC5883L_MODE_CONTINUOUS 0x00U
#define HMC5883L_STATUS_RDY 0x01U
#define HMC5883L_COUNTS_PER_GAUSS 820.0f

#define QMC5883L_REG_DATA_X_LSB 0x00U
#define QMC5883L_REG_STATUS 0x06U
#define QMC5883L_REG_CONTROL1 0x09U
#define QMC5883L_REG_SETRESET 0x0BU
#define QMC5883L_REG_CHIP_ID 0x0DU
#define QMC5883L_STATUS_DRDY 0x01U
#define QMC5883L_STATUS_OVL 0x02U
/* 连续测量、50 Hz、±8 G、过采样 512。 */
#define QMC5883L_CONTROL1_50HZ_8G 0x1DU
#define QMC5883L_SETRESET_RECOMMENDED 0x01U
#define QMC5883L_COUNTS_PER_GAUSS 3000.0f

#define QMC5883P_REG_CHIP_ID 0x00U
#define QMC5883P_REG_DATA_X_LSB 0x01U
#define QMC5883P_REG_STATUS 0x09U
#define QMC5883P_REG_CONTROL1 0x0AU
#define QMC5883P_REG_CONTROL2 0x0BU
#define QMC5883P_CHIP_ID 0x80U
#define QMC5883P_STATUS_DRDY 0x01U
#define QMC5883P_STATUS_OVFL 0x02U
/* 连续测量、50 Hz、±8 G。 */
#define QMC5883P_CONTROL1_50HZ_8G 0xC3U
#define QMC5883P_CONTROL2_SETRESET 0x00U
#define QMC5883P_COUNTS_PER_GAUSS 1000.0f
#define QMC6308_REG_CHIP_ID 0x0DU
#define QMC6308_CHIP_ID 0x80U

static bool Mag_BusReady(I2C_HandleTypeDef *i2c, uint8_t address_7bit)
{
    bool ready;

    if (!I2C2_Lock(MAG_LOCK_TIMEOUT_MS)) return false;
    ready = (HAL_I2C_IsDeviceReady(i2c, (uint16_t)(address_7bit << 1), 3U, MAG_I2C_TIMEOUT_MS) == HAL_OK);
    I2C2_Unlock();
    return ready;
}

static bool Mag_ReadRegisters(MagDevice_t *device, uint8_t register_address, uint8_t *data, uint16_t length)
{
    bool ok;

    if (!I2C2_Lock(MAG_LOCK_TIMEOUT_MS)) return false;
    ok = (HAL_I2C_Mem_Read(device->i2c, device->address, register_address, I2C_MEMADD_SIZE_8BIT, data, length, MAG_I2C_TIMEOUT_MS) == HAL_OK);
    I2C2_Unlock();
    return ok;
}

static bool Mag_WriteRegister(MagDevice_t *device, uint8_t register_address, uint8_t value)
{
    bool ok;

    if (!I2C2_Lock(MAG_LOCK_TIMEOUT_MS)) return false;
    ok = (HAL_I2C_Mem_Write(device->i2c, device->address, register_address, I2C_MEMADD_SIZE_8BIT, &value, 1U, MAG_I2C_TIMEOUT_MS) == HAL_OK);
    I2C2_Unlock();
    return ok;
}

static int16_t Mag_Be16(const uint8_t *bytes)
{
    return (int16_t)(((uint16_t)bytes[0] << 8) | bytes[1]);
}

static int16_t Mag_Le16(const uint8_t *bytes)
{
    return (int16_t)(((uint16_t)bytes[1] << 8) | bytes[0]);
}

static void Mag_FillGauss(MagRawData_t *data, float counts_per_gauss)
{
    if (counts_per_gauss <= 0.0f)
    {
        data->x_gauss = 0.0f;
        data->y_gauss = 0.0f;
        data->z_gauss = 0.0f;
        data->magnitude_gauss = 0.0f;
        return;
    }
    data->x_gauss = (float)data->raw_x / counts_per_gauss;
    data->y_gauss = (float)data->raw_y / counts_per_gauss;
    data->z_gauss = (float)data->raw_z / counts_per_gauss;
    data->magnitude_gauss = sqrtf((data->x_gauss * data->x_gauss) + (data->y_gauss * data->y_gauss) + (data->z_gauss * data->z_gauss));
}

static bool Mag_InitHmc(MagDevice_t *device)
{
    uint8_t id[3];

    device->address = (uint16_t)(HMC5883L_I2C_ADDRESS << 1);
    device->chip = MAG_CHIP_HMC5883L;
    device->counts_per_gauss = HMC5883L_COUNTS_PER_GAUSS;
    /* 有应答就先按 HMC 配置；ID 不是 H43 也继续，避免假芯片卡死。 */
    (void)Mag_ReadRegisters(device, HMC5883L_REG_IDA, id, sizeof(id));
    if (!Mag_WriteRegister(device, HMC5883L_REG_CRA, HMC5883L_CRA_75HZ_8AVG)) return false;
    if (!Mag_WriteRegister(device, HMC5883L_REG_CRB, HMC5883L_CRB_GAIN_1P9GA)) return false;
    if (!Mag_WriteRegister(device, HMC5883L_REG_MODE, HMC5883L_MODE_CONTINUOUS)) return false;
    return true;
}

static bool Mag_InitQmc(MagDevice_t *device)
{
    uint8_t chip_id = 0U;

    device->address = (uint16_t)(QMC5883L_I2C_ADDRESS << 1);
    device->chip = MAG_CHIP_QMC5883L;
    device->counts_per_gauss = QMC5883L_COUNTS_PER_GAUSS;
    (void)Mag_ReadRegisters(device, QMC5883L_REG_CHIP_ID, &chip_id, 1U);
    (void)chip_id;
    if (!Mag_WriteRegister(device, QMC5883L_REG_SETRESET, QMC5883L_SETRESET_RECOMMENDED)) return false;
    if (!Mag_WriteRegister(device, QMC5883L_REG_CONTROL1, QMC5883L_CONTROL1_50HZ_8G)) return false;
    return true;
}

static bool Mag_InitQmcP(MagDevice_t *device)
{
    uint8_t chip_id = 0U;
    uint8_t chip_id_d = 0U;

    device->address = (uint16_t)(QMC5883P_I2C_ADDRESS << 1);
    device->counts_per_gauss = QMC5883P_COUNTS_PER_GAUSS;
    (void)Mag_ReadRegisters(device, QMC5883P_REG_CHIP_ID, &chip_id, 1U);
    (void)Mag_ReadRegisters(device, QMC6308_REG_CHIP_ID, &chip_id_d, 1U);
    if (chip_id == QMC5883P_CHIP_ID)
    {
        device->chip = MAG_CHIP_QMC5883P;
        if (!Mag_WriteRegister(device, QMC5883P_REG_CONTROL2, QMC5883P_CONTROL2_SETRESET)) return false;
        if (!Mag_WriteRegister(device, QMC5883P_REG_CONTROL1, QMC5883P_CONTROL1_50HZ_8G)) return false;
        return true;
    }
    /* 0x2C 但 CHIP_ID 在 0x0D，按 QMC6308 / 类 QMC5883L 布局配置。 */
    device->chip = MAG_CHIP_QMC6308;
    device->counts_per_gauss = QMC5883L_COUNTS_PER_GAUSS;
    if (!Mag_WriteRegister(device, QMC5883L_REG_SETRESET, QMC5883L_SETRESET_RECOMMENDED)) return false;
    if (!Mag_WriteRegister(device, QMC5883L_REG_CONTROL1, QMC5883L_CONTROL1_50HZ_8G)) return false;
    return true;
}

static bool Mag_ReadHmc(MagDevice_t *device, MagRawData_t *data)
{
    uint8_t status;
    uint8_t raw[6];

    if (!Mag_ReadRegisters(device, HMC5883L_REG_STATUS, &status, 1U)) return false;
    if ((status & HMC5883L_STATUS_RDY) == 0U) return false;
    /* 必须从 0x03 连续读完 X/Z/Y，否则输出锁死。 */
    if (!Mag_ReadRegisters(device, HMC5883L_REG_DATA_X_MSB, raw, sizeof(raw))) return false;
    data->raw_x = Mag_Be16(&raw[0]);
    data->raw_z = Mag_Be16(&raw[2]);
    data->raw_y = Mag_Be16(&raw[4]);
    data->overflow = ((data->raw_x == HMC5883L_OVERFLOW_COUNTS) || (data->raw_y == HMC5883L_OVERFLOW_COUNTS) || (data->raw_z == HMC5883L_OVERFLOW_COUNTS));
    Mag_FillGauss(data, device->counts_per_gauss);
    return !data->overflow;
}

static bool Mag_ReadQmc(MagDevice_t *device, MagRawData_t *data)
{
    uint8_t status;
    uint8_t raw[6];

    if (!Mag_ReadRegisters(device, QMC5883L_REG_STATUS, &status, 1U)) return false;
    if ((status & QMC5883L_STATUS_DRDY) == 0U) return false;
    data->overflow = ((status & QMC5883L_STATUS_OVL) != 0U);
    if (!Mag_ReadRegisters(device, QMC5883L_REG_DATA_X_LSB, raw, sizeof(raw))) return false;
    data->raw_x = Mag_Le16(&raw[0]);
    data->raw_y = Mag_Le16(&raw[2]);
    data->raw_z = Mag_Le16(&raw[4]);
    Mag_FillGauss(data, device->counts_per_gauss);
    return !data->overflow;
}

static bool Mag_ReadQmcP(MagDevice_t *device, MagRawData_t *data)
{
    uint8_t status;
    uint8_t raw[6];

    if (device->chip == MAG_CHIP_QMC6308) return Mag_ReadQmc(device, data);
    if (!Mag_ReadRegisters(device, QMC5883P_REG_STATUS, &status, 1U)) return false;
    if ((status & QMC5883P_STATUS_DRDY) == 0U) return false;
    data->overflow = ((status & QMC5883P_STATUS_OVFL) != 0U);
    if (!Mag_ReadRegisters(device, QMC5883P_REG_DATA_X_LSB, raw, sizeof(raw))) return false;
    data->raw_x = Mag_Le16(&raw[0]);
    data->raw_y = Mag_Le16(&raw[2]);
    data->raw_z = Mag_Le16(&raw[4]);
    Mag_FillGauss(data, device->counts_per_gauss);
    return !data->overflow;
}

bool Mag_ScanBus(I2C_HandleTypeDef *i2c, MagProbe_t *probe)
{
    uint8_t address_7bit;

    if ((i2c == NULL) || (probe == NULL)) return false;
    memset(probe, 0, sizeof(*probe));
    if (!I2C2_Lock(MAG_SCAN_LOCK_TIMEOUT_MS))
    {
        probe->fail_code = 3U;
        return false;
    }

    for (address_7bit = 0x08U; address_7bit <= 0x77U; address_7bit++)
    {
        if (HAL_I2C_IsDeviceReady(i2c, (uint16_t)(address_7bit << 1), 1U, MAG_I2C_TIMEOUT_MS) != HAL_OK) continue;
        if (probe->found_count < 6U)
        {
            probe->found_addrs[probe->found_count] = address_7bit;
            probe->found_count++;
        }
        if (address_7bit == HMC5883L_I2C_ADDRESS) probe->hmc_ack = true;
        if (address_7bit == QMC5883L_I2C_ADDRESS) probe->qmc_ack = true;
        if (address_7bit == QMC5883P_I2C_ADDRESS) probe->qmc_p_ack = true;
        if (address_7bit == 0x76U) probe->baro_ack = true;
    }

    if (probe->hmc_ack)
    {
        (void)HAL_I2C_Mem_Read(i2c, (uint16_t)(HMC5883L_I2C_ADDRESS << 1), HMC5883L_REG_IDA, I2C_MEMADD_SIZE_8BIT, probe->hmc_id, 3U, MAG_I2C_TIMEOUT_MS);
    }
    if (probe->qmc_ack)
    {
        (void)HAL_I2C_Mem_Read(i2c, (uint16_t)(QMC5883L_I2C_ADDRESS << 1), QMC5883L_REG_CHIP_ID, I2C_MEMADD_SIZE_8BIT, &probe->qmc_id, 1U, MAG_I2C_TIMEOUT_MS);
    }
    if (probe->qmc_p_ack)
    {
        (void)HAL_I2C_Mem_Read(i2c, (uint16_t)(QMC5883P_I2C_ADDRESS << 1), QMC5883P_REG_CHIP_ID, I2C_MEMADD_SIZE_8BIT, &probe->qmc_p_id, 1U, MAG_I2C_TIMEOUT_MS);
        (void)HAL_I2C_Mem_Read(i2c, (uint16_t)(QMC5883P_I2C_ADDRESS << 1), QMC6308_REG_CHIP_ID, I2C_MEMADD_SIZE_8BIT, &probe->qmc_p_id_d, 1U, MAG_I2C_TIMEOUT_MS);
    }
    I2C2_Unlock();
    if (probe->found_count == 0U) probe->fail_code = 1U;
    return probe->found_count > 0U;
}

bool Mag_DeviceInit(MagDevice_t *device, I2C_HandleTypeDef *i2c)
{
    if ((device == NULL) || (i2c == NULL)) return false;
    memset(device, 0, sizeof(*device));
    device->i2c = i2c;

    if (Mag_BusReady(i2c, HMC5883L_I2C_ADDRESS) && Mag_InitHmc(device))
    {
        device->initialized = true;
        return true;
    }
    memset(device, 0, sizeof(*device));
    device->i2c = i2c;
    if (Mag_BusReady(i2c, QMC5883L_I2C_ADDRESS) && Mag_InitQmc(device))
    {
        device->initialized = true;
        return true;
    }
    memset(device, 0, sizeof(*device));
    device->i2c = i2c;
    if (Mag_BusReady(i2c, QMC5883P_I2C_ADDRESS) && Mag_InitQmcP(device))
    {
        device->initialized = true;
        return true;
    }
    memset(device, 0, sizeof(*device));
    return false;
}

bool Mag_DeviceRead(MagDevice_t *device, MagRawData_t *data)
{
    bool ok;

    if ((device == NULL) || (data == NULL) || !device->initialized) return false;
    memset(data, 0, sizeof(*data));
    if (device->chip == MAG_CHIP_HMC5883L) ok = Mag_ReadHmc(device, data);
    else if (device->chip == MAG_CHIP_QMC5883L) ok = Mag_ReadQmc(device, data);
    else if ((device->chip == MAG_CHIP_QMC5883P) || (device->chip == MAG_CHIP_QMC6308)) ok = Mag_ReadQmcP(device, data);
    else return false;
    if (!ok) return false;
    device->sample_count++;
    device->data = *data;
    device->data.sample_count = device->sample_count;
    data->sample_count = device->sample_count;
    return true;
}

MagChip_t Mag_DeviceChip(const MagDevice_t *device)
{
    if (device == NULL) return MAG_CHIP_NONE;
    return device->chip;
}

uint8_t Mag_DeviceAddress7Bit(const MagDevice_t *device)
{
    if ((device == NULL) || !device->initialized) return 0U;
    return (uint8_t)(device->address >> 1);
}
