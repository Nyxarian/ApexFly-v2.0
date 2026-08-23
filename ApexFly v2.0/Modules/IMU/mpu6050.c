/**
 * @file mpu6050.c
 * @brief MPU6050 六轴惯性传感器底层驱动实现。
 *
 * 本文件完成设备寻址、WHO_AM_I 验证、寄存器初始化、14 字节连续读取和单位换算。
 */

/* 引入 MPU6050 公开类型和函数声明。 */
#include "mpu6050.h"
/* 提供 memset()。 */
#include <string.h>
/* 提供 NULL。 */
#include <stddef.h>

/* 采样率分频寄存器。 */
#define MPU6050_REG_SMPLRT_DIV 0x19U
/* 数字低通滤波器配置寄存器。 */
#define MPU6050_REG_CONFIG 0x1AU
/* 陀螺仪量程配置寄存器。 */
#define MPU6050_REG_GYRO_CONFIG 0x1BU
/* 加速度计量程配置寄存器。 */
#define MPU6050_REG_ACCEL_CONFIG 0x1CU
/* INT 引脚电气特性配置寄存器。 */
#define MPU6050_REG_INT_PIN_CFG 0x37U
/* 中断功能使能寄存器。 */
#define MPU6050_REG_INT_ENABLE 0x38U
/* 连续 14 字节传感器数据的起始地址。 */
#define MPU6050_REG_ACCEL_XOUT_H 0x3BU
/* 电源和时钟源配置寄存器。 */
#define MPU6050_REG_PWR_MGMT_1 0x6BU
/* 各传感器轴待机控制寄存器。 */
#define MPU6050_REG_PWR_MGMT_2 0x6CU
/* 芯片身份寄存器。 */
#define MPU6050_REG_WHO_AM_I 0x75U
/* AD0 接地时正确的 WHO_AM_I 返回值。 */
#define MPU6050_WHO_AM_I_VALUE 0x68U
/* 单次阻塞 I2C 操作的最大等待时间，单位为毫秒。 */
#define MPU6050_I2C_TIMEOUT_MS 20U

/**
 * @brief 向 MPU6050 的一个寄存器写入一个字节。
 * @param device 保存 I2C 句柄和地址的设备对象。
 * @param register_address 目标寄存器地址。
 * @param value 写入寄存器的数值。
 * @return 寄存器写入状态。
 */
static MPU6050_Status_t MPU6050_WriteRegister(MPU6050_Device_t *device, uint8_t register_address, uint8_t value)
{
    /* 保存 HAL I2C 写函数的返回状态。 */
    HAL_StatusTypeDef hal_status;

    /* 设备对象或 I2C 句柄为空时不能访问总线。 */
    if ((device == NULL) || (device->i2c == NULL))
    {
        /* 返回参数错误。 */
        return MPU6050_STATUS_INVALID_PARAM;
    }

    /* 使用 8 位寄存器地址模式阻塞写入一个字节。 */
    hal_status = HAL_I2C_Mem_Write(device->i2c, device->address, register_address, I2C_MEMADD_SIZE_8BIT, &value, 1U, MPU6050_I2C_TIMEOUT_MS);

    /* HAL 没有返回 HAL_OK 时，可能发生 NACK、总线错误或超时。 */
    if (hal_status != HAL_OK)
    {
        /* 记录一次 I2C 传输失败，供上层统计总线健康度。 */
        device->i2c_error_count++;
        /* 将 HAL 错误转换成 MPU6050 驱动层错误。 */
        return MPU6050_STATUS_I2C_ERROR;
    }

    /* 寄存器写入成功。 */
    return MPU6050_STATUS_OK;
}

/**
 * @brief 从指定寄存器开始连续读取多个字节。
 * @param device 保存 I2C 句柄和地址的设备对象。
 * @param register_address 连续读取的起始寄存器地址。
 * @param data 接收数据的缓冲区地址。
 * @param length 需要读取的字节数量。
 * @return 连续读取状态。
 */
static MPU6050_Status_t MPU6050_ReadRegisters(MPU6050_Device_t *device, uint8_t register_address, uint8_t *data, uint16_t length)
{
    /* 保存 HAL I2C 读函数的返回状态。 */
    HAL_StatusTypeDef hal_status;

    /* 检查设备对象、I2C 句柄、接收缓冲区和读取长度。 */
    if ((device == NULL) || (device->i2c == NULL) || (data == NULL) || (length == 0U))
    {
        /* 任意必要参数无效时返回参数错误。 */
        return MPU6050_STATUS_INVALID_PARAM;
    }

    /* 使用 8 位寄存器地址模式阻塞读取指定数量的字节。 */
    hal_status = HAL_I2C_Mem_Read(device->i2c, device->address, register_address, I2C_MEMADD_SIZE_8BIT, data, length, MPU6050_I2C_TIMEOUT_MS);

    /* 判断 HAL I2C 读取是否成功。 */
    if (hal_status != HAL_OK)
    {
        /* 记录一次 I2C 传输失败，供上层统计总线健康度。 */
        device->i2c_error_count++;
        /* 将 HAL 错误转换成 MPU6050 驱动层错误。 */
        return MPU6050_STATUS_I2C_ERROR;
    }

    /* 数据已经写入调用者提供的缓冲区。 */
    return MPU6050_STATUS_OK;
}

/**
 * @brief 读取 WHO_AM_I 芯片身份寄存器。
 */
MPU6050_Status_t MPU6050_ReadWhoAmI(MPU6050_Device_t *device, uint8_t *who_am_i)
{
    /* 调用者必须提供保存芯片 ID 的变量地址。 */
    if (who_am_i == NULL)
    {
        /* 返回参数错误。 */
        return MPU6050_STATUS_INVALID_PARAM;
    }

    /* 从 WHO_AM_I 寄存器读取一个字节并直接返回读取状态。 */
    return MPU6050_ReadRegisters(device, MPU6050_REG_WHO_AM_I, who_am_i, 1U);
}

/**
 * @brief 检测并初始化 MPU6050。
 */
MPU6050_Status_t MPU6050_Init(MPU6050_Device_t *device, I2C_HandleTypeDef *i2c)
{
    /* 保存每一步驱动操作的状态。 */
    MPU6050_Status_t status;
    /* 保存 WHO_AM_I 读取结果。 */
    uint8_t who_am_i;

    /* 设备对象和 I2C 句柄不能为空。 */
    if ((device == NULL) || (i2c == NULL))
    {
        /* 缺少必要参数时返回参数错误。 */
        return MPU6050_STATUS_INVALID_PARAM;
    }

    /* 清除设备对象中可能存在的旧状态。 */
    memset(device, 0, sizeof(MPU6050_Device_t));
    /* 保存 CubeMX 生成的 I2C 句柄。 */
    device->i2c = i2c;
    /* 保存左移一位后的 HAL I2C 地址。 */
    device->address = MPU6050_ADDRESS_HAL;
    /* 全部寄存器配置完成之前保持未初始化状态。 */
    device->initialized = false;

    /* 最多尝试三次寻址，确认设备能够应答。 */
    if (HAL_I2C_IsDeviceReady(device->i2c, device->address, 3U, MPU6050_I2C_TIMEOUT_MS) != HAL_OK)
    {
        /* 无应答时应检查供电、地址、接线和 I2C 上拉电阻。 */
        return MPU6050_STATUS_NOT_FOUND;
    }

    /* 读取芯片身份寄存器。 */
    status = MPU6050_ReadWhoAmI(device, &who_am_i);

    /* WHO_AM_I 读取失败时不继续初始化。 */
    if (status != MPU6050_STATUS_OK)
    {
        /* 返回具体读取错误。 */
        return status;
    }

    /* MPU6050 在当前地址下应返回 0x68。 */
    if (who_am_i != MPU6050_WHO_AM_I_VALUE)
    {
        /* 有设备应答但身份不正确。 */
        return MPU6050_STATUS_BAD_ID;
    }

    /* 清除睡眠位并选择 X 轴陀螺仪 PLL 作为时钟源。 */
    status = MPU6050_WriteRegister(device, MPU6050_REG_PWR_MGMT_1, 0x01U);
    /* 检查电源管理寄存器是否写入成功。 */
    if (status != MPU6050_STATUS_OK)
    {
        /* 返回寄存器写入错误。 */
        return status;
    }

    /* 保持三轴加速度计和三轴陀螺仪全部启用。 */
    status = MPU6050_WriteRegister(device, MPU6050_REG_PWR_MGMT_2, 0x00U);
    /* 检查传感器轴使能配置。 */
    if (status != MPU6050_STATUS_OK)
    {
        /* 返回寄存器写入错误。 */
        return status;
    }

    /* 设置 DLPF_CFG=3，陀螺仪带宽约 42Hz，加速度计带宽约 44Hz。 */
    status = MPU6050_WriteRegister(device, MPU6050_REG_CONFIG, 0x03U);
    /* 检查数字低通滤波器配置。 */
    if (status != MPU6050_STATUS_OK)
    {
        /* 返回寄存器写入错误。 */
        return status;
    }

    /* 分频值为零，DLPF 开启时传感器输出采样率约为 1kHz。 */
    status = MPU6050_WriteRegister(device, MPU6050_REG_SMPLRT_DIV, 0x00U);
    /* 检查采样率配置。 */
    if (status != MPU6050_STATUS_OK)
    {
        /* 返回寄存器写入错误。 */
        return status;
    }

    /* 设置 FS_SEL=3，将陀螺仪量程配置为正负 2000 度每秒。 */
    status = MPU6050_WriteRegister(device, MPU6050_REG_GYRO_CONFIG, 0x18U);
    /* 检查陀螺仪量程配置。 */
    if (status != MPU6050_STATUS_OK)
    {
        /* 返回寄存器写入错误。 */
        return status;
    }

    /* 设置 AFS_SEL=2，将加速度计量程配置为正负 8g。 */
    status = MPU6050_WriteRegister(device, MPU6050_REG_ACCEL_CONFIG, 0x10U);
    /* 检查加速度计量程配置。 */
    if (status != MPU6050_STATUS_OK)
    {
        /* 返回寄存器写入错误。 */
        return status;
    }

    /* INT 使用高电平有效、推挽输出和短脉冲模式。 */
    status = MPU6050_WriteRegister(device, MPU6050_REG_INT_PIN_CFG, 0x00U);
    /* 检查中断引脚配置。 */
    if (status != MPU6050_STATUS_OK)
    {
        /* 返回寄存器写入错误。 */
        return status;
    }

    /* 开启 Data Ready 中断，新数据产生时 INT 引脚输出脉冲。 */
    status = MPU6050_WriteRegister(device, MPU6050_REG_INT_ENABLE, 0x01U);
    /* 检查数据就绪中断配置。 */
    if (status != MPU6050_STATUS_OK)
    {
        /* 返回寄存器写入错误。 */
        return status;
    }

    /* 只有全部配置成功后才标记设备可以正式读取。 */
    device->initialized = true;
    /* 返回初始化成功。 */
    return MPU6050_STATUS_OK;
}

/**
 * @brief 连续读取七组原始传感器数据。
 */
MPU6050_Status_t MPU6050_ReadRaw(MPU6050_Device_t *device, MPU6050_RawData_t *raw_data)
{
    /* 保存 14 字节连续读取状态。 */
    MPU6050_Status_t status;
    /* 保存 AX、AY、AZ、温度、GX、GY、GZ 的高低字节。 */
    uint8_t buffer[14];

    /* 检查设备对象和输出结构体地址。 */
    if ((device == NULL) || (raw_data == NULL))
    {
        /* 返回参数错误。 */
        return MPU6050_STATUS_INVALID_PARAM;
    }

    /* 初始化失败时禁止读取正式数据。 */
    if (device->initialized == false)
    {
        /* 返回未初始化状态。 */
        return MPU6050_STATUS_NOT_INITIALIZED;
    }

    /* 从 ACCEL_XOUT_H 开始一次性读取 14 字节。 */
    status = MPU6050_ReadRegisters(device, MPU6050_REG_ACCEL_XOUT_H, buffer, sizeof(buffer));

    /* I2C 读取失败时不解析缓冲区。 */
    if (status != MPU6050_STATUS_OK)
    {
        /* 返回底层读取错误。 */
        return status;
    }

    /* 组合加速度 X 的高低字节。 */
    raw_data->accel_x = (int16_t)(((uint16_t)buffer[0] << 8U) | buffer[1]);
    /* 组合加速度 Y 的高低字节。 */
    raw_data->accel_y = (int16_t)(((uint16_t)buffer[2] << 8U) | buffer[3]);
    /* 组合加速度 Z 的高低字节。 */
    raw_data->accel_z = (int16_t)(((uint16_t)buffer[4] << 8U) | buffer[5]);
    /* 组合温度的高低字节。 */
    raw_data->temperature = (int16_t)(((uint16_t)buffer[6] << 8U) | buffer[7]);
    /* 组合陀螺仪 X 的高低字节。 */
    raw_data->gyro_x = (int16_t)(((uint16_t)buffer[8] << 8U) | buffer[9]);
    /* 组合陀螺仪 Y 的高低字节。 */
    raw_data->gyro_y = (int16_t)(((uint16_t)buffer[10] << 8U) | buffer[11]);
    /* 组合陀螺仪 Z 的高低字节。 */
    raw_data->gyro_z = (int16_t)(((uint16_t)buffer[12] << 8U) | buffer[13]);

    /* 七组原始数据已经写入 raw_data。 */
    return MPU6050_STATUS_OK;
}

/**
 * @brief 读取原始数据并转换成物理单位。
 */
MPU6050_Status_t MPU6050_ReadData(MPU6050_Device_t *device, MPU6050_Data_t *data)
{
    /* 保存原始数据读取状态。 */
    MPU6050_Status_t status;
    /* 临时保存七组寄存器原始数据。 */
    MPU6050_RawData_t raw_data;

    /* 调用者必须提供有效的数据输出结构体。 */
    if (data == NULL)
    {
        /* 返回参数错误。 */
        return MPU6050_STATUS_INVALID_PARAM;
    }

    /* 获取一组完整的原始传感器数据。 */
    status = MPU6050_ReadRaw(device, &raw_data);

    /* 原始数据读取失败时不进行单位换算。 */
    if (status != MPU6050_STATUS_OK)
    {
        /* 返回原始读取错误。 */
        return status;
    }

    /* 正负 8g 量程下，原始值除以 4096 得到 X 轴加速度。 */
    data->accel_x_g = (float)raw_data.accel_x / MPU6050_ACCEL_SCALE_8G;
    /* 正负 8g 量程下，原始值除以 4096 得到 Y 轴加速度。 */
    data->accel_y_g = (float)raw_data.accel_y / MPU6050_ACCEL_SCALE_8G;
    /* 正负 8g 量程下，原始值除以 4096 得到 Z 轴加速度。 */
    data->accel_z_g = (float)raw_data.accel_z / MPU6050_ACCEL_SCALE_8G;
    /* 按数据手册公式把内部温度原始值转换为摄氏度。 */
    data->temperature_c = (float)raw_data.temperature / 340.0f + 36.53f;
    /* 正负 2000 度每秒量程下，原始值除以 16.4 得到 X 轴角速度。 */
    data->gyro_x_dps = (float)raw_data.gyro_x / MPU6050_GYRO_SCALE_2000DPS;
    /* 正负 2000 度每秒量程下，原始值除以 16.4 得到 Y 轴角速度。 */
    data->gyro_y_dps = (float)raw_data.gyro_y / MPU6050_GYRO_SCALE_2000DPS;
    /* 正负 2000 度每秒量程下，原始值除以 16.4 得到 Z 轴角速度。 */
    data->gyro_z_dps = (float)raw_data.gyro_z / MPU6050_GYRO_SCALE_2000DPS;

    /* 数据读取和单位换算全部完成，记录采样时间戳供上层计算真实采样间隔。 */
    data->timestamp_ms = HAL_GetTick();
    device->last_timestamp_ms = data->timestamp_ms;
    return MPU6050_STATUS_OK;
}

/**
 * @brief 获取最近一次成功读取数据样本的时间戳。
 */
uint32_t MPU6050_GetLastTimestampMs(const MPU6050_Device_t *device)
{
    if (device == NULL) return 0U;
    return device->last_timestamp_ms;
}

/**
 * @brief 获取自设备初始化以来累计的 I2C 传输错误次数。
 */
uint32_t MPU6050_GetErrorCount(const MPU6050_Device_t *device)
{
    if (device == NULL) return 0U;
    return device->i2c_error_count;
}

/**
 * @brief 清零 I2C 传输错误计数。
 */
void MPU6050_ResetErrorCount(MPU6050_Device_t *device)
{
    if (device == NULL) return;
    device->i2c_error_count = 0U;
}
