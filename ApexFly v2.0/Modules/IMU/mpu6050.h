/**
 * @file mpu6050.h
 * @brief MPU6050 六轴惯性传感器底层驱动的公开定义。
 *
 * 本文件定义设备地址、量程换算系数、返回状态、数据结构和对外函数。
 * 本驱动只负责寄存器配置、数据读取和单位换算，不负责校准、滤波和姿态解算。
 */

/* 防止同一个源文件重复包含本头文件。 */
#ifndef MPU6050_H
/* 定义头文件保护宏。 */
#define MPU6050_H

/* 引入 STM32 HAL 类型，主要使用 I2C_HandleTypeDef。 */
#include "main.h"
/* 引入 uint8_t、uint16_t 和 int16_t。 */
#include <stdint.h>
/* 引入 bool、true 和 false。 */
#include <stdbool.h>

/* 如果由 C++ 文件包含，则继续使用 C 语言函数链接规则。 */
#ifdef __cplusplus
extern "C" {
#endif

/* AD0 接地时，MPU6050 的 7 位 I2C 地址为 0x68。 */
#define MPU6050_ADDRESS_7BIT 0x68U
/* STM32 HAL 要求设备地址左移一位，所以实际传入 HAL 的地址为 0xD0。 */
#define MPU6050_ADDRESS_HAL (MPU6050_ADDRESS_7BIT << 1U)
/* 加速度计配置为正负 8g 时，每 1g 对应 4096 个原始计数。 */
#define MPU6050_ACCEL_SCALE_8G 4096.0f
/* 陀螺仪配置为正负 2000 度每秒时，每 1 度每秒对应 16.4 个原始计数。 */
#define MPU6050_GYRO_SCALE_2000DPS 16.4f

/**
 * @brief MPU6050 驱动函数返回状态。
 *
 * 驱动函数返回该枚举后，上层可以区分参数错误、设备不存在、芯片 ID 错误、
 * I2C 错误和未初始化等情况，不需要依赖额外的全局错误变量。
 */
typedef enum
{
    /* 函数执行成功。 */
    MPU6050_STATUS_OK = 0,
    /* 调用者传入了空指针或零长度等非法参数。 */
    MPU6050_STATUS_INVALID_PARAM,
    /* 指定 I2C 地址没有收到设备应答。 */
    MPU6050_STATUS_NOT_FOUND,
    /* WHO_AM_I 返回值不是预期的 0x68。 */
    MPU6050_STATUS_BAD_ID,
    /* HAL I2C 读写失败或超时。 */
    MPU6050_STATUS_I2C_ERROR,
    /* 设备尚未完成初始化。 */
    MPU6050_STATUS_NOT_INITIALIZED
} MPU6050_Status_t;

/**
 * @brief MPU6050 寄存器中的原始有符号数据。
 *
 * 原始数据没有转换单位，适合检查通信是否正确以及传感器是否发生量程饱和。
 */
typedef struct
{
    /* X 轴加速度原始计数。 */
    int16_t accel_x;
    /* Y 轴加速度原始计数。 */
    int16_t accel_y;
    /* Z 轴加速度原始计数。 */
    int16_t accel_z;
    /* 芯片内部温度原始计数。 */
    int16_t temperature;
    /* X 轴陀螺仪原始计数。 */
    int16_t gyro_x;
    /* Y 轴陀螺仪原始计数。 */
    int16_t gyro_y;
    /* Z 轴陀螺仪原始计数。 */
    int16_t gyro_z;
} MPU6050_RawData_t;

/**
 * @brief 按当前量程转换后的物理量数据。
 *
 * 这些数据还没有扣除零偏，也没有进行坐标轴转换、低通滤波和姿态解算。
 */
typedef struct
{
    /* X 轴加速度，单位为 g。 */
    float accel_x_g;
    /* Y 轴加速度，单位为 g。 */
    float accel_y_g;
    /* Z 轴加速度，单位为 g。 */
    float accel_z_g;
    /* 芯片内部温度，单位为摄氏度。 */
    float temperature_c;
    /* X 轴角速度，单位为度每秒。 */
    float gyro_x_dps;
    /* Y 轴角速度，单位为度每秒。 */
    float gyro_y_dps;
    /* Z 轴角速度，单位为度每秒。 */
    float gyro_z_dps;
} MPU6050_Data_t;

/**
 * @brief MPU6050 设备对象。
 *
 * 设备对象保存 I2C 句柄、设备地址和初始化状态，使驱动不需要在内部写死 hi2c1。
 */
typedef struct
{
    /* 指向 CubeMX 生成的 I2C 句柄，当前板子使用时传入 &hi2c1。 */
    I2C_HandleTypeDef *i2c;
    /* 保存 HAL 使用的左移一位后的 I2C 地址。 */
    uint16_t address;
    /* true 表示初始化全部成功，false 表示暂时不能读取正式数据。 */
    bool initialized;
    /* 自设备初始化以来累计的 I2C 传输失败次数，用于统计总线健康度。 */
    uint32_t i2c_error_count;
} MPU6050_Device_t;

/**
 * @brief 检查设备、验证芯片 ID 并配置滤波器、采样率、量程和中断。
 * @param device MPU6050 设备对象地址。
 * @param i2c STM32 I2C 句柄地址，当前板子应传入 &hi2c1。
 * @return 初始化状态。
 */
MPU6050_Status_t MPU6050_Init(MPU6050_Device_t *device, I2C_HandleTypeDef *i2c);

/**
 * @brief 读取 WHO_AM_I 芯片身份寄存器。
 * @param device MPU6050 设备对象地址。
 * @param who_am_i 保存芯片 ID 的变量地址。
 * @return 读取状态；成功时 who_am_i 正常为 0x68。
 */
MPU6050_Status_t MPU6050_ReadWhoAmI(MPU6050_Device_t *device, uint8_t *who_am_i);

/**
 * @brief 连续读取加速度、温度和陀螺仪的 14 字节原始数据。
 * @param device 已成功初始化的设备对象地址。
 * @param raw_data 保存七组原始数据的结构体地址。
 * @return 数据读取状态。
 */
MPU6050_Status_t MPU6050_ReadRaw(MPU6050_Device_t *device, MPU6050_RawData_t *raw_data);

/**
 * @brief 读取原始数据并转换为 g、摄氏度和度每秒。
 * @param device 已成功初始化的设备对象地址。
 * @param data 保存换算数据的结构体地址。
 * @return 数据读取与换算状态。
 */
MPU6050_Status_t MPU6050_ReadData(MPU6050_Device_t *device, MPU6050_Data_t *data);

/**
 * @brief 获取自设备初始化以来累计的 I2C 传输错误次数。
 * @param device 设备对象地址。
 * @return I2C 传输失败累计次数。
 */
uint32_t MPU6050_GetErrorCount(const MPU6050_Device_t *device);

/**
 * @brief 清零 I2C 传输错误计数。
 * @param device 设备对象地址。
 */
void MPU6050_ResetErrorCount(MPU6050_Device_t *device);

/* C++ 兼容范围结束。 */
#ifdef __cplusplus
}
#endif

/* MPU6050_H 头文件保护范围结束。 */
#endif
