/**
 * @file hmc5883l.h
 * @brief GY-273 磁力计底层。只负责 I2C 探测、初始化和读三轴原始磁场。
 *
 * 本板实际焊的是 QMC5883P（地址 0x2C，CHIP_ID 在 0x00，数据从 0x01 起，小端 XYZ）。
 * 2026-09-11 清理：HMC5883L(0x1E)、QMC5883L(0x0D)、QMC6308 的兼容分支已删除，
 * 只保留在用的 QMC5883P。将来若换芯片，在这里重新加一路判型即可。
 *
 * 只接 VCC/GND/SCL/SDA，不使用 DRDY。连续测量后由上层定时读。
 * 本驱动不做硬铁校准、倾角补偿、航向融合，也不写 PWM。
 */

#ifndef HMC5883L_H
#define HMC5883L_H

#include "stm32g4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* QMC5883P 的 7 位地址。 */
#define QMC5883P_I2C_ADDRESS 0x2CU

typedef enum
{
    /* 还没探测到芯片。 */
    MAG_CHIP_NONE = 0,
    /* QMC5883P：CHIP_ID 在 0x00，数据从 0x01 起。 */
    MAG_CHIP_QMC5883P
} MagChip_t;

typedef struct
{
    /* 芯片坐标系 X 轴原始计数。 */
    int16_t raw_x;
    /* 芯片坐标系 Y 轴原始计数。 */
    int16_t raw_y;
    /* 芯片坐标系 Z 轴原始计数。 */
    int16_t raw_z;
    /* 换算后的 X 轴磁场，单位高斯。 */
    float x_gauss;
    /* 换算后的 Y 轴磁场，单位高斯。 */
    float y_gauss;
    /* 换算后的 Z 轴磁场，单位高斯。 */
    float z_gauss;
    /* 三轴合成模长，单位高斯。 */
    float magnitude_gauss;
    /* 累计成功样本数。 */
    uint32_t sample_count;
    /* true 表示本拍有轴饱和或状态溢出。 */
    bool overflow;
} MagRawData_t;

typedef struct
{
    I2C_HandleTypeDef *i2c;
    /* HAL 用的 8 位地址，已左移一位。 */
    uint16_t address;
    MagChip_t chip;
    /* 每高斯对应的原始计数，用于换算。 */
    float counts_per_gauss;
    /* 驱动内部累计的成功样本数。 */
    uint32_t sample_count;
    MagRawData_t data;
    bool initialized;
} MagDevice_t;

/**
 * @brief I2C2 总线扫描结果。
 *
 * 扫描 0x08～0x77，把所有应答的从机地址记进 found_addrs。
 * 主要用途是排查"新挂的设备地址撞不撞车"——例如接入激光测距之后，
 * 扫一次就能看出总线上到底多了哪个地址。
 */
typedef struct
{
    /* 扫描到的从机个数。 */
    uint8_t found_count;
    /* 应答的 7 位地址列表，最多 8 个。 */
    uint8_t found_addrs[8];
    /* 0x2C（QMC5883P）是否应答。 */
    bool qmc_p_ack;
    /* 0x2C 的寄存器 0x00，QMC5883P 应为 0x80；无应答则为 0。 */
    uint8_t qmc_p_id;
    /* 0=成功，1=总线上没有任何从机，2=有应答但配置失败，3=拿不到 I2C2 锁。 */
    uint8_t fail_code;
} MagProbe_t;

/**
 * @brief 在 I2C2 上探测 QMC5883P，并启动连续测量。
 * @param device 设备对象。
 * @param i2c 本板传入 &hi2c2。
 * @return 探测并配置成功返回 true。
 */
bool Mag_DeviceInit(MagDevice_t *device, I2C_HandleTypeDef *i2c);

/**
 * @brief 扫描 I2C2 上的 0x08～0x77，记录应答地址，并读 QMC5883P 的 CHIP_ID。
 * @param i2c 本板传入 &hi2c2。
 * @param probe 输出扫描结果。
 * @return 至少扫到一个从机返回 true。
 */
bool Mag_ScanBus(I2C_HandleTypeDef *i2c, MagProbe_t *probe);

/**
 * @brief 读取一拍三轴磁场。六个数据寄存器一次读完。
 * @param device 已初始化的设备。
 * @param data 输出原始计数和高斯值。
 * @return 通信成功且未饱和返回 true；未就绪、饱和或 I2C 失败返回 false。
 */
bool Mag_DeviceRead(MagDevice_t *device, MagRawData_t *data);

/**
 * @brief 返回已识别的芯片类型。
 */
MagChip_t Mag_DeviceChip(const MagDevice_t *device);

/**
 * @brief 返回 7 位 I2C 地址，未识别时为 0。
 */
uint8_t Mag_DeviceAddress7Bit(const MagDevice_t *device);

#ifdef __cplusplus
}
#endif

#endif
