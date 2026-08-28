/**
 * @file hmc5883l.h
 * @brief GY-273 磁力计底层。只负责 I2C 探测、初始化和读三轴原始磁场。
 *
 * 市售 GY-273 可能是 HMC5883L（0x1E）、QMC5883L（0x0D）或 QMC5883P/QMC6308（0x2C），上电先判型。
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

/* HMC5883L 的 7 位地址。 */
#define HMC5883L_I2C_ADDRESS 0x1EU
/* QMC5883L 的 7 位地址，老款 GY-273 常见。 */
#define QMC5883L_I2C_ADDRESS 0x0DU
/* QMC5883P / QMC6308 的 7 位地址，新款 GY-273 常见。 */
#define QMC5883P_I2C_ADDRESS 0x2CU
/* HMC 任一轴读到该值表示本拍饱和，必须丢弃。 */
#define HMC5883L_OVERFLOW_COUNTS (-4096)

typedef enum
{
    /* 还没探测到芯片。 */
    MAG_CHIP_NONE = 0,
    /* 手册中的 HMC5883L。 */
    MAG_CHIP_HMC5883L,
    /* 实际焊盘是 QMC5883L。 */
    MAG_CHIP_QMC5883L,
    /* 新款 GY-273：CHIP_ID 在 0x00，数据从 0x01 起。 */
    MAG_CHIP_QMC5883P,
    /* 同地址 0x2C，但寄存器布局接近 QMC5883L。 */
    MAG_CHIP_QMC6308
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

typedef struct
{
    /* 扫描到的从机个数，最多 6 个。 */
    uint8_t found_count;
    /* 7 位地址列表。 */
    uint8_t found_addrs[6];
    /* 0x1E 是否应答。 */
    bool hmc_ack;
    /* 0x0D 是否应答。 */
    bool qmc_ack;
    /* 0x2C 是否应答，新款 GY-273。 */
    bool qmc_p_ack;
    /* 0x76 气压计是否还在总线上。 */
    bool baro_ack;
    /* HMC 识别寄存器 0x0A～0x0C，无应答则为 0。 */
    uint8_t hmc_id[3];
    /* QMC5883L CHIP_ID，无应答则为 0。 */
    uint8_t qmc_id;
    /* 0x2C 的寄存器 0x00，QMC5883P 应为 0x80。 */
    uint8_t qmc_p_id;
    /* 0x2C 的寄存器 0x0D，QMC6308 应为 0x80。 */
    uint8_t qmc_p_id_d;
    /* 0=成功，1=总线上没有任何从机，2=有应答但配置失败，3=拿不到 I2C2 锁，4=已做总线恢复。 */
    uint8_t fail_code;
} MagProbe_t;

/**
 * @brief 在 I2C2 上探测 HMC 或 QMC，并启动连续测量。
 * @param device 设备对象。
 * @param i2c 本板传入 &hi2c2。
 * @return 探测并配置成功返回 true。
 */
bool Mag_DeviceInit(MagDevice_t *device, I2C_HandleTypeDef *i2c);

/**
 * @brief 扫描 I2C2 上 0x08～0x77，并读 HMC/QMC 识别字节。
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
