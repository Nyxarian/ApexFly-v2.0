/**
 * @file spl06.h
 * @brief SPL06-001 气压计驱动。只读 I2C 寄存器并换算出气压、温度和标准气压高度。
 *
 * 本板 SDO 接地，7 位地址 0x76，挂在 I2C2，不和 MPU6050 的 I2C1 抢总线。
 * 压力 8 Hz / OSR32，温度 8 Hz / OSR8；温度只在驱动内部参与压力补偿。
 * 不在这里做卡尔曼、油门或任务调度。
 */

#ifndef SPL06_H
#define SPL06_H

#include "stm32g4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* SDO 接地时 SPL06-001 的 7 位地址。SDO 接 3.3V 则是 0x77。 */
#define SPL06_I2C_ADDRESS 0x76U
#define SPL06_I2C_ADDRESS_ALT 0x77U

typedef struct
{
    int16_t c0;
    int16_t c1;
    int32_t c00;
    int32_t c10;
    int16_t c01;
    int16_t c11;
    int16_t c20;
    int16_t c21;
    int16_t c30;
} SPL06_Calibration_t;

typedef struct
{
    int32_t raw_pressure;
    int32_t raw_temperature;
    /* 补偿后气压，单位帕。 */
    float pressure_pa;
    /* 芯片补偿温度，单位摄氏度，仅供观察。 */
    float temperature_c;
    /* 标准大气海拔，单位米，不是离地高度。 */
    float altitude_m;
    uint32_t sample_count;
} SPL06_Data_t;

typedef struct
{
    I2C_HandleTypeDef *i2c;
    uint16_t address;
    uint8_t product_id;
    float pressure_scale;
    float temperature_scale;
    SPL06_Calibration_t calibration;
    SPL06_Data_t data;
    bool initialized;
} SPL06_Device_t;

/**
 * @brief 复位芯片、读校准系数并启动连续压力和内部温度补偿测量。
 * @param device SPL06 设备对象。
 * @param i2c 本板传入 &hi2c2。
 * @return 成功返回 true。
 */
bool SPL06_Init(SPL06_Device_t *device, I2C_HandleTypeDef *i2c);
bool SPL06_InitAt(SPL06_Device_t *device, I2C_HandleTypeDef *i2c, uint8_t address_7bit);

/**
 * @brief 读取一组新样本并换算气压、温度和标准气压高度。
 * @param device 已初始化的设备。
 * @param data 输出。
 * @return 本拍数据有效返回 true；尚未转换完成返回 false。
 */
bool SPL06_ReadData(SPL06_Device_t *device, SPL06_Data_t *data);

#ifdef __cplusplus
}
#endif

#endif
