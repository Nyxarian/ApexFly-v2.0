/**
 * @file vl53l1x.h
 * @brief VL53L1X 飞行时间（ToF）激光测距底层驱动接口。
 *
 * 只负责 I2C 读写、初始化、单次测距触发与结果读取。不认识飞控，也不做定高。
 * 上层由 Modules/ToF/tof.c 的任务定时调用。
 *
 * ── 为什么用"单次测量"而不是"连续测量" ──────────────────────────
 * 2026-09-11 实测：本芯片的连续模式（MODE_START=0x40）能写进去、寄存器也读回 0x40，
 * 但不再产生新测量结果。而单次模式（MODE_START=0x10）每次都能正常完成：
 *   mode 从 0x10 自动回到 0x00，测距状态读到 0x09（测距完成），距离正确。
 * 因此本驱动改用"单次触发 + 轮询完成"的方式工作，由上层按固定节奏反复触发。
 *
 * ── 硬件事实 ────────────────────────────────────────────────
 *   I2C 地址 0x29（出厂默认），与磁力计 QMC5883P 的 0x2C 不冲突，可并挂 I2C2。
 *   XSHUT 低电平复位；本模块自带约 187k 上拉，悬空即为正常工作。
 *   工作电压 2.6~3.5V，直接接 3.3V。
 *
 * 走线约定：本驱动的每一次 I2C 访问都必须先拿 I2C2 锁，和 hmc5883l.c 一致。
 */

#ifndef VL53L1X_H
#define VL53L1X_H

#include "stm32g4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 出厂默认的 7 位 I2C 地址。 */
#define VL53L1X_I2C_ADDRESS 0x29U

/**
 * @brief 初始化过程中的分步状态，用于定位"卡在哪一步"。
 */
typedef enum
{
    VL53L1X_INIT_NOT_STARTED = 0,
    VL53L1X_INIT_OK,
    /* 芯片无应答（接线、供电或 XSHUT 没被拉高）。 */
    VL53L1X_INIT_NO_ACK,
    /* 型号 ID 不是 0xEACC，说明不是 VL53L1X。 */
    VL53L1X_INIT_BAD_MODEL_ID,
    /* 写配置失败。 */
    VL53L1X_INIT_CONFIG_WRITE_FAIL,
    /* 等启动完成失败。 */
    VL53L1X_INIT_CONFIG_VERIFY_FAIL,
    /* 启动一次测量失败。 */
    VL53L1X_INIT_START_FAIL
} VL53L1X_InitStatus_t;

/**
 * @brief 一次测距的结果。
 */
typedef struct
{
    /* 距离，单位毫米。 */
    uint16_t distance_mm;
    /* 映射后的测距状态码，0 表示本次结果有效。 */
    uint8_t range_status;
    /* true 表示本次拿到的是有效新数据。 */
    bool valid;
    /* 累计通信错误次数。 */
    uint32_t error_count;
} VL53L1X_Result_t;

typedef struct
{
    I2C_HandleTypeDef *i2c;
    uint16_t address;
    bool initialized;
    VL53L1X_InitStatus_t last_init_status;
    uint32_t error_count;
    /* 上一次手动校准保存下来的 VHV 配置，用于恢复。 */
    uint8_t saved_vhv_init;
    uint8_t saved_vhv_timeout;
    bool calibrated;
} VL53L1X_Device_t;

/**
 * @brief 初始化芯片并配置为短距档。初始化末尾会自动触发第一次测量。
 * @param device 设备对象。
 * @param i2c 本板传入 &hi2c2。
 * @return 成功返回 true；失败返回 false，原因见 VL53L1X_GetLastInitStatus()。
 */
bool VL53L1X_Init(VL53L1X_Device_t *device, I2C_HandleTypeDef *i2c);

/**
 * @brief 读取上一次初始化的分步状态，用于排查失败原因。
 */
VL53L1X_InitStatus_t VL53L1X_GetLastInitStatus(const VL53L1X_Device_t *device);

/**
 * @brief 把状态码转成可读字符串，供串口打印。
 */
const char *VL53L1X_InitStatusString(VL53L1X_InitStatus_t status);

/**
 * @brief 读芯片型号 ID，用于确认通信基本正常。0 表示读失败。
 */
uint16_t VL53L1X_ReadModelId(VL53L1X_Device_t *device);

/**
 * @brief 触发一次单次测距。上层按固定节奏调用。
 * @return 命令发出成功返回 true。
 */
bool VL53L1X_StartSingleShot(VL53L1X_Device_t *device);

/**
 * @brief 查询这次测量是否已完成。
 * @return 已完成返回 true。
 */
bool VL53L1X_IsMeasurementDone(VL53L1X_Device_t *device);

/**
 * @brief 读取已完成的那次测量结果，并做好首次手动校准。
 * @return 读成功返回 true（是否有效看 result->valid）。
 */
bool VL53L1X_ReadMeasurement(VL53L1X_Device_t *device, VL53L1X_Result_t *result);

#ifdef __cplusplus
}
#endif

#endif
