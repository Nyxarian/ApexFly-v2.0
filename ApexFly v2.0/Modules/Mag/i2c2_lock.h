/**
 * @file i2c2_lock.h
 * @brief I2C2 总线互斥。气压计、磁力计和后续激光共用 PA8/PA9，HAL I2C 不能并发。
 *
 * 所有 hi2c2 的读写都必须先 I2C2_Lock、结束后 I2C2_Unlock。
 * 本文件不配置引脚，也不识别从机地址。
 */

#ifndef I2C2_LOCK_H
#define I2C2_LOCK_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 创建 I2C2 互斥量。可重复调用，只创建一次。
 */
void I2C2_Lock_Init(void);

/**
 * @brief 占用 I2C2，直到超时。
 * @param timeout_ms 最长等待时间，单位毫秒。
 * @return 拿到锁返回 true。
 */
bool I2C2_Lock(uint32_t timeout_ms);

/**
 * @brief 释放 I2C2。
 */
void I2C2_Unlock(void);

/**
 * @brief 总线挂死时恢复 I2C2。先补时钟把从机卡死的传输踢完，再重新初始化外设。
 */
void I2C2_BusRecover(void);

#ifdef __cplusplus
}
#endif

#endif
