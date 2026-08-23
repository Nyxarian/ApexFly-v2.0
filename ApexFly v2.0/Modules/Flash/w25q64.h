/**
 * @file w25q64.h
 * @brief Winbond W25Q64 外置 SPI Flash 底层驱动。
 *
 * 只负责片选、JEDEC 识别、任意地址读、按页编程、4KB 扇区擦除。
 * PID 双备份、黑匣子记录、分区表都不在本文件，由 blackbox 任务调用本接口完成。
 * SPI 和片选由调用方注入，不要在驱动里写死引脚。
 *
 * 当前飞机端硬件（与可飞老板一致）：
 * SPI3：SCK=PB3，MISO=PB4，MOSI=PB5；片选 W_CS=PB6，空闲为高。
 *
 * 写之前必须先把目标扇区擦成 0xFF；本驱动不会自动擦除。
 */

#ifndef W25Q64_H
#define W25Q64_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* W25Q64 总容量 8MB。 */
#define W25Q64_TOTAL_SIZE_BYTES 0x00800000UL
/* 页编程单位 256 字节，跨页必须拆开写。 */
#define W25Q64_PAGE_SIZE_BYTES 256U
/* 最小擦除单位 4KB。 */
#define W25Q64_SECTOR_SIZE_BYTES 4096U

typedef enum
{
    /* 操作成功。 */
    W25Q64_STATUS_OK = 0,
    /* 空指针、越界地址或长度为 0。 */
    W25Q64_STATUS_INVALID_PARAMETER,
    /* HAL SPI 收发失败，或写使能锁存未置位。 */
    W25Q64_STATUS_SPI_ERROR,
    /* 等待芯片内部编程 / 擦除超时。 */
    W25Q64_STATUS_TIMEOUT,
    /* JEDEC ID 不是 Winbond W25Q64（期望 EF xx 17）。 */
    W25Q64_STATUS_ID_MISMATCH,
    /* 预留给上层回读校验，本驱动当前不使用。 */
    W25Q64_STATUS_VERIFY_ERROR
} W25Q64_Status_t;

typedef struct
{
    /* CubeMX 生成的 SPI 句柄，飞机端为 hspi3。 */
    SPI_HandleTypeDef *spi;
    /* 片选端口，飞机端为 GPIOB。 */
    GPIO_TypeDef *chip_select_port;
    /* 片选引脚，飞机端为 W_CS_Pin（PB6）。 */
    uint16_t chip_select_pin;
    /* JEDEC 厂商 ID，Winbond 为 0xEF。 */
    uint8_t manufacturer_id;
    /* JEDEC 存储类型。 */
    uint8_t memory_type;
    /* JEDEC 容量码，W25Q64 为 0x17。 */
    uint8_t capacity_id;
    /* Init 成功后为 true，未初始化禁止读写擦。 */
    bool initialized;
} W25Q64_Device_t;

/** @brief 拉高片选后读 JEDEC ID，确认是 Winbond W25Q64（通常返回 EF 40 17）。 */
W25Q64_Status_t W25Q64_Init(W25Q64_Device_t *device, SPI_HandleTypeDef *spi, GPIO_TypeDef *chip_select_port, uint16_t chip_select_pin);

/** @brief 从任意 24 位地址连续读取，不必页对齐。address 和 length 单位均为字节。 */
W25Q64_Status_t W25Q64_Read(W25Q64_Device_t *device, uint32_t address, uint8_t *data, uint32_t length);

/**
 * @brief 连续写入任意长度数据。
 * 内部按 256 字节页边界拆分；调用前必须保证目标区域已经擦成 0xFF。
 */
W25Q64_Status_t W25Q64_Write(W25Q64_Device_t *device, uint32_t address, const uint8_t *data, uint32_t length);

/** @brief 擦除包含 address 的一个 4KB 扇区，并等待芯片内部擦除结束。 */
W25Q64_Status_t W25Q64_EraseSector(W25Q64_Device_t *device, uint32_t address);

#ifdef __cplusplus
}
#endif

#endif
