/**
 * @file w25q64.c
 * @brief W25Q64 SPI Flash 底层实现。
 *
 * 命令、时序和页拆分与老飞机端 Device/w25q64.c 保持一致，便于沿用已验证的芯片行为。
 * 本文件不保存 PID、也不写黑匣子分区；上层以后再封装双备份和记录队列。
 */

#include "w25q64.h"

#include "FreeRTOS.h"
#include "task.h"
#include <stddef.h>
#include <string.h>

#define W25Q64_COMMAND_WRITE_ENABLE 0x06U
#define W25Q64_COMMAND_READ_STATUS_1 0x05U
#define W25Q64_COMMAND_READ_DATA 0x03U
#define W25Q64_COMMAND_PAGE_PROGRAM 0x02U
#define W25Q64_COMMAND_SECTOR_ERASE 0x20U
#define W25Q64_COMMAND_JEDEC_ID 0x9FU

#define W25Q64_STATUS_BUSY 0x01U
#define W25Q64_STATUS_WRITE_ENABLE_LATCH 0x02U
#define W25Q64_SPI_TIMEOUT_MS 100U
#define W25Q64_PROGRAM_TIMEOUT_MS 20U
#define W25Q64_ERASE_TIMEOUT_MS 1000U

static void W25Q64_Select(W25Q64_Device_t *device)
{
    HAL_GPIO_WritePin(device->chip_select_port, device->chip_select_pin, GPIO_PIN_RESET);
}

static void W25Q64_Deselect(W25Q64_Device_t *device)
{
    HAL_GPIO_WritePin(device->chip_select_port, device->chip_select_pin, GPIO_PIN_SET);
}

static W25Q64_Status_t W25Q64_Transmit(W25Q64_Device_t *device, const uint8_t *data, uint16_t length)
{
    if (HAL_SPI_Transmit(device->spi, (uint8_t *)data, length, W25Q64_SPI_TIMEOUT_MS) != HAL_OK)
        return W25Q64_STATUS_SPI_ERROR;
    return W25Q64_STATUS_OK;
}

static W25Q64_Status_t W25Q64_Receive(W25Q64_Device_t *device, uint8_t *data, uint16_t length)
{
    if (HAL_SPI_Receive(device->spi, data, length, W25Q64_SPI_TIMEOUT_MS) != HAL_OK)
        return W25Q64_STATUS_SPI_ERROR;
    return W25Q64_STATUS_OK;
}

static W25Q64_Status_t W25Q64_ReadStatus1(W25Q64_Device_t *device, uint8_t *status)
{
    uint8_t command = W25Q64_COMMAND_READ_STATUS_1;
    W25Q64_Status_t result;

    W25Q64_Select(device);
    result = W25Q64_Transmit(device, &command, 1U);
    if (result == W25Q64_STATUS_OK) result = W25Q64_Receive(device, status, 1U);
    W25Q64_Deselect(device);
    return result;
}

static W25Q64_Status_t W25Q64_WaitReady(W25Q64_Device_t *device, uint32_t timeout_ms)
{
    uint8_t status;
    uint32_t start_tick = HAL_GetTick();
    W25Q64_Status_t result;

    do
    {
        result = W25Q64_ReadStatus1(device, &status);
        if (result != W25Q64_STATUS_OK) return result;
        if ((status & W25Q64_STATUS_BUSY) == 0U) return W25Q64_STATUS_OK;
        /* 调度器起来后让出 CPU，避免擦扇区时卡住电机和电台。 */
        if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) vTaskDelay(1);
        else HAL_Delay(1U);
    } while ((HAL_GetTick() - start_tick) < timeout_ms);
    return W25Q64_STATUS_TIMEOUT;
}

static W25Q64_Status_t W25Q64_WriteEnable(W25Q64_Device_t *device)
{
    uint8_t command = W25Q64_COMMAND_WRITE_ENABLE;
    uint8_t status;
    W25Q64_Status_t result;

    W25Q64_Select(device);
    result = W25Q64_Transmit(device, &command, 1U);
    W25Q64_Deselect(device);
    if (result != W25Q64_STATUS_OK) return result;
    result = W25Q64_ReadStatus1(device, &status);
    if (result != W25Q64_STATUS_OK) return result;
    return ((status & W25Q64_STATUS_WRITE_ENABLE_LATCH) != 0U) ? W25Q64_STATUS_OK : W25Q64_STATUS_SPI_ERROR;
}

static void W25Q64_BuildAddressCommand(uint8_t command[4], uint8_t opcode, uint32_t address)
{
    command[0] = opcode;
    command[1] = (uint8_t)(address >> 16);
    command[2] = (uint8_t)(address >> 8);
    command[3] = (uint8_t)address;
}

W25Q64_Status_t W25Q64_Init(W25Q64_Device_t *device, SPI_HandleTypeDef *spi, GPIO_TypeDef *chip_select_port, uint16_t chip_select_pin)
{
    uint8_t command = W25Q64_COMMAND_JEDEC_ID;
    uint8_t identifier[3];
    W25Q64_Status_t result;

    if ((device == NULL) || (spi == NULL) || (chip_select_port == NULL))
        return W25Q64_STATUS_INVALID_PARAMETER;

    memset(device, 0, sizeof(*device));
    device->spi = spi;
    device->chip_select_port = chip_select_port;
    device->chip_select_pin = chip_select_pin;
    W25Q64_Deselect(device);
    HAL_Delay(1U);

    W25Q64_Select(device);
    result = W25Q64_Transmit(device, &command, 1U);
    if (result == W25Q64_STATUS_OK) result = W25Q64_Receive(device, identifier, sizeof(identifier));
    W25Q64_Deselect(device);
    if (result != W25Q64_STATUS_OK) return result;

    device->manufacturer_id = identifier[0];
    device->memory_type = identifier[1];
    device->capacity_id = identifier[2];
    /* Winbond 厂商码 0xEF，容量码 0x17 对应 64Mbit。 */
    if ((identifier[0] != 0xEFU) || (identifier[2] != 0x17U)) return W25Q64_STATUS_ID_MISMATCH;
    device->initialized = true;
    return W25Q64_STATUS_OK;
}

W25Q64_Status_t W25Q64_Read(W25Q64_Device_t *device, uint32_t address, uint8_t *data, uint32_t length)
{
    uint8_t command[4];
    uint16_t chunk;
    W25Q64_Status_t result;

    if ((device == NULL) || !device->initialized || (data == NULL) || (length == 0U) ||
        (address >= W25Q64_TOTAL_SIZE_BYTES) || (length > (W25Q64_TOTAL_SIZE_BYTES - address)))
        return W25Q64_STATUS_INVALID_PARAMETER;

    result = W25Q64_WaitReady(device, W25Q64_PROGRAM_TIMEOUT_MS);
    if (result != W25Q64_STATUS_OK) return result;
    W25Q64_BuildAddressCommand(command, W25Q64_COMMAND_READ_DATA, address);
    W25Q64_Select(device);
    result = W25Q64_Transmit(device, command, sizeof(command));
    while ((result == W25Q64_STATUS_OK) && (length > 0U))
    {
        chunk = (length > 65535UL) ? 65535U : (uint16_t)length;
        result = W25Q64_Receive(device, data, chunk);
        data += chunk;
        length -= chunk;
    }
    W25Q64_Deselect(device);
    return result;
}

static W25Q64_Status_t W25Q64_ProgramPage(W25Q64_Device_t *device, uint32_t address, const uint8_t *data, uint16_t length)
{
    uint8_t command[4];
    W25Q64_Status_t result;

    result = W25Q64_WriteEnable(device);
    if (result != W25Q64_STATUS_OK) return result;
    W25Q64_BuildAddressCommand(command, W25Q64_COMMAND_PAGE_PROGRAM, address);
    W25Q64_Select(device);
    result = W25Q64_Transmit(device, command, sizeof(command));
    if (result == W25Q64_STATUS_OK) result = W25Q64_Transmit(device, data, length);
    W25Q64_Deselect(device);
    if (result != W25Q64_STATUS_OK) return result;
    return W25Q64_WaitReady(device, W25Q64_PROGRAM_TIMEOUT_MS);
}

W25Q64_Status_t W25Q64_Write(W25Q64_Device_t *device, uint32_t address, const uint8_t *data, uint32_t length)
{
    uint16_t page_remaining;
    uint16_t chunk;
    W25Q64_Status_t result;

    if ((device == NULL) || !device->initialized || (data == NULL) || (length == 0U) ||
        (address >= W25Q64_TOTAL_SIZE_BYTES) || (length > (W25Q64_TOTAL_SIZE_BYTES - address)))
        return W25Q64_STATUS_INVALID_PARAMETER;

    while (length > 0U)
    {
        page_remaining = (uint16_t)(W25Q64_PAGE_SIZE_BYTES - (address % W25Q64_PAGE_SIZE_BYTES));
        chunk = (length < page_remaining) ? (uint16_t)length : page_remaining;
        result = W25Q64_ProgramPage(device, address, data, chunk);
        if (result != W25Q64_STATUS_OK) return result;
        address += chunk;
        data += chunk;
        length -= chunk;
    }
    return W25Q64_STATUS_OK;
}

W25Q64_Status_t W25Q64_EraseSector(W25Q64_Device_t *device, uint32_t address)
{
    uint8_t command[4];
    W25Q64_Status_t result;

    if ((device == NULL) || !device->initialized || (address >= W25Q64_TOTAL_SIZE_BYTES))
        return W25Q64_STATUS_INVALID_PARAMETER;

    address &= ~(W25Q64_SECTOR_SIZE_BYTES - 1UL);
    result = W25Q64_WriteEnable(device);
    if (result != W25Q64_STATUS_OK) return result;
    W25Q64_BuildAddressCommand(command, W25Q64_COMMAND_SECTOR_ERASE, address);
    W25Q64_Select(device);
    result = W25Q64_Transmit(device, command, sizeof(command));
    W25Q64_Deselect(device);
    if (result != W25Q64_STATUS_OK) return result;
    return W25Q64_WaitReady(device, W25Q64_ERASE_TIMEOUT_MS);
}
