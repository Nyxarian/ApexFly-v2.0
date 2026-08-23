/**
 * @file ra01sc.c
 * @brief RA-01SC（LLCC68）LoRa 无线模块底层驱动实现。
 *
 * 命令码依据 Semtech LLCC68 接口。单次 SPI 命令用阻塞 HAL，
 * 发送完成和收到数据包由 DIO1 中断通知上层。
 * 本文件从老工程 Device/ra01sc 迁入，协议层尚未接入。
 */

#include "ra01sc.h" /* 引入"ra01sc.h"提供的类型、宏或函数声明。 */
#include <stddef.h> /* 引入<stddef.h>提供的类型、宏或函数声明。 */
#include <string.h> /* 引入<string.h>提供的类型、宏或函数声明。 */

/* 单次HAL SPI调用允许等待的最长时间。 */
#define RA01SC_SPI_TIMEOUT_MS 100U /* 定义宏RA01SC_SPI_TIMEOUT_MS，集中保存本驱动使用的固定参数。 */

/* 等待BUSY引脚回到低电平的最长时间。 */
#define RA01SC_BUSY_TIMEOUT_MS 200U /* 定义宏RA01SC_BUSY_TIMEOUT_MS，集中保存本驱动使用的固定参数。 */

/* LLCC68内部晶振频率，用于把Hz转换为PLL频率字。 */
#define RA01SC_XTAL_FREQUENCY_HZ 32000000UL /* 定义宏RA01SC_XTAL_FREQUENCY_HZ，集中保存本驱动使用的固定参数。 */

/* LLCC68连续接收模式的特殊超时参数。 */
#define RA01SC_RX_CONTINUOUS_TIMEOUT 0x00FFFFFFUL /* 定义宏RA01SC_RX_CONTINUOUS_TIMEOUT，集中保存本驱动使用的固定参数。 */

/* SPI读取操作使用的空字节。 */
#define RA01SC_NOP 0x00U /* 定义宏RA01SC_NOP，集中保存本驱动使用的固定参数。 */

/* LLCC68命令：进入待机模式。 */
#define RA01SC_CMD_SET_STANDBY 0x80U /* 定义宏RA01SC_CMD_SET_STANDBY，集中保存本驱动使用的固定参数。 */

/* LLCC68命令：启动发送。 */
#define RA01SC_CMD_SET_TX 0x83U /* 定义宏RA01SC_CMD_SET_TX，集中保存本驱动使用的固定参数。 */

/* LLCC68命令：启动接收。 */
#define RA01SC_CMD_SET_RX 0x82U /* 定义宏RA01SC_CMD_SET_RX，集中保存本驱动使用的固定参数。 */

/* LLCC68命令：配置内部稳压器工作方式。 */
#define RA01SC_CMD_SET_REGULATOR_MODE 0x96U /* 定义宏RA01SC_CMD_SET_REGULATOR_MODE，集中保存本驱动使用的固定参数。 */

/* LLCC68命令：执行内部模块校准。 */
#define RA01SC_CMD_CALIBRATE 0x89U /* 定义宏RA01SC_CMD_CALIBRATE，集中保存本驱动使用的固定参数。 */

/* LLCC68命令：按频段执行镜像抑制校准。 */
#define RA01SC_CMD_CALIBRATE_IMAGE 0x98U /* 定义宏RA01SC_CMD_CALIBRATE_IMAGE，集中保存本驱动使用的固定参数。 */

/* LLCC68命令：配置功率放大器。 */
#define RA01SC_CMD_SET_PA_CONFIG 0x95U /* 定义宏RA01SC_CMD_SET_PA_CONFIG，集中保存本驱动使用的固定参数。 */

/* LLCC68命令：配置发送或接收结束后的回退模式。 */
#define RA01SC_CMD_SET_RX_TX_FALLBACK_MODE 0x93U /* 定义宏RA01SC_CMD_SET_RX_TX_FALLBACK_MODE，集中保存本驱动使用的固定参数。 */

/* LLCC68命令：写寄存器。 */
#define RA01SC_CMD_WRITE_REGISTER 0x0DU /* 定义宏RA01SC_CMD_WRITE_REGISTER，集中保存本驱动使用的固定参数。 */

/* LLCC68命令：读寄存器。 */
#define RA01SC_CMD_READ_REGISTER 0x1DU /* 定义宏RA01SC_CMD_READ_REGISTER，集中保存本驱动使用的固定参数。 */

/* LLCC68命令：写内部数据缓冲区。 */
#define RA01SC_CMD_WRITE_BUFFER 0x0EU /* 定义宏RA01SC_CMD_WRITE_BUFFER，集中保存本驱动使用的固定参数。 */

/* LLCC68命令：读内部数据缓冲区。 */
#define RA01SC_CMD_READ_BUFFER 0x1EU /* 定义宏RA01SC_CMD_READ_BUFFER，集中保存本驱动使用的固定参数。 */

/* LLCC68命令：配置系统中断和DIO1映射。 */
#define RA01SC_CMD_SET_DIO_IRQ_PARAMS 0x08U /* 定义宏RA01SC_CMD_SET_DIO_IRQ_PARAMS，集中保存本驱动使用的固定参数。 */

/* LLCC68命令：读取中断状态。 */
#define RA01SC_CMD_GET_IRQ_STATUS 0x12U /* 定义宏RA01SC_CMD_GET_IRQ_STATUS，集中保存本驱动使用的固定参数。 */

/* LLCC68命令：清除中断状态。 */
#define RA01SC_CMD_CLEAR_IRQ_STATUS 0x02U /* 定义宏RA01SC_CMD_CLEAR_IRQ_STATUS，集中保存本驱动使用的固定参数。 */

/* LLCC68命令：配置射频中心频率。 */
#define RA01SC_CMD_SET_RF_FREQUENCY 0x86U /* 定义宏RA01SC_CMD_SET_RF_FREQUENCY，集中保存本驱动使用的固定参数。 */

/* LLCC68命令：选择LoRa或GFSK包类型。 */
#define RA01SC_CMD_SET_PACKET_TYPE 0x8AU /* 定义宏RA01SC_CMD_SET_PACKET_TYPE，集中保存本驱动使用的固定参数。 */

/* LLCC68命令：配置发射功率和功率爬升时间。 */
#define RA01SC_CMD_SET_TX_PARAMS 0x8EU /* 定义宏RA01SC_CMD_SET_TX_PARAMS，集中保存本驱动使用的固定参数。 */

/* LLCC68命令：配置扩频因子、带宽和编码率。 */
#define RA01SC_CMD_SET_MODULATION_PARAMS 0x8BU /* 定义宏RA01SC_CMD_SET_MODULATION_PARAMS，集中保存本驱动使用的固定参数。 */

/* LLCC68命令：配置前导码、包头、负载长度、CRC和IQ。 */
#define RA01SC_CMD_SET_PACKET_PARAMS 0x8CU /* 定义宏RA01SC_CMD_SET_PACKET_PARAMS，集中保存本驱动使用的固定参数。 */

/* LLCC68命令：配置发送和接收缓冲区起始地址。 */
#define RA01SC_CMD_SET_BUFFER_BASE_ADDRESS 0x8FU /* 定义宏RA01SC_CMD_SET_BUFFER_BASE_ADDRESS，集中保存本驱动使用的固定参数。 */

/* LLCC68命令：读取芯片状态。 */
#define RA01SC_CMD_GET_STATUS 0xC0U /* 定义宏RA01SC_CMD_GET_STATUS，集中保存本驱动使用的固定参数。 */

/* LLCC68命令：读取接收数据长度和缓冲区起始地址。 */
#define RA01SC_CMD_GET_RX_BUFFER_STATUS 0x13U /* 定义宏RA01SC_CMD_GET_RX_BUFFER_STATUS，集中保存本驱动使用的固定参数。 */

/* LLCC68命令：读取最近一次LoRa数据包状态。 */
#define RA01SC_CMD_GET_PACKET_STATUS 0x14U /* 定义宏RA01SC_CMD_GET_PACKET_STATUS，集中保存本驱动使用的固定参数。 */

/* LLCC68包类型：LoRa。 */
#define RA01SC_PACKET_TYPE_LORA 0x01U /* 定义宏RA01SC_PACKET_TYPE_LORA，集中保存本驱动使用的固定参数。 */

/* 待机时使用内部RC时钟。 */
#define RA01SC_STANDBY_RC 0x00U /* 定义宏RA01SC_STANDBY_RC，集中保存本驱动使用的固定参数。 */

/* 使用模块内部DC-DC和LDO组合模式。 */
#define RA01SC_REGULATOR_DCDC 0x01U /* 定义宏RA01SC_REGULATOR_DCDC，集中保存本驱动使用的固定参数。 */

/* 发送和接收结束后回到STDBY_RC。 */
#define RA01SC_FALLBACK_STANDBY_RC 0x20U /* 定义宏RA01SC_FALLBACK_STANDBY_RC，集中保存本驱动使用的固定参数。 */

/* 200us功率爬升时间编码。 */
#define RA01SC_RAMP_TIME_200_US 0x04U /* 定义宏RA01SC_RAMP_TIME_200_US，集中保存本驱动使用的固定参数。 */

/* LoRa同步字寄存器地址。 */
#define RA01SC_REG_LORA_SYNC_WORD 0x0740U /* 定义宏RA01SC_REG_LORA_SYNC_WORD，集中保存本驱动使用的固定参数。 */

/* 过流保护寄存器地址。 */
#define RA01SC_REG_OCP 0x08E7U /* 定义宏RA01SC_REG_OCP，集中保存本驱动使用的固定参数。 */

/* 发射钳位修正寄存器地址。 */
#define RA01SC_REG_TX_CLAMP_CONFIG 0x08D8U /* 定义宏RA01SC_REG_TX_CLAMP_CONFIG，集中保存本驱动使用的固定参数。 */

/* 发射钳位修正位掩码。 */
#define RA01SC_TX_CLAMP_CONFIG_MASK 0x1EU /* 定义宏RA01SC_TX_CLAMP_CONFIG_MASK，集中保存本驱动使用的固定参数。 */

/* +17dBm功放配置下使用140mA过流保护值。 */
#define RA01SC_OCP_140_MA 0x38U /* 定义宏RA01SC_OCP_140_MA，集中保存本驱动使用的固定参数。 */

/* 本驱动允许RA-01SC工作的最低频率。 */
#define RA01SC_MIN_FREQUENCY_HZ 410000000UL /* 定义宏RA01SC_MIN_FREQUENCY_HZ，集中保存本驱动使用的固定参数。 */

/* 本驱动允许RA-01SC工作的最高频率。 */
#define RA01SC_MAX_FREQUENCY_HZ 525000000UL /* 定义宏RA01SC_MAX_FREQUENCY_HZ，集中保存本驱动使用的固定参数。 */

/**
 * @brief 检查设备对象中是否已经保存完整硬件连接信息。
 */
static bool RA01SC_IsHardwareValid(const RA01SC_Device_t *device) /* 定义函数RA01SC_IsHardwareValid，实现该函数对应的底层操作。 */
{ /* 开始当前函数、判断、循环或数据类型的代码块。 */
    if (device == NULL) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return false; /* 把false作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    if (device->hardware.spi == NULL) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return false; /* 把false作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    if ((device->hardware.nss_port == NULL) || (device->hardware.reset_port == NULL) || (device->hardware.busy_port == NULL)) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return false; /* 把false作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    if ((device->hardware.dio1_port == NULL) || (device->hardware.txen_port == NULL) || (device->hardware.rxen_port == NULL)) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return false; /* 把false作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    if ((device->hardware.nss_pin == 0U) || (device->hardware.reset_pin == 0U) || (device->hardware.busy_pin == 0U)) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return false; /* 把false作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    if ((device->hardware.dio1_pin == 0U) || (device->hardware.txen_pin == 0U) || (device->hardware.rxen_pin == 0U)) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return false; /* 把false作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    return true; /* 把true作为当前函数结果返回给调用者。 */
} /* 结束当前函数、判断、循环或数据类型的代码块。 */

/**
 * @brief 检查LoRa配置是否在RA-01SC和LLCC68允许范围内。
 */
static bool RA01SC_IsConfigValid(const RA01SC_Config_t *config) /* 定义函数RA01SC_IsConfigValid，实现该函数对应的底层操作。 */
{ /* 开始当前函数、判断、循环或数据类型的代码块。 */
    if (config == NULL) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return false; /* 把false作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    if ((config->frequency_hz < RA01SC_MIN_FREQUENCY_HZ) || (config->frequency_hz > RA01SC_MAX_FREQUENCY_HZ)) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return false; /* 把false作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    if ((config->spreading_factor < RA01SC_LORA_SF5) || (config->spreading_factor > RA01SC_LORA_SF11)) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return false; /* 把false作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    if ((config->bandwidth != RA01SC_LORA_BW_125_KHZ) && (config->bandwidth != RA01SC_LORA_BW_250_KHZ) && (config->bandwidth != RA01SC_LORA_BW_500_KHZ)) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return false; /* 把false作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    if ((config->coding_rate < RA01SC_LORA_CR_4_5) || (config->coding_rate > RA01SC_LORA_CR_4_8)) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return false; /* 把false作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    if ((config->header_type != RA01SC_LORA_HEADER_EXPLICIT) && (config->header_type != RA01SC_LORA_HEADER_IMPLICIT)) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return false; /* 把false作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    if ((config->preamble_length == 0U) || (config->max_payload_length == 0U)) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return false; /* 把false作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    if ((config->tx_power_dbm < -9) || (config->tx_power_dbm > 17)) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return false; /* 把false作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    if ((config->sync_word != RA01SC_LORA_PUBLIC_SYNC_WORD) && (config->sync_word != RA01SC_LORA_PRIVATE_SYNC_WORD)) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return false; /* 把false作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    if ((config->tx_timeout_ms == 0U) || (config->tx_timeout_ms > 262143U)) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return false; /* 把false作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    return true; /* 把true作为当前函数结果返回给调用者。 */
} /* 结束当前函数、判断、循环或数据类型的代码块。 */

/**
 * @brief 控制NSS引脚电平。
 */
static void RA01SC_SetNss(const RA01SC_Device_t *device, GPIO_PinState state) /* 定义函数RA01SC_SetNss，实现该函数对应的底层操作。 */
{ /* 开始当前函数、判断、循环或数据类型的代码块。 */
    HAL_GPIO_WritePin(device->hardware.nss_port, device->hardware.nss_pin, state); /* 调用HAL_GPIO_WritePin函数执行本行对应操作。 */
} /* 结束当前函数、判断、循环或数据类型的代码块。 */

/**
 * @brief 关闭外部射频开关的发送和接收通道。
 */
static void RA01SC_SetRfSwitchStandby(const RA01SC_Device_t *device) /* 定义函数RA01SC_SetRfSwitchStandby，实现该函数对应的底层操作。 */
{ /* 开始当前函数、判断、循环或数据类型的代码块。 */
    HAL_GPIO_WritePin(device->hardware.txen_port, device->hardware.txen_pin, GPIO_PIN_RESET); /* 调用HAL_GPIO_WritePin函数执行本行对应操作。 */
    HAL_GPIO_WritePin(device->hardware.rxen_port, device->hardware.rxen_pin, GPIO_PIN_RESET); /* 调用HAL_GPIO_WritePin函数执行本行对应操作。 */
} /* 结束当前函数、判断、循环或数据类型的代码块。 */

/**
 * @brief 将外部射频开关切换到发送通道。
 */
static void RA01SC_SetRfSwitchTransmit(const RA01SC_Device_t *device) /* 定义函数RA01SC_SetRfSwitchTransmit，实现该函数对应的底层操作。 */
{ /* 开始当前函数、判断、循环或数据类型的代码块。 */
    HAL_GPIO_WritePin(device->hardware.rxen_port, device->hardware.rxen_pin, GPIO_PIN_RESET); /* 调用HAL_GPIO_WritePin函数执行本行对应操作。 */
    HAL_GPIO_WritePin(device->hardware.txen_port, device->hardware.txen_pin, GPIO_PIN_SET); /* 调用HAL_GPIO_WritePin函数执行本行对应操作。 */
} /* 结束当前函数、判断、循环或数据类型的代码块。 */

/**
 * @brief 将外部射频开关切换到接收通道。
 */
static void RA01SC_SetRfSwitchReceive(const RA01SC_Device_t *device) /* 定义函数RA01SC_SetRfSwitchReceive，实现该函数对应的底层操作。 */
{ /* 开始当前函数、判断、循环或数据类型的代码块。 */
    HAL_GPIO_WritePin(device->hardware.txen_port, device->hardware.txen_pin, GPIO_PIN_RESET); /* 调用HAL_GPIO_WritePin函数执行本行对应操作。 */
    HAL_GPIO_WritePin(device->hardware.rxen_port, device->hardware.rxen_pin, GPIO_PIN_SET); /* 调用HAL_GPIO_WritePin函数执行本行对应操作。 */
} /* 结束当前函数、判断、循环或数据类型的代码块。 */

/**
 * @brief 等待BUSY引脚变为低电平。
 */
static RA01SC_Status_t RA01SC_WaitWhileBusy(const RA01SC_Device_t *device, uint32_t timeout_ms) /* 定义函数RA01SC_WaitWhileBusy，实现该函数对应的底层操作。 */
{ /* 开始当前函数、判断、循环或数据类型的代码块。 */
    uint32_t start_tick; /* 声明变量start_tick，用于保存当前函数或模块运行数据。 */

    if (RA01SC_IsHardwareValid(device) == false) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return RA01SC_STATUS_INVALID_PARAM; /* 把RA01SC_STATUS_INVALID_PARAM作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    start_tick = HAL_GetTick(); /* 调用HAL_GetTick函数，并把返回结果保存到start_tick。 */

    while (HAL_GPIO_ReadPin(device->hardware.busy_port, device->hardware.busy_pin) == GPIO_PIN_SET) /* 在本行条件持续成立时重复执行循环体。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        if ((HAL_GetTick() - start_tick) >= timeout_ms) /* 判断本行括号中的条件是否成立。 */
        { /* 开始当前函数、判断、循环或数据类型的代码块。 */
            return RA01SC_STATUS_BUSY_TIMEOUT; /* 把RA01SC_STATUS_BUSY_TIMEOUT作为当前函数结果返回给调用者。 */
        } /* 结束当前函数、判断、循环或数据类型的代码块。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    return RA01SC_STATUS_OK; /* 把RA01SC_STATUS_OK作为当前函数结果返回给调用者。 */
} /* 结束当前函数、判断、循环或数据类型的代码块。 */

/**
 * @brief 在SPI主机模式下发送若干字节，同时丢弃接收到的无效字节。
 */
static RA01SC_Status_t RA01SC_SpiWrite(const RA01SC_Device_t *device, const uint8_t *data, uint16_t length) /* 定义函数RA01SC_SpiWrite，实现该函数对应的底层操作。 */
{ /* 开始当前函数、判断、循环或数据类型的代码块。 */
    if ((data == NULL) || (length == 0U)) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return RA01SC_STATUS_INVALID_PARAM; /* 把RA01SC_STATUS_INVALID_PARAM作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    if (HAL_SPI_Transmit(device->hardware.spi, (uint8_t *)data, length, RA01SC_SPI_TIMEOUT_MS) != HAL_OK) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return RA01SC_STATUS_SPI_ERROR; /* 把RA01SC_STATUS_SPI_ERROR作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    return RA01SC_STATUS_OK; /* 把RA01SC_STATUS_OK作为当前函数结果返回给调用者。 */
} /* 结束当前函数、判断、循环或数据类型的代码块。 */

/**
 * @brief 发送空字节并读取SPI从机返回的数据。
 */
static RA01SC_Status_t RA01SC_SpiRead(const RA01SC_Device_t *device, uint8_t *data, uint16_t length) /* 定义函数RA01SC_SpiRead，实现该函数对应的底层操作。 */
{ /* 开始当前函数、判断、循环或数据类型的代码块。 */
    uint8_t dummy_tx[16]; /* 声明变量dummy_tx，用于保存当前函数或模块运行数据。 */
    uint16_t offset; /* 声明变量offset，用于保存当前函数或模块运行数据。 */
    uint16_t chunk_length; /* 声明变量chunk_length，用于保存当前函数或模块运行数据。 */

    if ((data == NULL) || (length == 0U)) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return RA01SC_STATUS_INVALID_PARAM; /* 把RA01SC_STATUS_INVALID_PARAM作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    memset(dummy_tx, RA01SC_NOP, sizeof(dummy_tx)); /* 调用memset函数执行本行对应操作。 */
    offset = 0U; /* 计算右侧表达式，并把结果写入offset。 */

    while (offset < length) /* 在本行条件持续成立时重复执行循环体。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        chunk_length = length - offset; /* 计算右侧表达式，并把结果写入chunk_length。 */

        if (chunk_length > sizeof(dummy_tx)) /* 判断本行括号中的条件是否成立。 */
        { /* 开始当前函数、判断、循环或数据类型的代码块。 */
            chunk_length = sizeof(dummy_tx); /* 调用sizeof函数，并把返回结果保存到chunk_length。 */
        } /* 结束当前函数、判断、循环或数据类型的代码块。 */

        if (HAL_SPI_TransmitReceive(device->hardware.spi, dummy_tx, &data[offset], chunk_length, RA01SC_SPI_TIMEOUT_MS) != HAL_OK) /* 判断本行括号中的条件是否成立。 */
        { /* 开始当前函数、判断、循环或数据类型的代码块。 */
            return RA01SC_STATUS_SPI_ERROR; /* 把RA01SC_STATUS_SPI_ERROR作为当前函数结果返回给调用者。 */
        } /* 结束当前函数、判断、循环或数据类型的代码块。 */

        offset += chunk_length; /* 执行本行声明、赋值、计算或函数调用。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    return RA01SC_STATUS_OK; /* 把RA01SC_STATUS_OK作为当前函数结果返回给调用者。 */
} /* 结束当前函数、判断、循环或数据类型的代码块。 */

/**
 * @brief 向LLCC68发送一个只写命令。
 */
static RA01SC_Status_t RA01SC_WriteCommand(RA01SC_Device_t *device, uint8_t command, const uint8_t *parameters, uint16_t parameter_length) /* 定义函数RA01SC_WriteCommand，实现该函数对应的底层操作。 */
{ /* 开始当前函数、判断、循环或数据类型的代码块。 */
    RA01SC_Status_t status; /* 声明变量status，用于保存当前函数或模块运行数据。 */

    status = RA01SC_WaitWhileBusy(device, RA01SC_BUSY_TIMEOUT_MS); /* 调用RA01SC_WaitWhileBusy函数，并把返回结果保存到status。 */

    if (status != RA01SC_STATUS_OK) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return status; /* 把status作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    RA01SC_SetNss(device, GPIO_PIN_RESET); /* 调用RA01SC_SetNss函数执行本行对应操作。 */
    status = RA01SC_SpiWrite(device, &command, 1U); /* 调用RA01SC_SpiWrite函数，并把返回结果保存到status。 */

    if ((status == RA01SC_STATUS_OK) && (parameter_length > 0U)) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        status = RA01SC_SpiWrite(device, parameters, parameter_length); /* 调用RA01SC_SpiWrite函数，并把返回结果保存到status。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    RA01SC_SetNss(device, GPIO_PIN_SET); /* 调用RA01SC_SetNss函数执行本行对应操作。 */

    if (status != RA01SC_STATUS_OK) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return status; /* 把status作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    return RA01SC_WaitWhileBusy(device, RA01SC_BUSY_TIMEOUT_MS); /* 把RA01SC_WaitWhileBusy(device, RA01SC_BUSY_TIMEOUT_MS)作为当前函数结果返回给调用者。 */
} /* 结束当前函数、判断、循环或数据类型的代码块。 */

/**
 * @brief 向LLCC68发送读取命令并取得返回数据。
 */
static RA01SC_Status_t RA01SC_ReadCommand(RA01SC_Device_t *device, const uint8_t *command_header, uint16_t command_header_length, uint8_t *data, uint16_t data_length) /* 定义函数RA01SC_ReadCommand，实现该函数对应的底层操作。 */
{ /* 开始当前函数、判断、循环或数据类型的代码块。 */
    RA01SC_Status_t status; /* 声明变量status，用于保存当前函数或模块运行数据。 */

    if ((command_header == NULL) || (command_header_length == 0U) || (data == NULL) || (data_length == 0U)) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return RA01SC_STATUS_INVALID_PARAM; /* 把RA01SC_STATUS_INVALID_PARAM作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    status = RA01SC_WaitWhileBusy(device, RA01SC_BUSY_TIMEOUT_MS); /* 调用RA01SC_WaitWhileBusy函数，并把返回结果保存到status。 */

    if (status != RA01SC_STATUS_OK) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return status; /* 把status作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    RA01SC_SetNss(device, GPIO_PIN_RESET); /* 调用RA01SC_SetNss函数执行本行对应操作。 */
    status = RA01SC_SpiWrite(device, command_header, command_header_length); /* 调用RA01SC_SpiWrite函数，并把返回结果保存到status。 */

    if (status == RA01SC_STATUS_OK) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        status = RA01SC_SpiRead(device, data, data_length); /* 调用RA01SC_SpiRead函数，并把返回结果保存到status。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    RA01SC_SetNss(device, GPIO_PIN_SET); /* 调用RA01SC_SetNss函数执行本行对应操作。 */

    if (status != RA01SC_STATUS_OK) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return status; /* 把status作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    return RA01SC_WaitWhileBusy(device, RA01SC_BUSY_TIMEOUT_MS); /* 把RA01SC_WaitWhileBusy(device, RA01SC_BUSY_TIMEOUT_MS)作为当前函数结果返回给调用者。 */
} /* 结束当前函数、判断、循环或数据类型的代码块。 */

/**
 * @brief 写入一个或多个连续LLCC68寄存器。
 */
static RA01SC_Status_t RA01SC_WriteRegister(RA01SC_Device_t *device, uint16_t address, const uint8_t *data, uint8_t length) /* 定义函数RA01SC_WriteRegister，实现该函数对应的底层操作。 */
{ /* 开始当前函数、判断、循环或数据类型的代码块。 */
    RA01SC_Status_t status; /* 声明变量status，用于保存当前函数或模块运行数据。 */
    uint8_t header[3]; /* 声明变量header，用于保存当前函数或模块运行数据。 */

    if ((data == NULL) || (length == 0U)) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return RA01SC_STATUS_INVALID_PARAM; /* 把RA01SC_STATUS_INVALID_PARAM作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    header[0] = RA01SC_CMD_WRITE_REGISTER; /* 计算右侧表达式，并把结果写入header[0]。 */
    header[1] = (uint8_t)(address >> 8U); /* 计算右侧表达式，并把结果写入header[1]。 */
    header[2] = (uint8_t)address; /* 计算右侧表达式，并把结果写入header[2]。 */

    status = RA01SC_WaitWhileBusy(device, RA01SC_BUSY_TIMEOUT_MS); /* 调用RA01SC_WaitWhileBusy函数，并把返回结果保存到status。 */

    if (status != RA01SC_STATUS_OK) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return status; /* 把status作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    RA01SC_SetNss(device, GPIO_PIN_RESET); /* 调用RA01SC_SetNss函数执行本行对应操作。 */
    status = RA01SC_SpiWrite(device, header, sizeof(header)); /* 调用RA01SC_SpiWrite函数，并把返回结果保存到status。 */

    if (status == RA01SC_STATUS_OK) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        status = RA01SC_SpiWrite(device, data, length); /* 调用RA01SC_SpiWrite函数，并把返回结果保存到status。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    RA01SC_SetNss(device, GPIO_PIN_SET); /* 调用RA01SC_SetNss函数执行本行对应操作。 */

    if (status != RA01SC_STATUS_OK) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return status; /* 把status作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    return RA01SC_WaitWhileBusy(device, RA01SC_BUSY_TIMEOUT_MS); /* 把RA01SC_WaitWhileBusy(device, RA01SC_BUSY_TIMEOUT_MS)作为当前函数结果返回给调用者。 */
} /* 结束当前函数、判断、循环或数据类型的代码块。 */

/**
 * @brief 读取一个或多个连续LLCC68寄存器。
 */
static RA01SC_Status_t RA01SC_ReadRegister(RA01SC_Device_t *device, uint16_t address, uint8_t *data, uint8_t length) /* 定义函数RA01SC_ReadRegister，实现该函数对应的底层操作。 */
{ /* 开始当前函数、判断、循环或数据类型的代码块。 */
    uint8_t header[4]; /* 声明变量header，用于保存当前函数或模块运行数据。 */

    if ((data == NULL) || (length == 0U)) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return RA01SC_STATUS_INVALID_PARAM; /* 把RA01SC_STATUS_INVALID_PARAM作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    header[0] = RA01SC_CMD_READ_REGISTER; /* 计算右侧表达式，并把结果写入header[0]。 */
    header[1] = (uint8_t)(address >> 8U); /* 计算右侧表达式，并把结果写入header[1]。 */
    header[2] = (uint8_t)address; /* 计算右侧表达式，并把结果写入header[2]。 */
    header[3] = RA01SC_NOP; /* 计算右侧表达式，并把结果写入header[3]。 */

    return RA01SC_ReadCommand(device, header, sizeof(header), data, length); /* 把RA01SC_ReadCommand(device, header, sizeof(header), data, length)作为当前函数结果返回给调用者。 */
} /* 结束当前函数、判断、循环或数据类型的代码块。 */

/**
 * @brief 向LLCC68内部数据缓冲区写入数据。
 */
static RA01SC_Status_t RA01SC_WriteBuffer(RA01SC_Device_t *device, uint8_t offset, const uint8_t *data, uint8_t length) /* 定义函数RA01SC_WriteBuffer，实现该函数对应的底层操作。 */
{ /* 开始当前函数、判断、循环或数据类型的代码块。 */
    RA01SC_Status_t status; /* 声明变量status，用于保存当前函数或模块运行数据。 */
    uint8_t header[2]; /* 声明变量header，用于保存当前函数或模块运行数据。 */

    if ((data == NULL) || (length == 0U)) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return RA01SC_STATUS_INVALID_PARAM; /* 把RA01SC_STATUS_INVALID_PARAM作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    header[0] = RA01SC_CMD_WRITE_BUFFER; /* 计算右侧表达式，并把结果写入header[0]。 */
    header[1] = offset; /* 计算右侧表达式，并把结果写入header[1]。 */

    status = RA01SC_WaitWhileBusy(device, RA01SC_BUSY_TIMEOUT_MS); /* 调用RA01SC_WaitWhileBusy函数，并把返回结果保存到status。 */

    if (status != RA01SC_STATUS_OK) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return status; /* 把status作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    RA01SC_SetNss(device, GPIO_PIN_RESET); /* 调用RA01SC_SetNss函数执行本行对应操作。 */
    status = RA01SC_SpiWrite(device, header, sizeof(header)); /* 调用RA01SC_SpiWrite函数，并把返回结果保存到status。 */

    if (status == RA01SC_STATUS_OK) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        status = RA01SC_SpiWrite(device, data, length); /* 调用RA01SC_SpiWrite函数，并把返回结果保存到status。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    RA01SC_SetNss(device, GPIO_PIN_SET); /* 调用RA01SC_SetNss函数执行本行对应操作。 */

    if (status != RA01SC_STATUS_OK) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return status; /* 把status作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    return RA01SC_WaitWhileBusy(device, RA01SC_BUSY_TIMEOUT_MS); /* 把RA01SC_WaitWhileBusy(device, RA01SC_BUSY_TIMEOUT_MS)作为当前函数结果返回给调用者。 */
} /* 结束当前函数、判断、循环或数据类型的代码块。 */

/**
 * @brief 从LLCC68内部数据缓冲区读取数据。
 */
static RA01SC_Status_t RA01SC_ReadBuffer(RA01SC_Device_t *device, uint8_t offset, uint8_t *data, uint8_t length) /* 定义函数RA01SC_ReadBuffer，实现该函数对应的底层操作。 */
{ /* 开始当前函数、判断、循环或数据类型的代码块。 */
    uint8_t header[3]; /* 声明变量header，用于保存当前函数或模块运行数据。 */

    if ((data == NULL) || (length == 0U)) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return RA01SC_STATUS_INVALID_PARAM; /* 把RA01SC_STATUS_INVALID_PARAM作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    header[0] = RA01SC_CMD_READ_BUFFER; /* 计算右侧表达式，并把结果写入header[0]。 */
    header[1] = offset; /* 计算右侧表达式，并把结果写入header[1]。 */
    header[2] = RA01SC_NOP; /* 计算右侧表达式，并把结果写入header[2]。 */

    return RA01SC_ReadCommand(device, header, sizeof(header), data, length); /* 把RA01SC_ReadCommand(device, header, sizeof(header), data, length)作为当前函数结果返回给调用者。 */
} /* 结束当前函数、判断、循环或数据类型的代码块。 */

/**
 * @brief 把以Hz表示的频率转换为LLCC68的32位PLL频率字。
 */
static uint32_t RA01SC_ConvertFrequencyToPllStep(uint32_t frequency_hz) /* 定义函数RA01SC_ConvertFrequencyToPllStep，实现该函数对应的底层操作。 */
{ /* 开始当前函数、判断、循环或数据类型的代码块。 */
    uint64_t numerator; /* 声明变量numerator，用于保存当前函数或模块运行数据。 */

    numerator = ((uint64_t)frequency_hz << 25U); /* 计算右侧表达式，并把结果写入numerator。 */

    return (uint32_t)(numerator / RA01SC_XTAL_FREQUENCY_HZ); /* 把(uint32_t)(numerator / RA01SC_XTAL_FREQUENCY_HZ)作为当前函数结果返回给调用者。 */
} /* 结束当前函数、判断、循环或数据类型的代码块。 */

/**
 * @brief 把发送超时时间从毫秒转换为LLCC68的15.625us RTC计数。
 */
static uint32_t RA01SC_ConvertMillisecondsToRtcStep(uint32_t timeout_ms) /* 定义函数RA01SC_ConvertMillisecondsToRtcStep，实现该函数对应的底层操作。 */
{ /* 开始当前函数、判断、循环或数据类型的代码块。 */
    uint64_t timeout_steps; /* 声明变量timeout_steps，用于保存当前函数或模块运行数据。 */

    timeout_steps = (uint64_t)timeout_ms * 64ULL; /* 计算右侧表达式，并把结果写入timeout_steps。 */

    if (timeout_steps > 0x00FFFFFFULL) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        timeout_steps = 0x00FFFFFFULL; /* 计算右侧表达式，并把结果写入timeout_steps。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    return (uint32_t)timeout_steps; /* 把(uint32_t)timeout_steps作为当前函数结果返回给调用者。 */
} /* 结束当前函数、判断、循环或数据类型的代码块。 */

/**
 * @brief 根据带宽枚举返回实际带宽Hz值。
 */
static uint32_t RA01SC_GetBandwidthHz(RA01SC_LoraBandwidth_t bandwidth) /* 定义函数RA01SC_GetBandwidthHz，实现该函数对应的底层操作。 */
{ /* 开始当前函数、判断、循环或数据类型的代码块。 */
    if (bandwidth == RA01SC_LORA_BW_250_KHZ) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return 250000UL; /* 把250000UL作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    if (bandwidth == RA01SC_LORA_BW_500_KHZ) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return 500000UL; /* 把500000UL作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    return 125000UL; /* 把125000UL作为当前函数结果返回给调用者。 */
} /* 结束当前函数、判断、循环或数据类型的代码块。 */

/**
 * @brief 根据LoRa符号持续时间判断是否需要低数据速率优化。
 */
static uint8_t RA01SC_GetLowDataRateOptimize(const RA01SC_Config_t *config) /* 定义函数RA01SC_GetLowDataRateOptimize，实现该函数对应的底层操作。 */
{ /* 开始当前函数、判断、循环或数据类型的代码块。 */
    uint32_t bandwidth_hz; /* 声明变量bandwidth_hz，用于保存当前函数或模块运行数据。 */
    uint32_t symbol_duration_us; /* 声明变量symbol_duration_us，用于保存当前函数或模块运行数据。 */

    bandwidth_hz = RA01SC_GetBandwidthHz(config->bandwidth); /* 调用RA01SC_GetBandwidthHz函数，并把返回结果保存到bandwidth_hz。 */
    symbol_duration_us = ((1UL << (uint8_t)config->spreading_factor) * 1000000UL) / bandwidth_hz; /* 计算右侧表达式，并把结果写入symbol_duration_us。 */

    if (symbol_duration_us >= 16000UL) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return 0x01U; /* 把0x01U作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    return 0x00U; /* 把0x00U作为当前函数结果返回给调用者。 */
} /* 结束当前函数、判断、循环或数据类型的代码块。 */

/**
 * @brief 配置LoRa数据包参数，其中payload_length可在发送前动态变化。
 */
static RA01SC_Status_t RA01SC_SetPacketParameters(RA01SC_Device_t *device, uint8_t payload_length) /* 定义函数RA01SC_SetPacketParameters，实现该函数对应的底层操作。 */
{ /* 开始当前函数、判断、循环或数据类型的代码块。 */
    uint8_t parameters[6]; /* 声明变量parameters，用于保存当前函数或模块运行数据。 */

    parameters[0] = (uint8_t)(device->config.preamble_length >> 8U); /* 计算右侧表达式，并把结果写入parameters[0]。 */
    parameters[1] = (uint8_t)device->config.preamble_length; /* 计算右侧表达式，并把结果写入parameters[1]。 */
    parameters[2] = (uint8_t)device->config.header_type; /* 计算右侧表达式，并把结果写入parameters[2]。 */
    parameters[3] = payload_length; /* 计算右侧表达式，并把结果写入parameters[3]。 */
    parameters[4] = device->config.crc_enabled ? 0x01U : 0x00U; /* 计算右侧表达式，并把结果写入parameters[4]。 */
    parameters[5] = device->config.invert_iq ? 0x01U : 0x00U; /* 计算右侧表达式，并把结果写入parameters[5]。 */

    return RA01SC_WriteCommand(device, RA01SC_CMD_SET_PACKET_PARAMS, parameters, sizeof(parameters)); /* 把RA01SC_WriteCommand(device, RA01SC_CMD_SET_PACKET_PARAMS, parameters, sizeof(parameters))作为当前函数结果返回给调用者。 */
} /* 结束当前函数、判断、循环或数据类型的代码块。 */

/**
 * @brief 配置与当前频段对应的镜像抑制校准参数。
 */
static RA01SC_Status_t RA01SC_CalibrateImage(RA01SC_Device_t *device) /* 定义函数RA01SC_CalibrateImage，实现该函数对应的底层操作。 */
{ /* 开始当前函数、判断、循环或数据类型的代码块。 */
    uint32_t frequency_mhz; /* 声明变量frequency_mhz，用于保存当前函数或模块运行数据。 */
    uint32_t lower_frequency_mhz; /* 声明变量lower_frequency_mhz，用于保存当前函数或模块运行数据。 */
    uint32_t upper_frequency_mhz; /* 声明变量upper_frequency_mhz，用于保存当前函数或模块运行数据。 */
    uint8_t parameters[2]; /* 声明变量parameters，用于保存当前函数或模块运行数据。 */

    frequency_mhz = device->config.frequency_hz / 1000000UL; /* 计算右侧表达式，并把结果写入frequency_mhz。 */
    lower_frequency_mhz = frequency_mhz > 4UL ? frequency_mhz - 4UL : frequency_mhz; /* 计算右侧表达式，并把结果写入lower_frequency_mhz。 */
    upper_frequency_mhz = frequency_mhz + 4UL; /* 计算右侧表达式，并把结果写入upper_frequency_mhz。 */
    parameters[0] = (uint8_t)(lower_frequency_mhz / 4UL); /* 计算右侧表达式，并把结果写入parameters[0]。 */
    parameters[1] = (uint8_t)((upper_frequency_mhz + 3UL) / 4UL); /* 计算右侧表达式，并把结果写入parameters[1]。 */

    return RA01SC_WriteCommand(device, RA01SC_CMD_CALIBRATE_IMAGE, parameters, sizeof(parameters)); /* 把RA01SC_WriteCommand(device, RA01SC_CMD_CALIBRATE_IMAGE, parameters, sizeof(parameters))作为当前函数结果返回给调用者。 */
} /* 结束当前函数、判断、循环或数据类型的代码块。 */

/**
 * @brief 配置功放、过流保护和发射钳位修正。
 */
static RA01SC_Status_t RA01SC_ConfigurePowerAmplifier(RA01SC_Device_t *device) /* 定义函数RA01SC_ConfigurePowerAmplifier，实现该函数对应的底层操作。 */
{ /* 开始当前函数、判断、循环或数据类型的代码块。 */
    RA01SC_Status_t status; /* 声明变量status，用于保存当前函数或模块运行数据。 */
    uint8_t pa_parameters[4]; /* 声明变量pa_parameters，用于保存当前函数或模块运行数据。 */
    uint8_t tx_parameters[2]; /* 声明变量tx_parameters，用于保存当前函数或模块运行数据。 */
    uint8_t register_value; /* 声明变量register_value，用于保存当前函数或模块运行数据。 */

    pa_parameters[0] = 0x02U; /* 计算右侧表达式，并把结果写入pa_parameters[0]。 */
    pa_parameters[1] = 0x03U; /* 计算右侧表达式，并把结果写入pa_parameters[1]。 */
    pa_parameters[2] = 0x00U; /* 计算右侧表达式，并把结果写入pa_parameters[2]。 */
    pa_parameters[3] = 0x01U; /* 计算右侧表达式，并把结果写入pa_parameters[3]。 */
    status = RA01SC_WriteCommand(device, RA01SC_CMD_SET_PA_CONFIG, pa_parameters, sizeof(pa_parameters)); /* 调用RA01SC_WriteCommand函数，并把返回结果保存到status。 */

    if (status != RA01SC_STATUS_OK) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return status; /* 把status作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    register_value = RA01SC_OCP_140_MA; /* 计算右侧表达式，并把结果写入register_value。 */
    status = RA01SC_WriteRegister(device, RA01SC_REG_OCP, &register_value, 1U); /* 调用RA01SC_WriteRegister函数，并把返回结果保存到status。 */

    if (status != RA01SC_STATUS_OK) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return status; /* 把status作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    status = RA01SC_ReadRegister(device, RA01SC_REG_TX_CLAMP_CONFIG, &register_value, 1U); /* 调用RA01SC_ReadRegister函数，并把返回结果保存到status。 */

    if (status != RA01SC_STATUS_OK) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return status; /* 把status作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    register_value |= RA01SC_TX_CLAMP_CONFIG_MASK; /* 将register_value与指定掩码合并后写回。 */
    status = RA01SC_WriteRegister(device, RA01SC_REG_TX_CLAMP_CONFIG, &register_value, 1U); /* 调用RA01SC_WriteRegister函数，并把返回结果保存到status。 */

    if (status != RA01SC_STATUS_OK) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return status; /* 把status作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    tx_parameters[0] = (uint8_t)device->config.tx_power_dbm; /* 计算右侧表达式，并把结果写入tx_parameters[0]。 */
    tx_parameters[1] = RA01SC_RAMP_TIME_200_US; /* 计算右侧表达式，并把结果写入tx_parameters[1]。 */

    return RA01SC_WriteCommand(device, RA01SC_CMD_SET_TX_PARAMS, tx_parameters, sizeof(tx_parameters)); /* 把RA01SC_WriteCommand(device, RA01SC_CMD_SET_TX_PARAMS, tx_parameters, sizeof(tx_parameters))作为当前函数结果返回给调用者。 */
} /* 结束当前函数、判断、循环或数据类型的代码块。 */

/**
 * @brief 填充适合首轮PING/PONG和遥控链路测试的默认参数。
 */
void RA01SC_GetDefaultConfig(RA01SC_Config_t *config) /* 定义函数RA01SC_GetDefaultConfig，实现该函数对应的底层操作。 */
{ /* 开始当前函数、判断、循环或数据类型的代码块。 */
    if (config == NULL) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return; /* 执行本行声明、赋值、计算或函数调用。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    config->frequency_hz = 433920000UL; /* 计算右侧表达式，并把结果写入config->frequency_hz。 */
    config->spreading_factor = RA01SC_LORA_SF7; /* 计算右侧表达式，并把结果写入config->spreading_factor。 */
    config->bandwidth = RA01SC_LORA_BW_125_KHZ; /* 计算右侧表达式，并把结果写入config->bandwidth。 */
    config->coding_rate = RA01SC_LORA_CR_4_5; /* 计算右侧表达式，并把结果写入config->coding_rate。 */
    config->preamble_length = 8U; /* 计算右侧表达式，并把结果写入config->preamble_length。 */
    config->header_type = RA01SC_LORA_HEADER_EXPLICIT; /* 计算右侧表达式，并把结果写入config->header_type。 */
    config->max_payload_length = RA01SC_MAX_PAYLOAD_LENGTH; /* 计算右侧表达式，并把结果写入config->max_payload_length。 */
    config->crc_enabled = true; /* 计算右侧表达式，并把结果写入config->crc_enabled。 */
    config->invert_iq = false; /* 计算右侧表达式，并把结果写入config->invert_iq。 */
    config->tx_power_dbm = 10; /* 计算右侧表达式，并把结果写入config->tx_power_dbm。 */
    config->sync_word = RA01SC_LORA_PUBLIC_SYNC_WORD; /* 计算右侧表达式，并把结果写入config->sync_word。 */
    config->tx_timeout_ms = 3000U; /* 计算右侧表达式，并把结果写入config->tx_timeout_ms。 */
} /* 结束当前函数、判断、循环或数据类型的代码块。 */

/**
 * @brief 复位并初始化RA-01SC。
 */
RA01SC_Status_t RA01SC_Init(RA01SC_Device_t *device, const RA01SC_Hardware_t *hardware, const RA01SC_Config_t *config) /* 定义函数RA01SC_Init，实现该函数对应的底层操作。 */
{ /* 开始当前函数、判断、循环或数据类型的代码块。 */
    RA01SC_Status_t status; /* 声明变量status，用于保存当前函数或模块运行数据。 */
    uint8_t chip_status; /* 声明变量chip_status，用于保存当前函数或模块运行数据。 */

    if ((device == NULL) || (hardware == NULL) || (config == NULL)) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return RA01SC_STATUS_INVALID_PARAM; /* 把RA01SC_STATUS_INVALID_PARAM作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    memset(device, 0, sizeof(RA01SC_Device_t)); /* 调用memset函数执行本行对应操作。 */
    device->hardware = *hardware; /* 计算右侧表达式，并把结果写入device->hardware。 */
    device->config = *config; /* 计算右侧表达式，并把结果写入device->config。 */
    device->initialized = false; /* 计算右侧表达式，并把结果写入device->initialized。 */

    if ((RA01SC_IsHardwareValid(device) == false) || (RA01SC_IsConfigValid(config) == false)) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return RA01SC_STATUS_INVALID_PARAM; /* 把RA01SC_STATUS_INVALID_PARAM作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    RA01SC_SetNss(device, GPIO_PIN_SET); /* 调用RA01SC_SetNss函数执行本行对应操作。 */
    RA01SC_SetRfSwitchStandby(device); /* 调用RA01SC_SetRfSwitchStandby函数执行本行对应操作。 */
    status = RA01SC_Reset(device); /* 调用RA01SC_Reset函数，并把返回结果保存到status。 */

    if (status != RA01SC_STATUS_OK) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return status; /* 把status作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    status = RA01SC_ApplyConfig(device); /* 调用RA01SC_ApplyConfig函数，并把返回结果保存到status。 */

    if (status != RA01SC_STATUS_OK) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return status; /* 把status作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    status = RA01SC_GetChipStatus(device, &chip_status); /* 调用RA01SC_GetChipStatus函数，并把返回结果保存到status。 */

    if (status != RA01SC_STATUS_OK) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return status; /* 把status作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    if ((chip_status == 0x00U) || (chip_status == 0xFFU)) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return RA01SC_STATUS_CHIP_NOT_RESPONDING; /* 把RA01SC_STATUS_CHIP_NOT_RESPONDING作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    device->initialized = true; /* 计算右侧表达式，并把结果写入device->initialized。 */

    return RA01SC_STATUS_OK; /* 把RA01SC_STATUS_OK作为当前函数结果返回给调用者。 */
} /* 结束当前函数、判断、循环或数据类型的代码块。 */

/**
 * @brief 执行硬件复位。
 */
RA01SC_Status_t RA01SC_Reset(RA01SC_Device_t *device) /* 定义函数RA01SC_Reset，实现该函数对应的底层操作。 */
{ /* 开始当前函数、判断、循环或数据类型的代码块。 */
    if (RA01SC_IsHardwareValid(device) == false) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return RA01SC_STATUS_INVALID_PARAM; /* 把RA01SC_STATUS_INVALID_PARAM作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    device->initialized = false; /* 计算右侧表达式，并把结果写入device->initialized。 */
    RA01SC_SetNss(device, GPIO_PIN_SET); /* 调用RA01SC_SetNss函数执行本行对应操作。 */
    RA01SC_SetRfSwitchStandby(device); /* 调用RA01SC_SetRfSwitchStandby函数执行本行对应操作。 */
    HAL_GPIO_WritePin(device->hardware.reset_port, device->hardware.reset_pin, GPIO_PIN_RESET); /* 调用HAL_GPIO_WritePin函数执行本行对应操作。 */
    HAL_Delay(2U); /* 调用HAL_Delay函数执行本行对应操作。 */
    HAL_GPIO_WritePin(device->hardware.reset_port, device->hardware.reset_pin, GPIO_PIN_SET); /* 调用HAL_GPIO_WritePin函数执行本行对应操作。 */
    HAL_Delay(10U); /* 调用HAL_Delay函数执行本行对应操作。 */

    return RA01SC_WaitWhileBusy(device, RA01SC_BUSY_TIMEOUT_MS); /* 把RA01SC_WaitWhileBusy(device, RA01SC_BUSY_TIMEOUT_MS)作为当前函数结果返回给调用者。 */
} /* 结束当前函数、判断、循环或数据类型的代码块。 */

/**
 * @brief 写入完整LoRa配置。
 */
RA01SC_Status_t RA01SC_ApplyConfig(RA01SC_Device_t *device) /* 定义函数RA01SC_ApplyConfig，实现该函数对应的底层操作。 */
{ /* 开始当前函数、判断、循环或数据类型的代码块。 */
    RA01SC_Status_t status; /* 声明变量status，用于保存当前函数或模块运行数据。 */
    uint32_t rf_frequency_word; /* 声明变量rf_frequency_word，用于保存当前函数或模块运行数据。 */
    uint16_t irq_mask; /* 声明变量irq_mask，用于保存当前函数或模块运行数据。 */
    uint8_t parameters[8]; /* 声明变量parameters，用于保存当前函数或模块运行数据。 */
    uint8_t sync_word[2]; /* 声明变量sync_word，用于保存当前函数或模块运行数据。 */

    if ((RA01SC_IsHardwareValid(device) == false) || (RA01SC_IsConfigValid(&device->config) == false)) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return RA01SC_STATUS_INVALID_PARAM; /* 把RA01SC_STATUS_INVALID_PARAM作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    parameters[0] = RA01SC_STANDBY_RC; /* 计算右侧表达式，并把结果写入parameters[0]。 */
    status = RA01SC_WriteCommand(device, RA01SC_CMD_SET_STANDBY, parameters, 1U); /* 调用RA01SC_WriteCommand函数，并把返回结果保存到status。 */

    if (status != RA01SC_STATUS_OK) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return status; /* 把status作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    parameters[0] = RA01SC_REGULATOR_DCDC; /* 计算右侧表达式，并把结果写入parameters[0]。 */
    status = RA01SC_WriteCommand(device, RA01SC_CMD_SET_REGULATOR_MODE, parameters, 1U); /* 调用RA01SC_WriteCommand函数，并把返回结果保存到status。 */

    if (status != RA01SC_STATUS_OK) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return status; /* 把status作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    parameters[0] = 0x7FU; /* 计算右侧表达式，并把结果写入parameters[0]。 */
    status = RA01SC_WriteCommand(device, RA01SC_CMD_CALIBRATE, parameters, 1U); /* 调用RA01SC_WriteCommand函数，并把返回结果保存到status。 */

    if (status != RA01SC_STATUS_OK) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return status; /* 把status作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    status = RA01SC_CalibrateImage(device); /* 调用RA01SC_CalibrateImage函数，并把返回结果保存到status。 */

    if (status != RA01SC_STATUS_OK) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return status; /* 把status作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    parameters[0] = RA01SC_PACKET_TYPE_LORA; /* 计算右侧表达式，并把结果写入parameters[0]。 */
    status = RA01SC_WriteCommand(device, RA01SC_CMD_SET_PACKET_TYPE, parameters, 1U); /* 调用RA01SC_WriteCommand函数，并把返回结果保存到status。 */

    if (status != RA01SC_STATUS_OK) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return status; /* 把status作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    rf_frequency_word = RA01SC_ConvertFrequencyToPllStep(device->config.frequency_hz); /* 调用RA01SC_ConvertFrequencyToPllStep函数，并把返回结果保存到rf_frequency_word。 */
    parameters[0] = (uint8_t)(rf_frequency_word >> 24U); /* 计算右侧表达式，并把结果写入parameters[0]。 */
    parameters[1] = (uint8_t)(rf_frequency_word >> 16U); /* 计算右侧表达式，并把结果写入parameters[1]。 */
    parameters[2] = (uint8_t)(rf_frequency_word >> 8U); /* 计算右侧表达式，并把结果写入parameters[2]。 */
    parameters[3] = (uint8_t)rf_frequency_word; /* 计算右侧表达式，并把结果写入parameters[3]。 */
    status = RA01SC_WriteCommand(device, RA01SC_CMD_SET_RF_FREQUENCY, parameters, 4U); /* 调用RA01SC_WriteCommand函数，并把返回结果保存到status。 */

    if (status != RA01SC_STATUS_OK) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return status; /* 把status作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    status = RA01SC_ConfigurePowerAmplifier(device); /* 调用RA01SC_ConfigurePowerAmplifier函数，并把返回结果保存到status。 */

    if (status != RA01SC_STATUS_OK) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return status; /* 把status作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    parameters[0] = (uint8_t)device->config.spreading_factor; /* 计算右侧表达式，并把结果写入parameters[0]。 */
    parameters[1] = (uint8_t)device->config.bandwidth; /* 计算右侧表达式，并把结果写入parameters[1]。 */
    parameters[2] = (uint8_t)device->config.coding_rate; /* 计算右侧表达式，并把结果写入parameters[2]。 */
    parameters[3] = RA01SC_GetLowDataRateOptimize(&device->config); /* 调用RA01SC_GetLowDataRateOptimize函数，并把返回结果保存到parameters[3]。 */
    status = RA01SC_WriteCommand(device, RA01SC_CMD_SET_MODULATION_PARAMS, parameters, 4U); /* 调用RA01SC_WriteCommand函数，并把返回结果保存到status。 */

    if (status != RA01SC_STATUS_OK) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return status; /* 把status作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    status = RA01SC_SetPacketParameters(device, device->config.max_payload_length); /* 调用RA01SC_SetPacketParameters函数，并把返回结果保存到status。 */

    if (status != RA01SC_STATUS_OK) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return status; /* 把status作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    parameters[0] = 0x00U; /* 计算右侧表达式，并把结果写入parameters[0]。 */
    parameters[1] = 0x00U; /* 计算右侧表达式，并把结果写入parameters[1]。 */
    status = RA01SC_WriteCommand(device, RA01SC_CMD_SET_BUFFER_BASE_ADDRESS, parameters, 2U); /* 调用RA01SC_WriteCommand函数，并把返回结果保存到status。 */

    if (status != RA01SC_STATUS_OK) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return status; /* 把status作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    sync_word[0] = (uint8_t)(device->config.sync_word >> 8U); /* 计算右侧表达式，并把结果写入sync_word[0]。 */
    sync_word[1] = (uint8_t)device->config.sync_word; /* 计算右侧表达式，并把结果写入sync_word[1]。 */
    status = RA01SC_WriteRegister(device, RA01SC_REG_LORA_SYNC_WORD, sync_word, sizeof(sync_word)); /* 调用RA01SC_WriteRegister函数，并把返回结果保存到status。 */

    if (status != RA01SC_STATUS_OK) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return status; /* 把status作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    parameters[0] = RA01SC_FALLBACK_STANDBY_RC; /* 计算右侧表达式，并把结果写入parameters[0]。 */
    status = RA01SC_WriteCommand(device, RA01SC_CMD_SET_RX_TX_FALLBACK_MODE, parameters, 1U); /* 调用RA01SC_WriteCommand函数，并把返回结果保存到status。 */

    if (status != RA01SC_STATUS_OK) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return status; /* 把status作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    irq_mask = RA01SC_IRQ_TX_DONE | RA01SC_IRQ_RX_DONE | RA01SC_IRQ_HEADER_ERROR | RA01SC_IRQ_CRC_ERROR | RA01SC_IRQ_TIMEOUT; /* 计算右侧表达式，并把结果写入irq_mask。 */
    parameters[0] = (uint8_t)(irq_mask >> 8U); /* 计算右侧表达式，并把结果写入parameters[0]。 */
    parameters[1] = (uint8_t)irq_mask; /* 计算右侧表达式，并把结果写入parameters[1]。 */
    parameters[2] = (uint8_t)(irq_mask >> 8U); /* 计算右侧表达式，并把结果写入parameters[2]。 */
    parameters[3] = (uint8_t)irq_mask; /* 计算右侧表达式，并把结果写入parameters[3]。 */
    parameters[4] = 0x00U; /* 计算右侧表达式，并把结果写入parameters[4]。 */
    parameters[5] = 0x00U; /* 计算右侧表达式，并把结果写入parameters[5]。 */
    parameters[6] = 0x00U; /* 计算右侧表达式，并把结果写入parameters[6]。 */
    parameters[7] = 0x00U; /* 计算右侧表达式，并把结果写入parameters[7]。 */
    status = RA01SC_WriteCommand(device, RA01SC_CMD_SET_DIO_IRQ_PARAMS, parameters, sizeof(parameters)); /* 调用RA01SC_WriteCommand函数，并把返回结果保存到status。 */

    if (status != RA01SC_STATUS_OK) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return status; /* 把status作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    parameters[0] = (uint8_t)(RA01SC_IRQ_ALL >> 8U); /* 计算右侧表达式，并把结果写入parameters[0]。 */
    parameters[1] = (uint8_t)RA01SC_IRQ_ALL; /* 计算右侧表达式，并把结果写入parameters[1]。 */
    status = RA01SC_WriteCommand(device, RA01SC_CMD_CLEAR_IRQ_STATUS, parameters, 2U); /* 调用RA01SC_WriteCommand函数，并把返回结果保存到status。 */

    if (status != RA01SC_STATUS_OK) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return status; /* 把status作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    RA01SC_SetRfSwitchStandby(device); /* 调用RA01SC_SetRfSwitchStandby函数执行本行对应操作。 */

    return RA01SC_STATUS_OK; /* 把RA01SC_STATUS_OK作为当前函数结果返回给调用者。 */
} /* 结束当前函数、判断、循环或数据类型的代码块。 */

/**
 * @brief 读取芯片状态字节。
 */
RA01SC_Status_t RA01SC_GetChipStatus(RA01SC_Device_t *device, uint8_t *chip_status) /* 定义函数RA01SC_GetChipStatus，实现该函数对应的底层操作。 */
{ /* 开始当前函数、判断、循环或数据类型的代码块。 */
    uint8_t command; /* 声明变量command，用于保存当前函数或模块运行数据。 */

    if ((RA01SC_IsHardwareValid(device) == false) || (chip_status == NULL)) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return RA01SC_STATUS_INVALID_PARAM; /* 把RA01SC_STATUS_INVALID_PARAM作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    command = RA01SC_CMD_GET_STATUS; /* 计算右侧表达式，并把结果写入command。 */

    return RA01SC_ReadCommand(device, &command, 1U, chip_status, 1U); /* 把RA01SC_ReadCommand(device, &command, 1U, chip_status, 1U)作为当前函数结果返回给调用者。 */
} /* 结束当前函数、判断、循环或数据类型的代码块。 */

/**
 * @brief 进入STDBY_RC待机模式。
 */
RA01SC_Status_t RA01SC_SetStandby(RA01SC_Device_t *device) /* 定义函数RA01SC_SetStandby，实现该函数对应的底层操作。 */
{ /* 开始当前函数、判断、循环或数据类型的代码块。 */
    RA01SC_Status_t status; /* 声明变量status，用于保存当前函数或模块运行数据。 */
    uint8_t parameter; /* 声明变量parameter，用于保存当前函数或模块运行数据。 */

    if ((device == NULL) || (device->initialized == false)) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return RA01SC_STATUS_NOT_INITIALIZED; /* 把RA01SC_STATUS_NOT_INITIALIZED作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    parameter = RA01SC_STANDBY_RC; /* 计算右侧表达式，并把结果写入parameter。 */
    status = RA01SC_WriteCommand(device, RA01SC_CMD_SET_STANDBY, &parameter, 1U); /* 调用RA01SC_WriteCommand函数，并把返回结果保存到status。 */

    if (status == RA01SC_STATUS_OK) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        RA01SC_SetRfSwitchStandby(device); /* 调用RA01SC_SetRfSwitchStandby函数执行本行对应操作。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    return status; /* 把status作为当前函数结果返回给调用者。 */
} /* 结束当前函数、判断、循环或数据类型的代码块。 */

/**
 * @brief 启动连续接收。
 */
RA01SC_Status_t RA01SC_StartReceive(RA01SC_Device_t *device) /* 定义函数RA01SC_StartReceive，实现该函数对应的底层操作。 */
{ /* 开始当前函数、判断、循环或数据类型的代码块。 */
    RA01SC_Status_t status; /* 声明变量status，用于保存当前函数或模块运行数据。 */
    uint8_t parameters[3]; /* 声明变量parameters，用于保存当前函数或模块运行数据。 */

    if ((device == NULL) || (device->initialized == false)) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return RA01SC_STATUS_NOT_INITIALIZED; /* 把RA01SC_STATUS_NOT_INITIALIZED作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    status = RA01SC_SetPacketParameters(device, device->config.max_payload_length); /* 调用RA01SC_SetPacketParameters函数，并把返回结果保存到status。 */

    if (status != RA01SC_STATUS_OK) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return status; /* 把status作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    parameters[0] = (uint8_t)(RA01SC_IRQ_ALL >> 8U); /* 计算右侧表达式，并把结果写入parameters[0]。 */
    parameters[1] = (uint8_t)RA01SC_IRQ_ALL; /* 计算右侧表达式，并把结果写入parameters[1]。 */
    status = RA01SC_WriteCommand(device, RA01SC_CMD_CLEAR_IRQ_STATUS, parameters, 2U); /* 调用RA01SC_WriteCommand函数，并把返回结果保存到status。 */

    if (status != RA01SC_STATUS_OK) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return status; /* 把status作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    RA01SC_SetRfSwitchReceive(device); /* 调用RA01SC_SetRfSwitchReceive函数执行本行对应操作。 */
    parameters[0] = (uint8_t)(RA01SC_RX_CONTINUOUS_TIMEOUT >> 16U); /* 计算右侧表达式，并把结果写入parameters[0]。 */
    parameters[1] = (uint8_t)(RA01SC_RX_CONTINUOUS_TIMEOUT >> 8U); /* 计算右侧表达式，并把结果写入parameters[1]。 */
    parameters[2] = (uint8_t)RA01SC_RX_CONTINUOUS_TIMEOUT; /* 计算右侧表达式，并把结果写入parameters[2]。 */
    status = RA01SC_WriteCommand(device, RA01SC_CMD_SET_RX, parameters, sizeof(parameters)); /* 调用RA01SC_WriteCommand函数，并把返回结果保存到status。 */

    if (status != RA01SC_STATUS_OK) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        RA01SC_SetRfSwitchStandby(device); /* 调用RA01SC_SetRfSwitchStandby函数执行本行对应操作。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    return status; /* 把status作为当前函数结果返回给调用者。 */
} /* 结束当前函数、判断、循环或数据类型的代码块。 */

/**
 * @brief 写入负载并启动一次发送。
 */
RA01SC_Status_t RA01SC_Send(RA01SC_Device_t *device, const uint8_t *data, uint8_t length) /* 定义函数RA01SC_Send，实现该函数对应的底层操作。 */
{ /* 开始当前函数、判断、循环或数据类型的代码块。 */
    RA01SC_Status_t status; /* 声明变量status，用于保存当前函数或模块运行数据。 */
    uint32_t timeout_steps; /* 声明变量timeout_steps，用于保存当前函数或模块运行数据。 */
    uint8_t parameters[3]; /* 声明变量parameters，用于保存当前函数或模块运行数据。 */

    if ((device == NULL) || (device->initialized == false)) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return RA01SC_STATUS_NOT_INITIALIZED; /* 把RA01SC_STATUS_NOT_INITIALIZED作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    if ((data == NULL) || (length == 0U)) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return RA01SC_STATUS_INVALID_PARAM; /* 把RA01SC_STATUS_INVALID_PARAM作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    if (length > device->config.max_payload_length) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return RA01SC_STATUS_PACKET_TOO_LONG; /* 把RA01SC_STATUS_PACKET_TOO_LONG作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    status = RA01SC_SetPacketParameters(device, length); /* 调用RA01SC_SetPacketParameters函数，并把返回结果保存到status。 */

    if (status != RA01SC_STATUS_OK) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return status; /* 把status作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    status = RA01SC_WriteBuffer(device, 0x00U, data, length); /* 调用RA01SC_WriteBuffer函数，并把返回结果保存到status。 */

    if (status != RA01SC_STATUS_OK) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return status; /* 把status作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    parameters[0] = (uint8_t)(RA01SC_IRQ_ALL >> 8U); /* 计算右侧表达式，并把结果写入parameters[0]。 */
    parameters[1] = (uint8_t)RA01SC_IRQ_ALL; /* 计算右侧表达式，并把结果写入parameters[1]。 */
    status = RA01SC_WriteCommand(device, RA01SC_CMD_CLEAR_IRQ_STATUS, parameters, 2U); /* 调用RA01SC_WriteCommand函数，并把返回结果保存到status。 */

    if (status != RA01SC_STATUS_OK) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return status; /* 把status作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    timeout_steps = RA01SC_ConvertMillisecondsToRtcStep(device->config.tx_timeout_ms); /* 调用RA01SC_ConvertMillisecondsToRtcStep函数，并把返回结果保存到timeout_steps。 */
    parameters[0] = (uint8_t)(timeout_steps >> 16U); /* 计算右侧表达式，并把结果写入parameters[0]。 */
    parameters[1] = (uint8_t)(timeout_steps >> 8U); /* 计算右侧表达式，并把结果写入parameters[1]。 */
    parameters[2] = (uint8_t)timeout_steps; /* 计算右侧表达式，并把结果写入parameters[2]。 */
    RA01SC_SetRfSwitchTransmit(device); /* 调用RA01SC_SetRfSwitchTransmit函数执行本行对应操作。 */
    status = RA01SC_WriteCommand(device, RA01SC_CMD_SET_TX, parameters, sizeof(parameters)); /* 调用RA01SC_WriteCommand函数，并把返回结果保存到status。 */

    if (status != RA01SC_STATUS_OK) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        RA01SC_SetRfSwitchStandby(device); /* 调用RA01SC_SetRfSwitchStandby函数执行本行对应操作。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    return status; /* 把status作为当前函数结果返回给调用者。 */
} /* 结束当前函数、判断、循环或数据类型的代码块。 */

/**
 * @brief 读取并清除中断状态。
 */
RA01SC_Status_t RA01SC_GetAndClearIrqStatus(RA01SC_Device_t *device, uint16_t *irq_status) /* 定义函数RA01SC_GetAndClearIrqStatus，实现该函数对应的底层操作。 */
{ /* 开始当前函数、判断、循环或数据类型的代码块。 */
    RA01SC_Status_t status; /* 声明变量status，用于保存当前函数或模块运行数据。 */
    uint8_t command_header[2]; /* 声明变量command_header，用于保存当前函数或模块运行数据。 */
    uint8_t irq_bytes[2]; /* 声明变量irq_bytes，用于保存当前函数或模块运行数据。 */
    uint8_t clear_parameters[2]; /* 声明变量clear_parameters，用于保存当前函数或模块运行数据。 */

    if ((device == NULL) || (device->initialized == false)) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return RA01SC_STATUS_NOT_INITIALIZED; /* 把RA01SC_STATUS_NOT_INITIALIZED作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    if (irq_status == NULL) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return RA01SC_STATUS_INVALID_PARAM; /* 把RA01SC_STATUS_INVALID_PARAM作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    command_header[0] = RA01SC_CMD_GET_IRQ_STATUS; /* 计算右侧表达式，并把结果写入command_header[0]。 */
    command_header[1] = RA01SC_NOP; /* 计算右侧表达式，并把结果写入command_header[1]。 */
    status = RA01SC_ReadCommand(device, command_header, sizeof(command_header), irq_bytes, sizeof(irq_bytes)); /* 调用RA01SC_ReadCommand函数，并把返回结果保存到status。 */

    if (status != RA01SC_STATUS_OK) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return status; /* 把status作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    *irq_status = ((uint16_t)irq_bytes[0] << 8U) | irq_bytes[1];

    if (*irq_status != RA01SC_IRQ_NONE) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        clear_parameters[0] = irq_bytes[0]; /* 计算右侧表达式，并把结果写入clear_parameters[0]。 */
        clear_parameters[1] = irq_bytes[1]; /* 计算右侧表达式，并把结果写入clear_parameters[1]。 */
        status = RA01SC_WriteCommand(device, RA01SC_CMD_CLEAR_IRQ_STATUS, clear_parameters, sizeof(clear_parameters)); /* 调用RA01SC_WriteCommand函数，并把返回结果保存到status。 */

        if (status != RA01SC_STATUS_OK) /* 判断本行括号中的条件是否成立。 */
        { /* 开始当前函数、判断、循环或数据类型的代码块。 */
            return status; /* 把status作为当前函数结果返回给调用者。 */
        } /* 结束当前函数、判断、循环或数据类型的代码块。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    if ((*irq_status & (RA01SC_IRQ_TX_DONE | RA01SC_IRQ_TIMEOUT)) != 0U) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        RA01SC_SetRfSwitchStandby(device); /* 调用RA01SC_SetRfSwitchStandby函数执行本行对应操作。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    return RA01SC_STATUS_OK; /* 把RA01SC_STATUS_OK作为当前函数结果返回给调用者。 */
} /* 结束当前函数、判断、循环或数据类型的代码块。 */

/**
 * @brief 读取接收缓冲区中的最新数据包。
 */
RA01SC_Status_t RA01SC_ReadPacket(RA01SC_Device_t *device, uint8_t *data, uint8_t capacity, uint8_t *length) /* 定义函数RA01SC_ReadPacket，实现该函数对应的底层操作。 */
{ /* 开始当前函数、判断、循环或数据类型的代码块。 */
    RA01SC_Status_t status; /* 声明变量status，用于保存当前函数或模块运行数据。 */
    uint8_t command_header[2]; /* 声明变量command_header，用于保存当前函数或模块运行数据。 */
    uint8_t buffer_status[2]; /* 声明变量buffer_status，用于保存当前函数或模块运行数据。 */
    uint8_t payload_length; /* 声明变量payload_length，用于保存当前函数或模块运行数据。 */
    uint8_t buffer_offset; /* 声明变量buffer_offset，用于保存当前函数或模块运行数据。 */

    if ((device == NULL) || (device->initialized == false)) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return RA01SC_STATUS_NOT_INITIALIZED; /* 把RA01SC_STATUS_NOT_INITIALIZED作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    if ((data == NULL) || (length == NULL) || (capacity == 0U)) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return RA01SC_STATUS_INVALID_PARAM; /* 把RA01SC_STATUS_INVALID_PARAM作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    command_header[0] = RA01SC_CMD_GET_RX_BUFFER_STATUS; /* 计算右侧表达式，并把结果写入command_header[0]。 */
    command_header[1] = RA01SC_NOP; /* 计算右侧表达式，并把结果写入command_header[1]。 */
    status = RA01SC_ReadCommand(device, command_header, sizeof(command_header), buffer_status, sizeof(buffer_status)); /* 调用RA01SC_ReadCommand函数，并把返回结果保存到status。 */

    if (status != RA01SC_STATUS_OK) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return status; /* 把status作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    payload_length = buffer_status[0]; /* 计算右侧表达式，并把结果写入payload_length。 */
    buffer_offset = buffer_status[1]; /* 计算右侧表达式，并把结果写入buffer_offset。 */

    if (payload_length == 0U) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        *length = 0U;
        return RA01SC_STATUS_NO_PACKET; /* 把RA01SC_STATUS_NO_PACKET作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    if (payload_length > capacity) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        *length = payload_length;
        return RA01SC_STATUS_PACKET_TOO_LONG; /* 把RA01SC_STATUS_PACKET_TOO_LONG作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    status = RA01SC_ReadBuffer(device, buffer_offset, data, payload_length); /* 调用RA01SC_ReadBuffer函数，并把返回结果保存到status。 */

    if (status != RA01SC_STATUS_OK) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return status; /* 把status作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    *length = payload_length;

    return RA01SC_STATUS_OK; /* 把RA01SC_STATUS_OK作为当前函数结果返回给调用者。 */
} /* 结束当前函数、判断、循环或数据类型的代码块。 */

/**
 * @brief 读取最近一次数据包的RSSI和SNR。
 */
RA01SC_Status_t RA01SC_GetPacketStatus(RA01SC_Device_t *device, RA01SC_PacketStatus_t *packet_status) /* 定义函数RA01SC_GetPacketStatus，实现该函数对应的底层操作。 */
{ /* 开始当前函数、判断、循环或数据类型的代码块。 */
    RA01SC_Status_t status; /* 声明变量status，用于保存当前函数或模块运行数据。 */
    uint8_t command_header[2]; /* 声明变量command_header，用于保存当前函数或模块运行数据。 */
    uint8_t status_bytes[3]; /* 声明变量status_bytes，用于保存当前函数或模块运行数据。 */

    if ((device == NULL) || (device->initialized == false)) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return RA01SC_STATUS_NOT_INITIALIZED; /* 把RA01SC_STATUS_NOT_INITIALIZED作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    if (packet_status == NULL) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return RA01SC_STATUS_INVALID_PARAM; /* 把RA01SC_STATUS_INVALID_PARAM作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    command_header[0] = RA01SC_CMD_GET_PACKET_STATUS; /* 计算右侧表达式，并把结果写入command_header[0]。 */
    command_header[1] = RA01SC_NOP; /* 计算右侧表达式，并把结果写入command_header[1]。 */
    status = RA01SC_ReadCommand(device, command_header, sizeof(command_header), status_bytes, sizeof(status_bytes)); /* 调用RA01SC_ReadCommand函数，并把返回结果保存到status。 */

    if (status != RA01SC_STATUS_OK) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return status; /* 把status作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    packet_status->rssi_dbm = -((float)status_bytes[0]) / 2.0f; /* 计算右侧表达式，并把结果写入packet_status->rssi_dbm。 */
    packet_status->snr_db = ((float)(int8_t)status_bytes[1]) / 4.0f; /* 计算右侧表达式，并把结果写入packet_status->snr_db。 */
    packet_status->signal_rssi_dbm = -((float)status_bytes[2]) / 2.0f; /* 计算右侧表达式，并把结果写入packet_status->signal_rssi_dbm。 */

    return RA01SC_STATUS_OK; /* 把RA01SC_STATUS_OK作为当前函数结果返回给调用者。 */
} /* 结束当前函数、判断、循环或数据类型的代码块。 */

/**
 * @brief 查询BUSY引脚。
 */
bool RA01SC_IsBusy(const RA01SC_Device_t *device) /* 定义函数RA01SC_IsBusy，实现该函数对应的底层操作。 */
{ /* 开始当前函数、判断、循环或数据类型的代码块。 */
    if (RA01SC_IsHardwareValid(device) == false) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return false; /* 把false作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    return HAL_GPIO_ReadPin(device->hardware.busy_port, device->hardware.busy_pin) == GPIO_PIN_SET; /* 把HAL_GPIO_ReadPin(device->hardware.busy_port, device->hardware.busy_pin) == GPIO_PIN_SET作为当前函数结果返回给调用者。 */
} /* 结束当前函数、判断、循环或数据类型的代码块。 */

/**
 * @brief 查询DIO1引脚。
 */
bool RA01SC_IsDio1High(const RA01SC_Device_t *device) /* 定义函数RA01SC_IsDio1High，实现该函数对应的底层操作。 */
{ /* 开始当前函数、判断、循环或数据类型的代码块。 */
    if (RA01SC_IsHardwareValid(device) == false) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return false; /* 把false作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    return HAL_GPIO_ReadPin(device->hardware.dio1_port, device->hardware.dio1_pin) == GPIO_PIN_SET; /* 把HAL_GPIO_ReadPin(device->hardware.dio1_port, device->hardware.dio1_pin) == GPIO_PIN_SET作为当前函数结果返回给调用者。 */
} /* 结束当前函数、判断、循环或数据类型的代码块。 */

/**
 * @brief 查询初始化状态。
 */
bool RA01SC_IsInitialized(const RA01SC_Device_t *device) /* 定义函数RA01SC_IsInitialized，实现该函数对应的底层操作。 */
{ /* 开始当前函数、判断、循环或数据类型的代码块。 */
    if (device == NULL) /* 判断本行括号中的条件是否成立。 */
    { /* 开始当前函数、判断、循环或数据类型的代码块。 */
        return false; /* 把false作为当前函数结果返回给调用者。 */
    } /* 结束当前函数、判断、循环或数据类型的代码块。 */

    return device->initialized; /* 把device->initialized作为当前函数结果返回给调用者。 */
} /* 结束当前函数、判断、循环或数据类型的代码块。 */

 