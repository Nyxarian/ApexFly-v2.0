/**
 * @file ra01sc.h
 * @brief RA-01SC（LLCC68）LoRa 无线模块底层驱动。
 *
 * 只负责芯片复位、SPI 命令、LoRa 参数、收发和中断状态。
 * 遥控协议、失联判断和电机控制不在本文件。
 * SPI/GPIO 由调用方填入 RA01SC_Hardware_t，不要在驱动里写死引脚。
 * 当前飞机端硬件：SPI1，NSS=PA4，SCK=PA5，MISO=PA6，MOSI=PA7，
 * DIO1=PB2，NRST=PE7，TXEN=PE8，RXEN=PE9，BUSY=PE10。
 */

#ifndef RA01SC_H /* 如果宏RA01SC_H尚未定义，则继续展开本头文件。 */
#define RA01SC_H /* 定义头文件保护宏RA01SC_H，防止重复包含。 */

#include "main.h" /* 引入"main.h"提供的类型、宏和函数声明。 */
#include <stdbool.h> /* 引入<stdbool.h>提供的类型、宏和函数声明。 */
#include <stdint.h> /* 引入<stdint.h>提供的类型、宏和函数声明。 */

#ifdef __cplusplus /* 判断当前文件是否正在由C++编译器处理。 */
extern "C" { /* 要求C++编译器按照C语言链接规则导出后续函数。 */
#endif /* 结束当前条件编译范围。 */

/* LLCC68内部数据缓冲区的最大长度为255字节。 */
#define RA01SC_MAX_PAYLOAD_LENGTH 255U /* 定义宏RA01SC_MAX_PAYLOAD_LENGTH，统一保存本驱动使用的固定值。 */

/* RA-01SC常用的LoRa公共网络同步字。 */
#define RA01SC_LORA_PUBLIC_SYNC_WORD 0x3444U /* 定义宏RA01SC_LORA_PUBLIC_SYNC_WORD，统一保存本驱动使用的固定值。 */

/* RA-01SC常用的LoRa私有网络同步字。 */
#define RA01SC_LORA_PRIVATE_SYNC_WORD 0x1424U /* 定义宏RA01SC_LORA_PRIVATE_SYNC_WORD，统一保存本驱动使用的固定值。 */

/**
 * @brief RA-01SC驱动函数返回状态。
 */
typedef enum /* 开始声明一个枚举类型。 */
{ /* 开始当前枚举、结构体或代码块。 */
    /* 函数执行成功。 */
    RA01SC_STATUS_OK = 0, /* 定义枚举成员RA01SC_STATUS_OK及其对应数值。 */
    /* 调用者传入了空指针、非法频率或非法LoRa参数。 */
    RA01SC_STATUS_INVALID_PARAM, /* 执行或声明本行所表示的代码内容。 */
    /* 驱动尚未完成初始化。 */
    RA01SC_STATUS_NOT_INITIALIZED, /* 执行或声明本行所表示的代码内容。 */
    /* BUSY引脚持续为高电平并超过等待时间。 */
    RA01SC_STATUS_BUSY_TIMEOUT, /* 执行或声明本行所表示的代码内容。 */
    /* HAL SPI发送或接收失败。 */
    RA01SC_STATUS_SPI_ERROR, /* 执行或声明本行所表示的代码内容。 */
    /* 复位后无法从芯片读取到合理状态。 */
    RA01SC_STATUS_CHIP_NOT_RESPONDING, /* 执行或声明本行所表示的代码内容。 */
    /* 数据长度超过LLCC68缓冲区或调用者接收缓冲区容量。 */
    RA01SC_STATUS_PACKET_TOO_LONG, /* 执行或声明本行所表示的代码内容。 */
    /* 当前没有可以读取的有效数据包。 */
    RA01SC_STATUS_NO_PACKET /* 执行或声明本行所表示的代码内容。 */
} RA01SC_Status_t; /* 结束当前类型定义，并将该类型命名为RA01SC_Status_t。 */

/**
 * @brief LLCC68的LoRa扩频因子。
 *
 * 扩频因子越大，灵敏度和距离通常越高，但空中传输时间和控制延迟也越大。
 * LLCC68支持SF5到SF11，不使用SX126x中的SF12。
 */
typedef enum /* 开始声明一个枚举类型。 */
{ /* 开始当前枚举、结构体或代码块。 */
    RA01SC_LORA_SF5 = 0x05, /* 定义枚举成员RA01SC_LORA_SF5及其对应数值。 */
    RA01SC_LORA_SF6 = 0x06, /* 定义枚举成员RA01SC_LORA_SF6及其对应数值。 */
    RA01SC_LORA_SF7 = 0x07, /* 定义枚举成员RA01SC_LORA_SF7及其对应数值。 */
    RA01SC_LORA_SF8 = 0x08, /* 定义枚举成员RA01SC_LORA_SF8及其对应数值。 */
    RA01SC_LORA_SF9 = 0x09, /* 定义枚举成员RA01SC_LORA_SF9及其对应数值。 */
    RA01SC_LORA_SF10 = 0x0A, /* 定义枚举成员RA01SC_LORA_SF10及其对应数值。 */
    RA01SC_LORA_SF11 = 0x0B /* 定义枚举成员RA01SC_LORA_SF11及其对应数值。 */
} RA01SC_LoraSpreadingFactor_t; /* 结束当前类型定义，并将该类型命名为RA01SC_LoraSpreadingFactor_t。 */

/**
 * @brief LLCC68的LoRa带宽编码。
 */
typedef enum /* 开始声明一个枚举类型。 */
{ /* 开始当前枚举、结构体或代码块。 */
    RA01SC_LORA_BW_125_KHZ = 0x04, /* 定义枚举成员RA01SC_LORA_BW_125_KHZ及其对应数值。 */
    RA01SC_LORA_BW_250_KHZ = 0x05, /* 定义枚举成员RA01SC_LORA_BW_250_KHZ及其对应数值。 */
    RA01SC_LORA_BW_500_KHZ = 0x06 /* 定义枚举成员RA01SC_LORA_BW_500_KHZ及其对应数值。 */
} RA01SC_LoraBandwidth_t; /* 结束当前类型定义，并将该类型命名为RA01SC_LoraBandwidth_t。 */

/**
 * @brief LLCC68的LoRa前向纠错编码率。
 */
typedef enum /* 开始声明一个枚举类型。 */
{ /* 开始当前枚举、结构体或代码块。 */
    RA01SC_LORA_CR_4_5 = 0x01, /* 定义枚举成员RA01SC_LORA_CR_4_5及其对应数值。 */
    RA01SC_LORA_CR_4_6 = 0x02, /* 定义枚举成员RA01SC_LORA_CR_4_6及其对应数值。 */
    RA01SC_LORA_CR_4_7 = 0x03, /* 定义枚举成员RA01SC_LORA_CR_4_7及其对应数值。 */
    RA01SC_LORA_CR_4_8 = 0x04 /* 定义枚举成员RA01SC_LORA_CR_4_8及其对应数值。 */
} RA01SC_LoraCodingRate_t; /* 结束当前类型定义，并将该类型命名为RA01SC_LoraCodingRate_t。 */

/**
 * @brief LoRa数据包头模式。
 */
typedef enum /* 开始声明一个枚举类型。 */
{ /* 开始当前枚举、结构体或代码块。 */
    /* 显式包头中携带负载长度和编码信息，适合当前遥控通信。 */
    RA01SC_LORA_HEADER_EXPLICIT = 0x00, /* 定义枚举成员RA01SC_LORA_HEADER_EXPLICIT及其对应数值。 */
    /* 隐式包头要求收发双方提前约定固定长度。 */
    RA01SC_LORA_HEADER_IMPLICIT = 0x01 /* 定义枚举成员RA01SC_LORA_HEADER_IMPLICIT及其对应数值。 */
} RA01SC_LoraHeaderType_t; /* 结束当前类型定义，并将该类型命名为RA01SC_LoraHeaderType_t。 */

/**
 * @brief LLCC68中断状态位。
 */
typedef enum /* 开始声明一个枚举类型。 */
{ /* 开始当前枚举、结构体或代码块。 */
    RA01SC_IRQ_NONE = 0x0000, /* 定义枚举成员RA01SC_IRQ_NONE及其对应数值。 */
    RA01SC_IRQ_TX_DONE = 0x0001, /* 定义枚举成员RA01SC_IRQ_TX_DONE及其对应数值。 */
    RA01SC_IRQ_RX_DONE = 0x0002, /* 定义枚举成员RA01SC_IRQ_RX_DONE及其对应数值。 */
    RA01SC_IRQ_PREAMBLE_DETECTED = 0x0004, /* 定义枚举成员RA01SC_IRQ_PREAMBLE_DETECTED及其对应数值。 */
    RA01SC_IRQ_SYNC_WORD_VALID = 0x0008, /* 定义枚举成员RA01SC_IRQ_SYNC_WORD_VALID及其对应数值。 */
    RA01SC_IRQ_HEADER_VALID = 0x0010, /* 定义枚举成员RA01SC_IRQ_HEADER_VALID及其对应数值。 */
    RA01SC_IRQ_HEADER_ERROR = 0x0020, /* 定义枚举成员RA01SC_IRQ_HEADER_ERROR及其对应数值。 */
    RA01SC_IRQ_CRC_ERROR = 0x0040, /* 定义枚举成员RA01SC_IRQ_CRC_ERROR及其对应数值。 */
    RA01SC_IRQ_CAD_DONE = 0x0080, /* 定义枚举成员RA01SC_IRQ_CAD_DONE及其对应数值。 */
    RA01SC_IRQ_CAD_DETECTED = 0x0100, /* 定义枚举成员RA01SC_IRQ_CAD_DETECTED及其对应数值。 */
    RA01SC_IRQ_TIMEOUT = 0x0200, /* 定义枚举成员RA01SC_IRQ_TIMEOUT及其对应数值。 */
    RA01SC_IRQ_ALL = 0x03FF /* 定义枚举成员RA01SC_IRQ_ALL及其对应数值。 */
} RA01SC_IrqMask_t; /* 结束当前类型定义，并将该类型命名为RA01SC_IrqMask_t。 */

/**
 * @brief 与具体PCB连接相关的SPI和GPIO信息。
 *
 * 将引脚放入结构体而不是写死在驱动内部，可以让同一套驱动同时用于
 * STM32G474飞机端和STM32F407遥控器端。
 */
typedef struct /* 开始声明一个结构体类型。 */
{ /* 开始当前枚举、结构体或代码块。 */
    SPI_HandleTypeDef *spi; /* 执行或声明本行所表示的代码内容。 */
    GPIO_TypeDef *nss_port; /* 执行或声明本行所表示的代码内容。 */
    uint16_t nss_pin; /* 声明结构体成员nss_pin，用于保存该字段对应的数据。 */
    GPIO_TypeDef *reset_port; /* 执行或声明本行所表示的代码内容。 */
    uint16_t reset_pin; /* 声明结构体成员reset_pin，用于保存该字段对应的数据。 */
    GPIO_TypeDef *busy_port; /* 执行或声明本行所表示的代码内容。 */
    uint16_t busy_pin; /* 声明结构体成员busy_pin，用于保存该字段对应的数据。 */
    GPIO_TypeDef *dio1_port; /* 执行或声明本行所表示的代码内容。 */
    uint16_t dio1_pin; /* 声明结构体成员dio1_pin，用于保存该字段对应的数据。 */
    GPIO_TypeDef *txen_port; /* 执行或声明本行所表示的代码内容。 */
    uint16_t txen_pin; /* 声明结构体成员txen_pin，用于保存该字段对应的数据。 */
    GPIO_TypeDef *rxen_port; /* 执行或声明本行所表示的代码内容。 */
    uint16_t rxen_pin; /* 声明结构体成员rxen_pin，用于保存该字段对应的数据。 */
} RA01SC_Hardware_t; /* 结束当前类型定义，并将该类型命名为RA01SC_Hardware_t。 */

/**
 * @brief LoRa空中通信参数。
 *
 * 飞机端和遥控器端的所有字段必须完全一致，否则无法互相解调数据。
 */
typedef struct /* 开始声明一个结构体类型。 */
{ /* 开始当前枚举、结构体或代码块。 */
    uint32_t frequency_hz; /* 声明结构体成员frequency_hz，用于保存该字段对应的数据。 */
    RA01SC_LoraSpreadingFactor_t spreading_factor; /* 声明结构体成员spreading_factor，用于保存该字段对应的数据。 */
    RA01SC_LoraBandwidth_t bandwidth; /* 声明结构体成员bandwidth，用于保存该字段对应的数据。 */
    RA01SC_LoraCodingRate_t coding_rate; /* 声明结构体成员coding_rate，用于保存该字段对应的数据。 */
    uint16_t preamble_length; /* 声明结构体成员preamble_length，用于保存该字段对应的数据。 */
    RA01SC_LoraHeaderType_t header_type; /* 声明结构体成员header_type，用于保存该字段对应的数据。 */
    uint8_t max_payload_length; /* 声明结构体成员max_payload_length，用于保存该字段对应的数据。 */
    bool crc_enabled; /* 声明结构体成员crc_enabled，用于保存该字段对应的数据。 */
    bool invert_iq; /* 声明结构体成员invert_iq，用于保存该字段对应的数据。 */
    int8_t tx_power_dbm; /* 声明结构体成员tx_power_dbm，用于保存该字段对应的数据。 */
    uint16_t sync_word; /* 声明结构体成员sync_word，用于保存该字段对应的数据。 */
    uint32_t tx_timeout_ms; /* 声明结构体成员tx_timeout_ms，用于保存该字段对应的数据。 */
} RA01SC_Config_t; /* 结束当前类型定义，并将该类型命名为RA01SC_Config_t。 */

/**
 * @brief 最近一次接收数据包的射频质量。
 */
typedef struct /* 开始声明一个结构体类型。 */
{ /* 开始当前枚举、结构体或代码块。 */
    float rssi_dbm; /* 声明结构体成员rssi_dbm，用于保存该字段对应的数据。 */
    float snr_db; /* 声明结构体成员snr_db，用于保存该字段对应的数据。 */
    float signal_rssi_dbm; /* 声明结构体成员signal_rssi_dbm，用于保存该字段对应的数据。 */
} RA01SC_PacketStatus_t; /* 结束当前类型定义，并将该类型命名为RA01SC_PacketStatus_t。 */

/**
 * @brief RA-01SC设备对象。
 */
typedef struct /* 开始声明一个结构体类型。 */
{ /* 开始当前枚举、结构体或代码块。 */
    RA01SC_Hardware_t hardware; /* 声明结构体成员hardware，用于保存该字段对应的数据。 */
    RA01SC_Config_t config; /* 声明结构体成员config，用于保存该字段对应的数据。 */
    bool initialized; /* 声明结构体成员initialized，用于保存该字段对应的数据。 */
} RA01SC_Device_t; /* 结束当前类型定义，并将该类型命名为RA01SC_Device_t。 */

/**
 * @brief 填充一套适合台架通信测试的默认LoRa参数。
 * @param config 用于保存默认参数的结构体地址。
 */
void RA01SC_GetDefaultConfig(RA01SC_Config_t *config); /* 声明函数RA01SC_GetDefaultConfig，供其他源文件调用。 */

/**
 * @brief 复位并初始化RA-01SC，然后写入LoRa通信参数。
 * @param device 设备对象地址。
 * @param hardware SPI和GPIO连接信息。
 * @param config LoRa通信参数。
 * @return 驱动执行状态。
 */
RA01SC_Status_t RA01SC_Init(RA01SC_Device_t *device, const RA01SC_Hardware_t *hardware, const RA01SC_Config_t *config); /* 声明函数RA01SC_Init，供其他源文件调用。 */

/**
 * @brief 对RA-01SC执行一次硬件复位。
 * @param device 已绑定硬件信息的设备对象。
 * @return 驱动执行状态。
 */
RA01SC_Status_t RA01SC_Reset(RA01SC_Device_t *device); /* 声明函数RA01SC_Reset，供其他源文件调用。 */

/**
 * @brief 重新向芯片写入device中保存的全部LoRa参数。
 * @param device 已绑定硬件信息的设备对象。
 * @return 驱动执行状态。
 */
RA01SC_Status_t RA01SC_ApplyConfig(RA01SC_Device_t *device); /* 声明函数RA01SC_ApplyConfig，供其他源文件调用。 */

/**
 * @brief 读取LLCC68原始芯片状态字节。
 * @param device 设备对象地址。
 * @param chip_status 用于保存状态字节的变量地址。
 * @return 驱动执行状态。
 */
RA01SC_Status_t RA01SC_GetChipStatus(RA01SC_Device_t *device, uint8_t *chip_status); /* 声明函数RA01SC_GetChipStatus，供其他源文件调用。 */

/**
 * @brief 让LLCC68和外部射频开关进入待机状态。
 * @param device 已初始化设备对象。
 * @return 驱动执行状态。
 */
RA01SC_Status_t RA01SC_SetStandby(RA01SC_Device_t *device); /* 声明函数RA01SC_SetStandby，供其他源文件调用。 */

/**
 * @brief 进入LoRa连续接收模式。
 * @param device 已初始化设备对象。
 * @return 驱动执行状态。
 */
RA01SC_Status_t RA01SC_StartReceive(RA01SC_Device_t *device); /* 声明函数RA01SC_StartReceive，供其他源文件调用。 */

/**
 * @brief 启动一次非阻塞LoRa发送。
 *
 * 函数只负责把数据写入芯片并启动发送。发送完成后DIO1产生中断，
 * APP层应读取RA01SC_IRQ_TX_DONE，再重新调用RA01SC_StartReceive()。
 *
 * @param device 已初始化设备对象。
 * @param data 要发送的数据地址。
 * @param length 数据长度，范围为1到255字节。
 * @return 驱动执行状态。
 */
RA01SC_Status_t RA01SC_Send(RA01SC_Device_t *device, const uint8_t *data, uint8_t length); /* 声明函数RA01SC_Send，供其他源文件调用。 */

/**
 * @brief 读取并清除LLCC68当前全部中断状态。
 * @param device 已初始化设备对象。
 * @param irq_status 用于保存RA01SC_IrqMask_t位组合的变量地址。
 * @return 驱动执行状态。
 */
RA01SC_Status_t RA01SC_GetAndClearIrqStatus(RA01SC_Device_t *device, uint16_t *irq_status); /* 声明函数RA01SC_GetAndClearIrqStatus，供其他源文件调用。 */

/**
 * @brief 从LLCC68接收缓冲区读取最新数据包。
 * @param device 已初始化设备对象。
 * @param data 调用者提供的接收缓冲区。
 * @param capacity 接收缓冲区容量。
 * @param length 用于返回实际数据长度。
 * @return 驱动执行状态。
 */
RA01SC_Status_t RA01SC_ReadPacket(RA01SC_Device_t *device, uint8_t *data, uint8_t capacity, uint8_t *length); /* 声明函数RA01SC_ReadPacket，供其他源文件调用。 */

/**
 * @brief 读取最近一次LoRa数据包的RSSI和SNR。
 * @param device 已初始化设备对象。
 * @param packet_status 用于保存射频质量的结构体地址。
 * @return 驱动执行状态。
 */
RA01SC_Status_t RA01SC_GetPacketStatus(RA01SC_Device_t *device, RA01SC_PacketStatus_t *packet_status); /* 声明函数RA01SC_GetPacketStatus，供其他源文件调用。 */

/**
 * @brief 判断LLCC68的BUSY引脚当前是否为高电平。
 * @param device 已绑定硬件信息的设备对象。
 * @return true表示芯片忙，false表示可以发送新命令。
 */
bool RA01SC_IsBusy(const RA01SC_Device_t *device); /* 声明函数RA01SC_IsBusy，供其他源文件调用。 */

/**
 * @brief 判断DIO1引脚当前是否为高电平。
 * @param device 已绑定硬件信息的设备对象。
 * @return DIO1引脚逻辑状态。
 */
bool RA01SC_IsDio1High(const RA01SC_Device_t *device); /* 声明函数RA01SC_IsDio1High，供其他源文件调用。 */

/**
 * @brief 查询设备是否已完成初始化。
 * @param device 设备对象地址。
 * @return 初始化状态。
 */
bool RA01SC_IsInitialized(const RA01SC_Device_t *device); /* 声明函数RA01SC_IsInitialized，供其他源文件调用。 */

#ifdef __cplusplus /* 判断当前文件是否正在由C++编译器处理。 */
} /* 结束当前代码块。 */
#endif /* 结束当前条件编译范围。 */

#endif /* 结束当前条件编译范围。 */

