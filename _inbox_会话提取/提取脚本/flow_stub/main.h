/* 仅用于宿主机 gcc 仿真的桩：顶掉 CubeMX 的聚合头 main.h。
   flow.c 会经 usart.h 用到 UART_HandleTypeDef / DMA_HandleTypeDef，
   所以这里必须把真正的 HAL 头引进来（只声明、不链接任何 HAL .c）。
   ⚠️ 本目录只加进仿真编译命令，绝不能加进 Keil 的包含路径。 */
#ifndef SIM_FLOW_MAIN_H
#define SIM_FLOW_MAIN_H

#include "stm32g4xx_hal.h"

#endif
