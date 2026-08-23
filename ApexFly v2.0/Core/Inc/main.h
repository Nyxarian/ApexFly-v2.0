/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_BLUE_Pin GPIO_PIN_9
#define LED_BLUE_GPIO_Port GPIOF
#define LED_RED_Pin GPIO_PIN_10
#define LED_RED_GPIO_Port GPIOF
#define LORA_NSS_Pin GPIO_PIN_4
#define LORA_NSS_GPIO_Port GPIOA
#define LORA_SCK_Pin GPIO_PIN_5
#define LORA_SCK_GPIO_Port GPIOA
#define LORA_MISO_Pin GPIO_PIN_6
#define LORA_MISO_GPIO_Port GPIOA
#define LORA_MOSI_Pin GPIO_PIN_7
#define LORA_MOSI_GPIO_Port GPIOA
#define IMU_INT_Pin GPIO_PIN_0
#define IMU_INT_GPIO_Port GPIOB
#define IMU_INT_EXTI_IRQn EXTI0_IRQn
#define LORA_DIO1_Pin GPIO_PIN_2
#define LORA_DIO1_GPIO_Port GPIOB
#define LORA_DIO1_EXTI_IRQn EXTI2_IRQn
#define LORA_NRST_Pin GPIO_PIN_7
#define LORA_NRST_GPIO_Port GPIOE
#define LORA_TXEN_Pin GPIO_PIN_8
#define LORA_TXEN_GPIO_Port GPIOE
#define LORA_RXEN_Pin GPIO_PIN_9
#define LORA_RXEN_GPIO_Port GPIOE
#define LORA_BUSY_Pin GPIO_PIN_10
#define LORA_BUSY_GPIO_Port GPIOE
#define W_CS_Pin GPIO_PIN_6
#define W_CS_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
