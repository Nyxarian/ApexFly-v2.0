/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
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

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "debug.h"
#include "i2c.h"
#include "imu_attitude.h"
#include "imu_bias.h"
#include "led.h"
#include "motor_test.h"
#include "mpu6050.h"
#include "blackbox.h"
#include "gps.h"
#include "i2c2_lock.h"
#include "mag.h"
#include "radio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for motorTestTask */
osThreadId_t motorTestTaskHandle;
const osThreadAttr_t motorTestTask_attributes = {
  .name = "motorTestTask",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 640 * 4
};
/* Definitions for ledTask */
osThreadId_t ledTaskHandle;
const osThreadAttr_t ledTask_attributes = {
  .name = "ledTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for radioTask */
osThreadId_t radioTaskHandle;
const osThreadAttr_t radioTask_attributes = {
  .name = "radioTask",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 512 * 4
};
/* Definitions for blackboxTask */
osThreadId_t blackboxTaskHandle;
const osThreadAttr_t blackboxTask_attributes = {
  .name = "blackboxTask",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = 768 * 4
};
/* Definitions for magtask */
osThreadId_t magtaskHandle;
const osThreadAttr_t magtask_attributes = {
  .name = "magtask",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = 512 * 4
};
/* Definitions for gpsTask */
osThreadId_t gpsTaskHandle;
const osThreadAttr_t gpsTask_attributes = {
  .name = "gpsTask",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = 512 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartMotorTestTask(void *argument);
void StartLedTask(void *argument);
void StartRadioTask(void *argument);
void StartBlackboxTask(void *argument);
void StartMagTask(void *argument);
void StartGpsTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* 栈溢出时停住调度，并让红灯常亮、蓝灯熄灭，方便面板直接看出异常。 */
   (void)xTask;
   (void)pcTaskName;
   taskDISABLE_INTERRUPTS();
   HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);
   HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
   for (;;)
   {
   }
}
/* USER CODE END 4 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  I2C2_Lock_Init();
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of motorTestTask */
  motorTestTaskHandle = osThreadNew(StartMotorTestTask, NULL, &motorTestTask_attributes);

  /* creation of ledTask */
  ledTaskHandle = osThreadNew(StartLedTask, NULL, &ledTask_attributes);

  /* creation of radioTask */
  radioTaskHandle = osThreadNew(StartRadioTask, NULL, &radioTask_attributes);

  /* creation of blackboxTask */
  blackboxTaskHandle = osThreadNew(StartBlackboxTask, NULL, &blackboxTask_attributes);

  /* creation of magtask */
  magtaskHandle = osThreadNew(StartMagTask, NULL, &magtask_attributes);

  /* creation of gpsTask */
  gpsTaskHandle = osThreadNew(StartGpsTask, NULL, &gpsTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  (void)argument;
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartMotorTestTask */
/**
  * @brief  遥控油门跟杆任务。
  *         电调解锁并采完零偏后，按 control_link 快照输出油门。
  *         测试前必须拆桨。
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartMotorTestTask */
void StartMotorTestTask(void *argument)
{
  /* USER CODE BEGIN StartMotorTestTask */
  MotorTest_Task(argument);
  /* USER CODE END StartMotorTestTask */
}

/* USER CODE BEGIN Header_StartLedTask */
/**
  * @brief  LED 心跳任务。
  *         蓝灯 PF9 和红灯 PF10 每秒翻转一次，用来确认 FreeRTOS 正在运行。
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartLedTask */
void StartLedTask(void *argument)
{
  /* USER CODE BEGIN StartLedTask */
  Led_Task(argument);
  /* USER CODE END StartLedTask */
}

/* USER CODE BEGIN Header_StartRadioTask */
/**
  * @brief  LoRa 电台任务。
  *         收遥控控制帧，写入 control_link，立刻回遥测。不写 TIM8。
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartRadioTask */
void StartRadioTask(void *argument)
{
  /* USER CODE BEGIN StartRadioTask */
  Radio_Task(argument);
  /* USER CODE END StartRadioTask */
}

/* USER CODE BEGIN Header_StartBlackboxTask */
/**
* @brief Function implementing the blackboxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBlackboxTask */
void StartBlackboxTask(void *argument)
{
  /* USER CODE BEGIN StartBlackboxTask */
  Blackbox_Task(argument);
  /* USER CODE END StartBlackboxTask */
}

/* USER CODE BEGIN Header_StartMagTask */
/**
  * @brief  磁力计任务。I2C2 读 GY-273，只发布磁场快照，不进姿态环。
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartMagTask */
void StartMagTask(void *argument)
{
  /* USER CODE BEGIN StartMagTask */
  Mag_Task(argument);
  /* USER CODE END StartMagTask */
}

/* USER CODE BEGIN Header_StartGpsTask */
/**
  * @brief  GPS 任务。独占 USART2 上的 ATGM336H，解析 NMEA，给黑匣子提供 UTC 时间。
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartGpsTask */
void StartGpsTask(void *argument)
{
  /* USER CODE BEGIN StartGpsTask */
  GPS_Task(argument);
  /* USER CODE END StartGpsTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

