/* 桩：只为了在 PC 上对 motor_test.c / tof.c 做语法与类型自检，不参与真实编译。
   这里只补"名字"和"类型"，不求行为正确——目的是让编译器把语法和类型查完。 */
#ifndef STUB_STM32G4XX_HAL_H
#define STUB_STM32G4XX_HAL_H
#include <stdint.h>
#include <stddef.h>

typedef enum { HAL_OK = 0, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef struct { void *Instance; } I2C_HandleTypeDef;
typedef struct { void *Instance; } SPI_HandleTypeDef;
typedef struct { volatile uint32_t ISR, RDR, TDR, BRR, CR1, CR2, CR3, GTPR; } USART_TypeDef_stub;
typedef struct { USART_TypeDef_stub *Instance; } UART_HandleTypeDef;
int HAL_UARTEx_EnableFifoMode(void *huart);
typedef struct { void *Instance; } TIM_HandleTypeDef;
typedef struct { void *Instance; } ADC_HandleTypeDef;
typedef struct { void *Instance; } GPIO_TypeDef;

/* GPIO 端口与引脚：main.h 里的 LORA_NSS_GPIO_Port 这类宏要用到。 */
#define GPIOA ((GPIO_TypeDef *)0)
#define GPIOB ((GPIO_TypeDef *)0)
#define GPIOC ((GPIO_TypeDef *)0)
#define GPIOD ((GPIO_TypeDef *)0)
#define GPIOE ((GPIO_TypeDef *)0)
#define GPIO_PIN_0 0x0001U
#define GPIO_PIN_1 0x0002U
#define GPIO_PIN_2 0x0004U
#define GPIO_PIN_3 0x0008U
#define GPIO_PIN_4 0x0010U
#define GPIO_PIN_5 0x0020U
#define GPIO_PIN_6 0x0040U
#define GPIO_PIN_7 0x0080U
#define GPIO_PIN_8 0x0100U
#define GPIO_PIN_9 0x0200U
#define GPIO_PIN_10 0x0400U
#define GPIO_PIN_11 0x0800U
#define GPIO_PIN_12 0x1000U
#define GPIO_PIN_13 0x2000U
#define GPIO_PIN_14 0x4000U
#define GPIO_PIN_15 0x8000U

/* UART 标志位读取：blackbox.c 轮询 ORE 用。 */
#define RESET 0U
#define UART_FLAG_ORE 0x00000008U
#define UART_CLEAR_OREF 0x00000008U
#define __HAL_UART_GET_FLAG(handle, flag) (0U)
#define __HAL_UART_CLEAR_FLAG(handle, flag) ((void)0)

/* CMSIS 内核里的东西（DWT 计时用） */
typedef struct { volatile uint32_t DHCSR, DCRSR, DCRDR, DEMCR; } CoreDebug_Type;
extern CoreDebug_Type CoreDebug_stub;
#define CoreDebug (&CoreDebug_stub)
#define CoreDebug_DEMCR_TRCENA_Msk (1UL << 24)

typedef struct { volatile uint32_t CTRL, CYCCNT, RESERVED0; } DWT_Type;
extern DWT_Type DWT_stub;
#define DWT (&DWT_stub)
#define DWT_CTRL_CYCCNTENA_Msk (1UL << 0)

extern uint32_t SystemCoreClock;
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t ms);
#endif
