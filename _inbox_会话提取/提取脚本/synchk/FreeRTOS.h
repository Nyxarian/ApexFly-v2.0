/* 桩：只为在 PC 上对 Modules 里的 .c 做语法/类型自检，不参与真实编译。
   这里只补"名字"和"类型"，不求行为正确——目的是让编译器把语法和类型查完。 */
#ifndef STUB_FREERTOS_H
#define STUB_FREERTOS_H
#include <stdint.h>
#include <stddef.h>
typedef long BaseType_t;
typedef unsigned long UBaseType_t;
typedef uint32_t TickType_t;
typedef void *TaskHandle_t;
typedef void *QueueHandle_t;
typedef void *SemaphoreHandle_t;
typedef struct { uint32_t dummy; } StaticTask_t;
typedef struct { uint32_t dummy; } StaticQueue_t;
typedef struct { uint32_t dummy; } StaticSemaphore_t;
#define pdFALSE 0
#define pdTRUE 1
#define pdPASS 1
#define pdMS_TO_TICKS(x) (x)
#define portMAX_DELAY 0xFFFFFFFFUL
void vTaskDelay(TickType_t ticks);
TickType_t xTaskGetTickCount(void);
void taskENTER_CRITICAL(void);
void taskEXIT_CRITICAL(void);
void portYIELD_FROM_ISR(BaseType_t x);
TaskHandle_t xTaskGetCurrentTaskHandle(void);
BaseType_t xTaskNotifyWait(uint32_t clear_on_entry, uint32_t clear_on_exit, uint32_t *value, TickType_t wait);
BaseType_t xTaskNotifyFromISR(TaskHandle_t task, uint32_t value, int action, BaseType_t *woken);
#define eSetBits 1
#endif
