#ifndef STUB_SEMPHR_H
#define STUB_SEMPHR_H
#include "FreeRTOS.h"
SemaphoreHandle_t xSemaphoreCreateBinaryStatic(StaticSemaphore_t *pxStaticSemaphore);
BaseType_t xSemaphoreGiveFromISR(SemaphoreHandle_t xSemaphore, BaseType_t *pxHigherPriorityTaskWoken);
BaseType_t xSemaphoreTake(SemaphoreHandle_t xSemaphore, TickType_t xBlockTime);
#endif
