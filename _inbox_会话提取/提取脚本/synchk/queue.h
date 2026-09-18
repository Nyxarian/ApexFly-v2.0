/* 桩：blackbox.c 用到 FreeRTOS 的队列，这里只补名字让编译器把类型查完。 */
#ifndef STUB_QUEUE_H
#define STUB_QUEUE_H
#include "FreeRTOS.h"
QueueHandle_t xQueueCreateStatic(unsigned long length, unsigned long item_size, void *storage, StaticQueue_t *static_queue);
int xQueueSend(QueueHandle_t queue, const void *item, TickType_t wait);
int xQueueReceive(QueueHandle_t queue, void *item, TickType_t wait);
int xQueueReset(QueueHandle_t queue);
unsigned long uxQueueMessagesWaiting(QueueHandle_t queue);
#endif
