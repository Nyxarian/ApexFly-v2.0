#ifndef STUB_CMSIS_OS_H
#define STUB_CMSIS_OS_H
#include <stdint.h>
uint32_t osKernelGetTickCount(void);
void osDelay(uint32_t ticks);
#endif
