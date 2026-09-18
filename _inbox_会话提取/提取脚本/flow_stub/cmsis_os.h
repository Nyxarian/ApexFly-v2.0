/* 仅用于宿主机 gcc 仿真的桩：顶掉 CMSIS-RTOS2 的 cmsis_os.h。
   flow.c 只用到这两个接口，其余一概不需要。 */
#ifndef SIM_CMSIS_OS_H
#define SIM_CMSIS_OS_H

#include <stdint.h>

/* 仿真里由 sim_flow.c 提供：返回模拟时钟，并驱动下一步（见 sim_flow.c 的说明）。 */
uint32_t osKernelGetTickCount(void);
void osDelay(uint32_t ticks);

#endif
