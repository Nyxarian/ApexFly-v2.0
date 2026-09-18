/* 仅用于宿主机 gcc 仿真的桩：顶掉 FreeRTOS 的 task.h。
   flow.c 只用到临界区与让出 CPU 这三个宏。
   ⚠️ taskYIELD 故意**不**做成空操作：仿真要靠它驱动下一步（见 sim_flow.c 的说明）。 */
#ifndef SIM_TASK_H
#define SIM_TASK_H

void Sim_TaskYield(void);

#define taskENTER_CRITICAL() do { } while (0)
#define taskEXIT_CRITICAL()  do { } while (0)
#define taskYIELD()          Sim_TaskYield()

#endif
