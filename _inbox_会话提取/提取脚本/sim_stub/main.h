/* 仅用于宿主机 gcc 仿真的桩，顶掉 HAL 的聚合头 main.h。
   motor.h 只需要能 include 到它，不需要其中任何内容。
   ⚠️ 本目录只加进仿真编译命令，绝不能加进 Keil 的包含路径。 */
#ifndef SIM_STUB_MAIN_H
#define SIM_STUB_MAIN_H
#endif
