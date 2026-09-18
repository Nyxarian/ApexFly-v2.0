/**
 * @file apex_config.h
 * @brief 手写的编译期开关。**不是 CubeMX 生成的**，可以放心改。
 *
 * 为什么单独开这个文件：飞控里有一批"选哪个传感器 / 用哪个口"的决定，
 * 它们既不属于某一个模块，也不该散落在各自的 #define 里。
 * 集中在这里，切换只改一个数，回退也只改一个数。
 */

#ifndef APEX_CONFIG_H
#define APEX_CONFIG_H

/* ── USART2 归谁用 ──────────────────────────────────────────────
   USART2（PA2/PA3 + DMA1_Channel1）原本是 GPS(ATGM336H) 的口。
   本板没留多余串口，而光流模块"只发不收"、只需要一个 RX 脚，
   所以在定点/定高做稳之前先把 GPS 摘下来，光流接到同一个座上
   （完整论证见 `优化一下/光流接入方案.md`）。

     1 = 光流 CX-OF（19200）
     0 = GPS ATGM336H（9600）

   ⚠️ 这个开关同时管三处，改一个数就够，但要知道是哪三处：
     ① 本文件的 APEX_UART2_BAUD —— 被 `Core/Src/usart.c` 的 huart2 波特率使用；
     ② `Modules/Flow/flow.c`    —— 光流任务与 USART2 的两个 HAL 回调（=1 时才编译）；
     ③ `Modules/GPS/gps.c`      —— GPS 任务与 USART2 的两个 HAL 回调（=0 时才编译）。
   ⚠️ ②③ 里的 HAL 回调**必须互斥**：两边同时定义会报
      `L6200E: Symbol HAL_UART_ErrorCallback multiply defined`
      —— 2026-09 做串口回显测试时，同一个符号已经踩过一次，别再来第二次。
   ⚠️ GPS 那条代码路径**故意不删**：第二阶段把光流挪到打印口的 RX 脚、
      GPS 接回 USART2 做返航时，只需要把这个数改回 0。 */
#define APEX_UART2_ROLE_FLOW 1

#if APEX_UART2_ROLE_FLOW
#define APEX_UART2_BAUD 19200U
#else
#define APEX_UART2_BAUD 9600U
#endif

#endif /* APEX_CONFIG_H */
