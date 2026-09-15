/**
 * @file serial_echo.h
 * @brief 【临时】USART3 透明回显：收到的每个字节原样发回去。
 *
 * 用途：验证上位机串口工具的收发回路是否正常。
 * 测完把本模块整个删掉即可，恢复清单见 `优化一下/串口回显测试（临时）.md`。
 *
 * ── 为什么必须走中断，不能沿用现有的 5ms 轮询 ──────────────────
 * 115200 8N1 每字节只有 86.8µs，而 USART3 的 FIFO 是关掉的（硬件只存 1 个字节）。
 * 现有收数据靠黑匣子任务每 5ms 轮询一次 RDR，5ms 能来 57 个字节、只能收到 1 个。
 * 任何轮询点只要被抢占几百微秒（250Hz 控制环每 4ms 必然跑一次），这期间到的字节就没了。
 * 所以这里开接收中断：ISR 只做"1 个字节进环形缓冲 + 通知任务"，回显任务慢慢往外发。
 *
 * ── 本模块临时占用的资源（测完都要还回去）────────────────────
 * 1. **USART3 的接收中断由本模块独占**：测试期间黑匣子的串口命令
 *    （`dump` / `dumpall` / `calmag` / `vm`）不可用；
 * 2. `USART3_IRQHandler` 由本模块实现（启动文件里是 `[WEAK]`，直接覆盖），
 *    所以不必改 CubeMX 生成的 `stm32g4xx_it.c`；
 * 3. NVIC 使能在 `usart.c` 的 `USER CODE BEGIN USART3_MspInit 1` 里加，
 *    任务是 CubeMX 重新生成也不会被覆盖的用户区。
 *
 * ── 和别的串口互不影响 ───────────────────────────────────────
 * GPS 走 USART2 + `HAL_UARTEx_RxEventCallback`，是另一条通道；
 * 本模块只用 USART3 的 `HAL_UART_RxCpltCallback`，并且按实例号过滤。
 */

#ifndef SERIAL_ECHO_H
#define SERIAL_ECHO_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 【临时】串口回显任务入口。
 *
 * 阻塞等接收通知，排空环形缓冲后原样发回，一直循环。
 * 由 `MX_FREERTOS_Init()` 创建，优先级 BelowNormal（和 tof/黑匣子同级），栈 1KB。
 *
 * @param argument 未使用。
 */
void SerialEcho_Task(void *argument);

#ifdef __cplusplus
}
#endif

#endif
