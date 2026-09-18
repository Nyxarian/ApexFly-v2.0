/* 语法检查专用的覆盖版头：把 USART2 判给 GPS（role = 0），
   用来验证"切回 GPS"那条路径也能编译过。
   放在 -I 的第一位，让 #include "apex_config.h" 先找到它。
   不参与固件构建，也不进版本库。 */
#ifndef APEX_CONFIG_H
#define APEX_CONFIG_H

#define APEX_UART2_ROLE_FLOW 0
#define APEX_UART2_BAUD 9600U

#endif
