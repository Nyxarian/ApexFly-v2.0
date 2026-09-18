#ifndef STUB_DEBUG_H
#define STUB_DEBUG_H
#include <stdint.h>
int Debug_Printf(const char *fmt, ...);
int Debug_Send(const uint8_t *data, uint16_t length);
#endif
