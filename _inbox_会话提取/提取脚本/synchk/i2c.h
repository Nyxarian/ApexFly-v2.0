#ifndef STUB_I2C_H
#define STUB_I2C_H
#include "stm32g4xx_hal.h"
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
void I2C2_Lock(void);
void I2C2_Unlock(void);
#endif
