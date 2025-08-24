#ifndef _sysTick_h_
#define _sysTick_h_

#include "stm32f10x.h"

typedef uint32_t sysTick_t;

void sysTick_Init(void);
sysTick_t sysTick_GetMs(void);
sysTick_t sysTick_GetS(void);
uint8_t sysTick_IsTimeout(sysTick_t start_time, uint32_t timeout_ms);

#endif
