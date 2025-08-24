#ifndef _TIM_H_
#define _TIM_H_

#include "stm32f10x.h"

#define TIM3_PERIOD 10000-1
#define TIM3_PRESCALER 7200-1

void Timer_Init(void);
void TIM3_RegisterCallback(void(* callback)(void));

#endif
