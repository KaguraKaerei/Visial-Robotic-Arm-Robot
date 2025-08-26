#ifndef _Delay_s_h_
#define _Delay_s_h_

#include "stm32f10x.h"
#include "sysTick.h"
#include "DWT.h"

void Delay_us(uint32_t us);
void Delay_ms(uint32_t ms);
void Delay_s(uint32_t s);
uint8_t DWT_DelayUs(uint32_t* start_time, uint32_t interval_us);
uint8_t sysTick_DelayMs(sysTick_t* start_time, uint32_t interval_ms);

#endif
