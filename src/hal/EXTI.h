#ifndef _EXTI_H_
#define _EXTI_H_

#include "stm32f10x.h"

void EXTI_Init(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin_x);

#endif
