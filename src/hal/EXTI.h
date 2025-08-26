#ifndef _EXTI_H_
#define _EXTI_H_

#include "stm32f10x.h"

typedef void(* EXTI_Callback_t)(void);

void iEXTI_Init(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin_x, GPIOMode_TypeDef mode, EXTITrigger_TypeDef trigger);
void EXTI_RegisterCallback(uint32_t EXTI_Line, EXTI_Callback_t callback);

#endif
