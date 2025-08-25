#ifndef _TIM_H_
#define _TIM_H_

#include "stm32f10x.h"

typedef enum{
    iTIM1,
    iTIM2,
    iTIM3,
    iTIM4
} iTIM_t;
typedef enum{
    TIM_MODE_BASIC,
    TIM_MODE_PWM,
    TIM_MODE_IC,
    TIM_MODE_ENCODER
} TIM_Mode_t;
typedef void(* TIM_Callback_t)(void);

void Timer_Init(iTIM_t TIM, TIM_Mode_t mode);
void TIM_RegisterCallback(iTIM_t TIM, TIM_Callback_t callback);

#endif
