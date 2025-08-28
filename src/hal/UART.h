#ifndef _UART_H_
#define _UART_H_

#include "stm32f10x.h"
#include <stdio.h>

typedef enum{
    iUSART1,
    iUSART2,
    iUSART3
} iUSART_t;
typedef enum{
    USART_MODE_BASIC,
    USART_MODE_HALF_DUPLEX
} USART_Mode_t;
typedef void(* USART_Callback_t)(void);

void iUSART_Init(iUSART_t USART, USART_Mode_t mode);
void USART_RegisterCallback(iUSART_t USART, USART_Callback_t callback);
void USART_Printf(USART_TypeDef* USARTx, const char* format, ...);

#endif
