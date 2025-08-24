#ifndef _main_h_
#define _main_h_

#include "stm32f10x.h"
// Hardware Abstraction Layer
#include "UART.h"
#include "TIM.h"
#include "GPIO_I2C.h"
#include "sysTick.h"
#include "DWT.h"
// Driver Layer
#include "Delay_d.h"
// Service Layer
#include "LOG_s.h"
// Application Layer

// Volatile Flags Definition
volatile uint8_t flag1 = 0;

// Function
void Board_Init(void);

#endif
