#ifndef _SYSTEM_MANAGER_H_
#define _SYSTEM_MANAGER_H_

#include "stm32f10x.h"
#include <stdbool.h>
// Hardware Abstraction Layer
#include "UART.h"
#include "TIM.h"
#include "EXTI.h"
#include "GPIO_I2C.h"
#include "sysTick.h"
#include "DWT.h"
// Driver Layer

// Service Layer
#include "LOG_s.h"
#include "Delay_s.h"
// Application Layer


// 系统状态定义
typedef enum{
    SYS_STATE_INIT,
    SYS_STATE_READY,
    SYS_STATE_RUNNING,
    SYS_STATE_ERROR
} SysState_t;
// 自检项定义
typedef enum{
    // Hal
    SYS_CHECK_USART,
    SYS_CHECK_TIMER,
    SYS_CHECK_EXTI,
    SYS_CHECK_I2C,
    SYS_CHECK_SYSTICK,
    SYS_CHECK_DWT,
    // Drvl

    // Srvl

    // Apl
    SYS_CHECK_MAX
} SysCheck_t;

// 接口函数声明
void SysManager_Init(void);
void SysManager_Process(void);
bool System_Is_Ready(void);

#endif
