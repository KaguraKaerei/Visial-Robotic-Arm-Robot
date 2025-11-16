#ifndef _SYSTEM_MANAGER_A_H_
#define _SYSTEM_MANAGER_A_H_

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
#include "LED_d.h"
#include "JY61p_d.h"
#include "Chassis_d.h"
#include "Servo_d.h"
// Service Layer
#include "LOG_s.h"
#include "Delay_s.h"
#include "Chassis_d.h"
#include "BlueTooth.h"
#include "VisionProtocol.h"
#include "s_Kinematics.h"
// Application Layer
#include "a_Arm.h"

// 系统状态定义
typedef enum{
    SYS_STATE_INIT,
    SYS_STATE_RUNNING,
    SYS_STATE_ERROR
} SysState_t;
// 系统模式定义
typedef enum{
    SYS_MODE_NORMAL,
    SYS_MODE_EXPLOSIVE,
    SYS_MODE_CSGO,
    SYS_MODE_RESCUE
} SysMode_t;

// 接口函数声明
void SysManager_Init(void);
void SysManager_Process(void);
bool System_Is_Ready(void);

#endif
