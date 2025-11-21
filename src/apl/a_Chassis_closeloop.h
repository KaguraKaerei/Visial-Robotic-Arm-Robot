#ifndef __A_CHASSIS_CLOSELOOP_H
#define __A_CHASSIS_CLOSELOOP_H

#include "stm32f10x.h"
#include "Chassis_d.h"
#include "PID_s.h"
#include "JY61p_d.h"
#include "UART.h"
#include "VisionProtocol.h"



extern float totaltargerangle;

/* 四路电机驱动板自带速度闭环 */
extern PID_Param_t xunjipid;
/* 角度 */
extern PID_Param_t anglepid;
/* 角度实际值 */
extern JY61P_Data_t jy61pdata;

extern int16_t temptemptemp;

void Chassis_closeloop_Init(void);
void Chassis_closeloop_Move(void);

#endif
