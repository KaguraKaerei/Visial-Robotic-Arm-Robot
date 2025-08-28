#ifndef _CHASSIS_D_H_
#define _CHASSIS_D_H_

#include "stm32f10x.h"

typedef enum{
    CHASSIS_WHEEL_LF,
    CHASSIS_WHEEL_RF,
    CHASSIS_WHEEL_LR,
    CHASSIS_WHEEL_RR,
    CHASSIS_WHEEL_MAX
} ChassisWheel_t;
typedef struct{
    int speed[CHASSIS_WHEEL_MAX];
    uint16_t pwm[CHASSIS_WHEEL_MAX];
    int encorderAll[CHASSIS_WHEEL_MAX];
    int encorder10ms[CHASSIS_WHEEL_MAX];
    float p;
    float i;
    float d;
} ChassisParam_t;

extern ChassisParam_t chassisParam;

void Chassis_Init(void);
void Chassis_SetSpeed(const ChassisParam_t* const param);
void Chassis_SetPWM(const ChassisParam_t* const param);
void Chassis_SetPID(const ChassisParam_t* const param);
void Chassis_GetData(ChassisParam_t* const param);

#endif
