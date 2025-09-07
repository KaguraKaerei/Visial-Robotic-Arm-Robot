#ifndef _CHASSIS_D_H_
#define _CHASSIS_D_H_

#include "stm32f10x.h"
#include "JY61p_d.h"
#include "PID_s.h"

extern uint8_t stopFlag;

/* ========================= 底 盘 类 定 义 ========================= */

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
    int encorderSpeed[CHASSIS_WHEEL_MAX];
    int encorderAll[CHASSIS_WHEEL_MAX];
    int encorder10ms[CHASSIS_WHEEL_MAX];
    int linearVel;
    float angularVel;           // rad/s
    float p, i, d;
} ChassisParam_t;

extern ChassisParam_t chassisParam;
extern PID_Param_t jy61pYawPID;

/* ========================= 底 盘 接 口 函 数 声 明 ========================= */

void Chassis_Init(void);
void Chassis_SetSpeed(const ChassisParam_t* const param);
void Chassis_SetPWM(const ChassisParam_t* const param);
void Chassis_SetPID(const ChassisParam_t* const param);
void Chassis_GetData(ChassisParam_t* const param);

void Chassis_Move(int linearVel, int angularVel);
void Chassis_Stop(void);
void Chassis_GoStraight(int speed);
void Chassis_Turn(int angle, int angularVel);

void Chassis_SelfCtrl_Init(void);
void Chassis_SelfCtrl_Move(int linearVel, int angularVel);
void Chassis_SelfCtrl_GoStraight(int speed);
void Chassis_SelfCtrl_Turn(int angle, int angularVel);
void Chassis_SelfCtrl_Turn_JY61(uint8_t relativeFlag, int targetAngle, uint8_t tolerance, JY61P_Data_t* const jyData, int timeout);

#endif
