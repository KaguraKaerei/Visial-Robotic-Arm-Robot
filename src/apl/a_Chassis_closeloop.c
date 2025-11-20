#include "a_Chassis_closeloop.h"
#include "Chassis_d.h"
#include "PID_s.h"
#include "JY61p_d.h"
#include "UART.h"
#include "VisionProtocol.h"

uint16_t basicspeed = 0; // 基础速度

float totaltargerangle = 0;

/* 四路电机驱动板自带速度闭环 */
PID_Param_t xunjipid;
/* 角度 */
PID_Param_t anglepid;
/* 角度实际值 */
JY61P_Data_t jy61pdata;

int16_t temptemptemp = 0;

void Chassis_closeloop_Init(void)
{
    PID_Init(&xunjipid);
    PID_SetPID(&xunjipid, 0.02, 0.0, 0.0);
    /* PID_SetPID(&xunjipid, 0.6, 0.0, 0.005); */
    PID_SetLimit(&xunjipid,1000,180);
    PID_Init(&anglepid);
    PID_SetPID(&anglepid, 80, 0.0, 0.0);		//就挺好
    PID_SetLimit(&anglepid, 1000, 1000);
    // PID_Controller();
    Delay_ms(100);
    Chassis_Move(0, 0);

/*     chassisParam.speed[CHASSIS_WHEEL_LF] = 0;
    chassisParam.speed[CHASSIS_WHEEL_RF] = 0;
    chassisParam.speed[CHASSIS_WHEEL_LR] = 0;
    chassisParam.speed[CHASSIS_WHEEL_RR] = 0;
    Chassis_SetSpeed(&chassisParam); */
}

/* 串级pid */
void Chassis_closeloop_Move(void)
{
    PID_Controller(&xunjipid, 0, VisionProtocol_Getopenmvdata(), 0.01); // 控制周期待定10ms
    JY61p_GetData(&jy61pdata);
    totaltargerangle += xunjipid.output;
	if (totaltargerangle > 360)
	{
        totaltargerangle = 360;
    }
	if (totaltargerangle < -360)
	{
        totaltargerangle = -360;
    }
    PID_Controller(&anglepid, /* xunjipid.output */totaltargerangle, jy61pdata.angle_z, 0.01); // 控制周期待定10ms
    // PID_Controller(&anglepid, temptemptemp, jy61pdata.angle_z, 0.01); // 控制周期待定10ms
    Chassis_Move(basicspeed,anglepid.output);
}
