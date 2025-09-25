#include "main.h"

// 时间片定义
static sysTick_t infoDelayer = 0;
static uint32_t yawDelayer   = 0;

JY61P_Data_t jy61pData_Read = {0};

uint8_t flag = 0;

int main()
{
    SysManager_Init();

    Delay_s(2);

    while(1)
	{
		// 状态机驱动部分
		SysManager_Process();               // 系统管理
        if(System_Is_Ready()){
            BlueTooth_Process();            // 蓝牙通信
            VisionProtocol_Process();       // 视觉通信
        }

		// 时间片轮转部分
		if(sysTick_DelayMs(&infoDelayer, 100)){
            // Chassis_GetData(&chassisParam);
        }
		if(DWT_DelayUs(&yawDelayer, 100)){
			
		}

		if(!flag){
            Chassis_Turn(-90, 90);
            flag = 1;
        }
    }
}
