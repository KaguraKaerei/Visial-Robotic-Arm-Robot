#include "main.h"

// 时间片定义
static sysTick_t infoDelayer = 0;
static uint32_t yawDelayer = 0;

JY61P_Data_t jy61pData_Read = { 0 };

int main()
{
    SysManager_Init();

    while(1){
        // 状态机驱动部分
        SysManager_Process();               // 系统管理
        if(System_Is_Ready()){
            BlueTooth_Process();            // 蓝牙通信
            VisionProtocol_Process();       // 视觉通信
            Arm_Process();                  // 机械臂控制
        }

        // 时间片轮转部分
        if(sysTick_DelayMs(&infoDelayer, 10)){
            float target_r = 960.0f;
            float target_z = 540.0f;
            _WavePrintf(4, &target_r, &target_z, &arm_param.r, &arm_param.z);
        }
        if(DWT_DelayUs(&yawDelayer, 100)){

        }
    }
}
