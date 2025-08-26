#include "main.h"

// 时间片定义
static sysTick_t taskDelayer1 = 0;
static sysTick_t taskDelayer2 = 0; 
static sysTick_t taskDelayer3 = 0;

int main()
{
    SysManager_Init();

	while(1)
	{
		// 状态机驱动部分
		SysManager_Process();
        if(System_Is_Ready()){

        }

		// 时间片轮转部分
		if(sysTick_DelayMs(&taskDelayer1, 100)){

        }
		if(sysTick_DelayMs(&taskDelayer2, 1000)){
            _INFO("1s");
        }
        if(sysTick_DelayMs(&taskDelayer3, 2000)){

        }
	}
}
