#include "TIM.h"

// 中断回调函数指针
static void(* TIM3_Callback)(void) = 0;
/**
  * @brief 定时器硬件初始化 
  */
void Timer_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	
	TIM_InternalClockConfig(TIM3);
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period = TIM3_PERIOD;
	TIM_TimeBaseInitStruct.TIM_Prescaler = TIM3_PRESCALER;
	TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStruct);
	
	TIM_ClearFlag(TIM3, TIM_IT_Update);
	
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_Cmd(TIM3, ENABLE);
}
/**
 * @brief 注册TIM3更新中断回调函数
 */
void TIM3_RegisterCallback(void(* callback)(void))
{
    TIM3_Callback = callback;
}
/**
 * @brief TIM3中断服务程序(硬件层，只负责调用回调)
 */
void TIM3_IRQHandler(void)
{
    if(TIM_GetFlagStatus(TIM3, TIM_FLAG_Update) == SET)
    {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
        // 调用驱动层注册的回调函数
        if(TIM3_Callback != 0){
            TIM3_Callback();
        }
    }
}
