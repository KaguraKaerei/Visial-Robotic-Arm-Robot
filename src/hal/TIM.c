#include "TIM.h"

#define NO_IRQN ((IRQn_Type)(-1))

/* 定时器初始化配置表 */
typedef struct{
    iTIM_t TIM;
    TIM_Mode_t Mode;
    uint16_t TIM_Channel;
    uint32_t RCC_Periph;
    TIM_TypeDef* TIMx;
    IRQn_Type IRQn;
    uint16_t TIM_Period;
    uint16_t TIM_Prescaler;
    uint16_t TIM_Pulse;
} TIM_Config_t;

// 五个舵机PWM：TIM2: PA0(CH1)、PA1(CH2) ; TIM3: PA7(CH2)、PB0(CH3)、PB1(CH4)   20ms周期，0.5ms-2.5ms脉宽
// 定时器，模式，通道，时钟，定时器，中断号，周期，预分频，占空比
static const TIM_Config_t TIM_Config[] = {
    // TIM2
    {iTIM2, TIM_MODE_BASIC, 0x0000, RCC_APB1Periph_TIM2, TIM2, TIM2_IRQn, 10000 - 1, 7200 - 1, 0},
    {iTIM2, TIM_MODE_PWM, 0x1100, RCC_APB1Periph_TIM2, TIM2, NO_IRQN, 20 * 1000 - 1, 72 - 1, 0},
    // TIM3
    {iTIM3, TIM_MODE_BASIC, 0x0000, RCC_APB1Periph_TIM3, TIM3, TIM3_IRQn, 10000 - 1, 7200 - 1, 0},
    {iTIM3, TIM_MODE_PWM, 0x0111, RCC_APB1Periph_TIM3, TIM3, NO_IRQN, 20 * 1000 - 1, 72 - 1, 0},
    // TIM4
    {iTIM4, TIM_MODE_BASIC, 0x0000, RCC_APB1Periph_TIM4, TIM4, TIM4_IRQn, 10000 - 1, 7200 - 1, 0},
    {iTIM4, TIM_MODE_PWM, 0x1111, RCC_APB1Periph_TIM4, TIM4, NO_IRQN, 1000 - 1, 72 - 1, 0},
};
/* TIM中断回调函数 */
static TIM_Callback_t TIM1_Callback = 0;
static TIM_Callback_t TIM2_Callback = 0;
static TIM_Callback_t TIM3_Callback = 0;
static TIM_Callback_t TIM4_Callback = 0;
/**
  * @brief 定时器硬件初始化
  * @param TIMx 定时器选择
  * @param mode 定时器模式
  */
void iTIM_Init(iTIM_t TIMx, TIM_Mode_t mode)
{
    // 定时器及模式选择
    const TIM_Config_t* config = 0;
    for(uint8_t i = 0; i < sizeof(TIM_Config) / sizeof(TIM_Config[0]); ++i){
        if(TIM_Config[i].TIM == TIMx && TIM_Config[i].Mode == mode){
            config = &TIM_Config[i];
            break;
        }
    }
    if(!config) return;
    // 基本初始化配置
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    
    if(config->Mode == TIM_MODE_PWM){
        GPIO_InitTypeDef GPIO_InitStructure;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        
        if(config->TIMx == TIM2){
            RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
            // 根据 TIM_Channel 位掩码配置对应引脚
            // TIM2: CH1=PA0, CH2=PA1, CH3=PA2, CH4=PA3
            uint16_t pins = 0;
            if(config->TIM_Channel & 0x1000) pins |= GPIO_Pin_0; // CH1
            if(config->TIM_Channel & 0x0100) pins |= GPIO_Pin_1; // CH2
            if(config->TIM_Channel & 0x0010) pins |= GPIO_Pin_2; // CH3
            if(config->TIM_Channel & 0x0001) pins |= GPIO_Pin_3; // CH4
            if(pins){
                GPIO_InitStructure.GPIO_Pin = pins;
                GPIO_Init(GPIOA, &GPIO_InitStructure);
            }
        }
        else if(config->TIMx == TIM3){
            // TIM3: CH1=PA6, CH2=PA7, CH3=PB0, CH4=PB1
            uint16_t pins_a = 0, pins_b = 0;
            if(config->TIM_Channel & 0x1000) pins_a |= GPIO_Pin_6; // CH1
            if(config->TIM_Channel & 0x0100) pins_a |= GPIO_Pin_7; // CH2
            if(config->TIM_Channel & 0x0010) pins_b |= GPIO_Pin_0; // CH3
            if(config->TIM_Channel & 0x0001) pins_b |= GPIO_Pin_1; // CH4
            
            if(pins_a){
                RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
                GPIO_InitStructure.GPIO_Pin = pins_a;
                GPIO_Init(GPIOA, &GPIO_InitStructure);
            }
            if(pins_b){
                RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
                GPIO_InitStructure.GPIO_Pin = pins_b;
                GPIO_Init(GPIOB, &GPIO_InitStructure);
            }
        }
        else if(config->TIMx == TIM4){
            RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
            // TIM4: CH1=PB6, CH2=PB7, CH3=PB8, CH4=PB9
            uint16_t pins = 0;
            if(config->TIM_Channel & 0x1000) pins |= GPIO_Pin_6; // CH1
            if(config->TIM_Channel & 0x0100) pins |= GPIO_Pin_7; // CH2
            if(config->TIM_Channel & 0x0010) pins |= GPIO_Pin_8; // CH3
            if(config->TIM_Channel & 0x0001) pins |= GPIO_Pin_9; // CH4
            if(pins){
                GPIO_InitStructure.GPIO_Pin = pins;
                GPIO_Init(GPIOB, &GPIO_InitStructure);
            }
        }
    }
    
    if(config->RCC_Periph & RCC_APB2Periph_TIM1)
        RCC_APB2PeriphClockCmd(config->RCC_Periph, ENABLE);
    else
        RCC_APB1PeriphClockCmd(config->RCC_Periph, ENABLE);
    TIM_InternalClockConfig(config->TIMx);
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    TIM_TimeBaseInitStruct.TIM_Period = config->TIM_Period;
    TIM_TimeBaseInitStruct.TIM_Prescaler = config->TIM_Prescaler;
    TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(config->TIMx, &TIM_TimeBaseInitStruct);
    // 不同模式初始化配置
    if(mode == TIM_MODE_PWM){
        TIM_OCInitTypeDef TIM_OCInitStruct;
        TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
        TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
        TIM_OCInitStruct.TIM_Pulse = config->TIM_Pulse;
        TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
        if(config->TIM_Channel & 0x1000){
            TIM_OC1Init(config->TIMx, &TIM_OCInitStruct);
            TIM_OC1PreloadConfig(config->TIMx, TIM_OCPreload_Enable);
        }
        if(config->TIM_Channel & 0x0100){
            TIM_OC2Init(config->TIMx, &TIM_OCInitStruct);
            TIM_OC2PreloadConfig(config->TIMx, TIM_OCPreload_Enable);
        }
        if(config->TIM_Channel & 0x0010){
            TIM_OC3Init(config->TIMx, &TIM_OCInitStruct);
            TIM_OC3PreloadConfig(config->TIMx, TIM_OCPreload_Enable);
        }
        if(config->TIM_Channel & 0x0001){
            TIM_OC4Init(config->TIMx, &TIM_OCInitStruct);
            TIM_OC4PreloadConfig(config->TIMx, TIM_OCPreload_Enable);
        }
    }
    else if(mode == TIM_MODE_IC){

    }
    else if(mode == TIM_MODE_ENCODER){

    }
    // NVIC配置
    if(config->IRQn != NO_IRQN){
        TIM_ClearFlag(config->TIMx, TIM_IT_Update);
        TIM_ITConfig(config->TIMx, TIM_IT_Update, ENABLE);
        NVIC_InitTypeDef NVIC_InitStructure;
        NVIC_InitStructure.NVIC_IRQChannel = config->IRQn;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
        NVIC_Init(&NVIC_InitStructure);
    }
    // 启动定时器
    TIM_Cmd(config->TIMx, ENABLE);
}
/**
 * @brief 注册定时器更新中断回调函数
 */
void TIM_RegisterCallback(iTIM_t TIM, TIM_Callback_t callback)
{
    switch(TIM){
        case iTIM1:
            TIM1_Callback = callback;
            break;
        case iTIM2:
            TIM2_Callback = callback;
            break;
        case iTIM3:
            TIM3_Callback = callback;
            break;
        case iTIM4:
            TIM4_Callback = callback;
            break;
    }
}
/**
 * @brief 定时器中断服务程序(硬件层，只负责调用回调)
 */
void TIM1_IRQHandler(void)
{
    if(TIM_GetFlagStatus(TIM1, TIM_FLAG_Update) == SET){
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
        // 调用注册的回调函数
        if(TIM1_Callback != 0){
            TIM1_Callback();
        }
    }
}
void TIM2_IRQHandler(void)
{
    if(TIM_GetFlagStatus(TIM2, TIM_FLAG_Update) == SET){
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        // 调用注册的回调函数
        if(TIM2_Callback != 0){
            TIM2_Callback();
        }
    }
}
void TIM3_IRQHandler(void)
{
    if(TIM_GetFlagStatus(TIM3, TIM_FLAG_Update) == SET){
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
        // 调用注册的回调函数
        if(TIM3_Callback != 0){
            TIM3_Callback();
        }
    }
}
void TIM4_IRQHandler(void)
{
    if(TIM_GetFlagStatus(TIM4, TIM_FLAG_Update) == SET){
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
        // 调用注册的回调函数
        if(TIM4_Callback != 0){
            TIM4_Callback();
        }
    }
}
