#include "EXTI.h"

static EXTI_Callback_t EXTI_Callback[16] = { 0 };

/**
 * @brief 初始化外部中断
 * @param GPIOx GPIO端口
 * @param GPIO_Pin_x GPIO引脚
 * @param mode GPIO模式
 * @param trigger 中断触发方式
 */
void iEXTI_Init(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin_x, GPIOMode_TypeDef mode, EXTITrigger_TypeDef trigger)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    uint8_t GPIO_PortSource;
    uint8_t GPIO_PinSource;

    if(GPIOx == GPIOA){
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
        GPIO_PortSource = GPIO_PortSourceGPIOA;
    }
    else if(GPIOx == GPIOB){
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
        GPIO_PortSource = GPIO_PortSourceGPIOB;
    }
    else if(GPIOx == GPIOC){
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
        GPIO_PortSource = GPIO_PortSourceGPIOC;
    }
    for(GPIO_PinSource = 0; GPIO_PinSource < 16; ++GPIO_PinSource){
        if(GPIO_Pin_x == (1 << GPIO_PinSource))
            break;
    }

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_x;
    GPIO_InitStructure.GPIO_Mode = mode;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOx, &GPIO_InitStructure);

    GPIO_EXTILineConfig(GPIO_PortSource, GPIO_PinSource);

    EXTI_InitStructure.EXTI_Line = 1 << GPIO_PinSource;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = trigger;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_InitTypeDef NVIC_InitStructure;
    if(GPIO_PinSource <= 4){
        NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn + GPIO_PinSource;
    }
    else if(GPIO_PinSource <= 9){
        NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
    }
    else{
        NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
    }
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}
/**
 * @brief 注册定时器更新中断回调函数
 */
void EXTI_RegisterCallback(uint32_t EXTI_Line, EXTI_Callback_t callback)
{
    for(uint8_t i = 0; i < 16; ++i){
        if(EXTI_Line == (1 << i)){
            EXTI_Callback[i] = callback;
            break;
        }
    }
}
/**
 * @brief 定时器中断服务程序(硬件层，只负责调用回调)
 */
void EXTI0_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line0) != RESET){
        EXTI_ClearITPendingBit(EXTI_Line0);
        // 调用驱动层注册的回调函数
        if(EXTI_Callback[0] != 0){
            EXTI_Callback[0]();
        }
    }
}
void EXTI1_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line1) != RESET){
        EXTI_ClearITPendingBit(EXTI_Line1);
        // 调用驱动层注册的回调函数
        if(EXTI_Callback[1] != 0){
            EXTI_Callback[1]();
        }
    }
}
void EXTI2_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line2) != RESET){
        EXTI_ClearITPendingBit(EXTI_Line2);
        // 调用驱动层注册的回调函数
        if(EXTI_Callback[2] != 0){
            EXTI_Callback[2]();
        }
    }
}
void EXTI3_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line3) != RESET){
        EXTI_ClearITPendingBit(EXTI_Line3);
        // 调用驱动层注册的回调函数
        if(EXTI_Callback[3] != 0){
            EXTI_Callback[3]();
        }
    }
}
void EXTI4_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line4) != RESET){
        EXTI_ClearITPendingBit(EXTI_Line4);
        // 调用驱动层注册的回调函数
        if(EXTI_Callback[4] != 0){
            EXTI_Callback[4]();
        }
    }
}
void EXTI9_5_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line5) != RESET){
        EXTI_ClearITPendingBit(EXTI_Line5);
        // 调用驱动层注册的回调函数
        if(EXTI_Callback[5] != 0){
            EXTI_Callback[5]();
        }
    }
    if(EXTI_GetITStatus(EXTI_Line6) != RESET){
        EXTI_ClearITPendingBit(EXTI_Line6);
        // 调用驱动层注册的回调函数
        if(EXTI_Callback[6] != 0){
            EXTI_Callback[6]();
        }
    }
    if(EXTI_GetITStatus(EXTI_Line7) != RESET){
        EXTI_ClearITPendingBit(EXTI_Line7);
        // 调用驱动层注册的回调函数
        if(EXTI_Callback[7] != 0){
            EXTI_Callback[7]();
        }
    }
    if(EXTI_GetITStatus(EXTI_Line8) != RESET){
        EXTI_ClearITPendingBit(EXTI_Line8);
        // 调用驱动层注册的回调函数
        if(EXTI_Callback[8] != 0){
            EXTI_Callback[8]();
        }
    }
    if(EXTI_GetITStatus(EXTI_Line9) != RESET){
        EXTI_ClearITPendingBit(EXTI_Line9);
        // 调用驱动层注册的回调函数
        if(EXTI_Callback[9] != 0){
            EXTI_Callback[9]();
        }
    }
}
void EXTI15_10_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line10) != RESET){
        EXTI_ClearITPendingBit(EXTI_Line10);
        // 调用驱动层注册的回调函数
        if(EXTI_Callback[10] != 0){
            EXTI_Callback[10]();
        }
    }
    if(EXTI_GetITStatus(EXTI_Line11) != RESET){
        EXTI_ClearITPendingBit(EXTI_Line11);
        // 调用驱动层注册的回调函数
        if(EXTI_Callback[11] != 0){
            EXTI_Callback[11]();
        }
    }
    if(EXTI_GetITStatus(EXTI_Line12) != RESET){
        EXTI_ClearITPendingBit(EXTI_Line12);
        // 调用驱动层注册的回调函数
        if(EXTI_Callback[12] != 0){
            EXTI_Callback[12]();
        }
    }
    if(EXTI_GetITStatus(EXTI_Line13) != RESET){
        EXTI_ClearITPendingBit(EXTI_Line13);
        // 调用驱动层注册的回调函数
        if(EXTI_Callback[13] != 0){
            EXTI_Callback[13]();
        }
    }
    if(EXTI_GetITStatus(EXTI_Line14) != RESET){
        EXTI_ClearITPendingBit(EXTI_Line14);
        // 调用驱动层注册的回调函数
        if(EXTI_Callback[14] != 0){
            EXTI_Callback[14]();
        }
    }
    if(EXTI_GetITStatus(EXTI_Line15) != RESET){
        EXTI_ClearITPendingBit(EXTI_Line15);
        // 调用驱动层注册的回调函数
        if(EXTI_Callback[15] != 0){
            EXTI_Callback[15]();
        }
    }
}
