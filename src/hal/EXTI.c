#include "EXTI.h"

void iEXTI_Init(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin_x)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    uint8_t GPIO_PortSource;
    uint8_t GPIO_PinSource;

    if(GPIOx == GPIOA){
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
        GPIO_PortSource = GPIO_PortSourceGPIOA;
    }
    if(GPIOx == GPIOB){
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
        GPIO_PortSource = GPIO_PortSourceGPIOB;
    }
    if(GPIOx == GPIOC){
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
        GPIO_PortSource = GPIO_PortSourceGPIOC;
    }
    for(GPIO_PinSource = 0; GPIO_PinSource < 16; ++GPIO_PinSource){
        if(GPIO_Pin_x == (1 << GPIO_PinSource))
            break;
    }

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_x;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOx, &GPIO_InitStructure);

    GPIO_EXTILineConfig(GPIO_PortSource, GPIO_PinSource);

    EXTI_InitStructure.EXTI_Line = 1 << GPIO_PinSource;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
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
