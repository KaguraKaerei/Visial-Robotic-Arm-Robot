#include "LED_d.h"

void LED_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_SetBits(GPIOC, GPIO_Pin_13);
}

void LED_On(void)
{
    GPIO_ResetBits(GPIOC, GPIO_Pin_13);
}

void LED_Off(void)
{
    GPIO_SetBits(GPIOC, GPIO_Pin_13);
}
