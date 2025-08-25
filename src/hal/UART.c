#include "UART.h"

/* 串口初始化配置表 */
typedef struct{
    iUSART_t USART;
    USART_Mode_t Mode;
    uint32_t RCC_Periph;
    USART_TypeDef* USARTx;
    IRQn_Type IRQn;
    uint32_t BaudRate;
} USART_Config_t;
const USART_Config_t USART_Config[] = {
    {iUSART1, USART_MODE_BASIC, RCC_APB2Periph_USART1, USART1, USART1_IRQn, 115200},
    {iUSART2, USART_MODE_BASIC, RCC_APB1Periph_USART2, USART2, USART2_IRQn, 115200},
    {iUSART3, USART_MODE_HALF_DUPLEX, RCC_APB1Periph_USART3, USART3, USART3_IRQn, 115200}
};
/* 中断回调函数指针 */
static USART_Callback_t UART1_Callback = 0;
static USART_Callback_t UART2_Callback = 0;
static USART_Callback_t UART3_Callback = 0;
/**
  * @brief 串口硬件初始化 
  */
void UART_Init(iUSART_t USART, USART_Mode_t mode)
{
    // 串口及模式选择
    const USART_Config_t* config = 0;
    for(uint8_t i = 0; i < sizeof(USART_Config)/sizeof(USART_Config[0]); ++i){
        if(USART_Config[i].USART == USART && USART_Config[i].Mode == mode){
            config = &USART_Config[i];
            break;
        }
    }
    if(!config) return;
    // GPIO初始化配置
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure;
    if(config->USART == iUSART1){
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
        if(mode == USART_MODE_BASIC){
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
            GPIO_Init(GPIOA, &GPIO_InitStructure);
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
            GPIO_Init(GPIOA, &GPIO_InitStructure);
        }
        else if(mode == USART_MODE_HALF_DUPLEX){
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_Init(GPIOA, &GPIO_InitStructure);
        }
    }
    else if(config->USART == iUSART2){
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
        if(mode == USART_MODE_BASIC){
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
            GPIO_Init(GPIOA, &GPIO_InitStructure);
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
            GPIO_Init(GPIOA, &GPIO_InitStructure);
        }
        else if(mode == USART_MODE_HALF_DUPLEX){
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_Init(GPIOA, &GPIO_InitStructure);
        }
    }
    else if(config->USART == iUSART3){
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
        if(mode == USART_MODE_BASIC){
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
            GPIO_Init(GPIOB, &GPIO_InitStructure);
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
            GPIO_Init(GPIOB, &GPIO_InitStructure);
        }
        else if(mode == USART_MODE_HALF_DUPLEX){
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_Init(GPIOB, &GPIO_InitStructure);
        }
    }
    // USART初始化配置
    RCC_APB2PeriphClockCmd(config->RCC_Periph, ENABLE);
    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = config->BaudRate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(config->USARTx, &USART_InitStructure);
    if(config->Mode == USART_MODE_HALF_DUPLEX){
        USART_HalfDuplexCmd(config->USARTx, ENABLE);
    }
    // NVIC初始化配置
    USART_ClearFlag(config->USARTx, USART_FLAG_RXNE);
    USART_ITConfig(config->USARTx, USART_IT_RXNE, ENABLE);
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = config->IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    // 启动串口
    USART_Cmd(config->USARTx, ENABLE);
}
/**
  * @brief 注册USART接收中断回调函数
  */
void USART_RegisterCallback(iUSART_t USART, USART_Callback_t callback)
{
    switch(USART)
    {
        case iUSART1:
        {
        	UART1_Callback = callback;
            break;
        }
        case iUSART2:
        {
            UART2_Callback = callback;
            break;
        }
        case iUSART3:
        {
            UART3_Callback = callback;
            break;
        }
    }
}
/**
  * @brief 串口中断服务程序(硬件层，只负责调用回调)
  */
void USART1_IRQHandler(void)
{
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
		// 调用驱动层注册的回调函数
		if(UART1_Callback != 0){
			UART1_Callback();
		}
	}
}
void USART2_IRQHandler(void)
{
    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
        // 调用驱动层注册的回调函数
        if(UART2_Callback != 0){
            UART2_Callback();
        }
    }
}
void USART3_IRQHandler(void)
{
    if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        USART_ClearITPendingBit(USART3, USART_IT_RXNE);
        // 调用驱动层注册的回调函数
        if(UART3_Callback != 0){
            UART3_Callback();
        }
    }
}

#pragma import(__use_no_semihosting)             

struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       

void _sys_exit(int x) 
{ 
	x = x; 
}

int fputc(int ch, FILE *f)
{    
	while((USART1->SR&0X40)==0){};
		USART1->DR = (u8) ch;
	return ch;
}

int fgetc(FILE *f)
{
	while((USART1->SR&0X20)==0){};
	return (int)(USART1->DR);
}
