#include "CAN.h"

/* ========================= 私 有 变 量 声 明 ========================= */

#define NO_IRQN ((IRQn_Type)(-1))

typedef struct{
    CAN_Mode_t Mode;
    uint32_t RCC_Periph;
    CAN_TypeDef* CANx;
    IRQn_Type RX0_IRQn;
    GPIO_TypeDef* TX_Port;
    uint16_t TX_Pin;
    GPIO_TypeDef* RX_Port;
    uint16_t RX_Pin;
    uint32_t GPIO_RCC;
    uint16_t Prescaler;
    uint8_t BS1;
    uint8_t BS2;
    FunctionalState NART;   // 不自动重装（DISABLE: 自动重装， ENABLE: 不自动重装）
} CAN_Config_t;

static const CAN_Config_t CAN_Config[] = {
    // CAN1 NORMAL 1Mbps
    {
        .Mode = CAN_MODE_NORMAL,
        .RCC_Periph = RCC_APB1Periph_CAN1,
        .CANx = CAN1,
        .RX0_IRQn = USB_LP_CAN1_RX0_IRQn,
        .TX_Port = GPIOA,
        .TX_Pin = GPIO_Pin_12,
        .RX_Port = GPIOA,
        .RX_Pin = GPIO_Pin_11,
        .GPIO_RCC = RCC_APB2Periph_GPIOA,
        .Prescaler = 4,
        .BS1 = CAN_BS1_7tq,
        .BS2 = CAN_BS2_1tq,
        .NART = DISABLE
    },
    // CAN1 LOOPBACK 1Mbps
    {
        .Mode = CAN_MODE_LOOPBACK,
        .RCC_Periph = RCC_APB1Periph_CAN1,
        .CANx = CAN1,
        .RX0_IRQn = USB_LP_CAN1_RX0_IRQn,
        .TX_Port = GPIOA,
        .TX_Pin = GPIO_Pin_12,
        .RX_Port = GPIOA,
        .RX_Pin = GPIO_Pin_11,
        .GPIO_RCC = RCC_APB2Periph_GPIOA,
        .Prescaler = 4,
        .BS1 = CAN_BS1_7tq,
        .BS2 = CAN_BS2_1tq,
        .NART = DISABLE
    },
    // CAN1 SILENT 1Mbps
    {
        .Mode = CAN_MODE_SILENT,
        .RCC_Periph = RCC_APB1Periph_CAN1,
        .CANx = CAN1,
        .RX0_IRQn = USB_LP_CAN1_RX0_IRQn,
        .TX_Port = GPIOA,
        .TX_Pin = GPIO_Pin_12,
        .RX_Port = GPIOA,
        .RX_Pin = GPIO_Pin_11,
        .GPIO_RCC = RCC_APB2Periph_GPIOA,
        .Prescaler = 4,
        .BS1 = CAN_BS1_7tq,
        .BS2 = CAN_BS2_1tq,
        .NART = DISABLE
    },
    // CAN1 SILENT_LOOPBACK 1Mbps
    {
        .Mode = CAN_MODE_SILENT_LOOPBACK,
        .RCC_Periph = RCC_APB1Periph_CAN1,
        .CANx = CAN1,
        .RX0_IRQn = USB_LP_CAN1_RX0_IRQn,
        .TX_Port = GPIOA,
        .TX_Pin = GPIO_Pin_12,
        .RX_Port = GPIOA,
        .RX_Pin = GPIO_Pin_11,
        .GPIO_RCC = RCC_APB2Periph_GPIOA,
        .Prescaler = 4,
        .BS1 = CAN_BS1_7tq,
        .BS2 = CAN_BS2_1tq,
        .NART = DISABLE
    }
};

static void(*CAN_RxCallback)(const CanRxMsg* msg) = 0;

/* ========================= 私 有 函 数 声 明 ========================= */



/* ========================= 接 口 函 数 实 现 ========================= */

void iCAN_Init(CAN_Mode_t mode)
{
    const CAN_Config_t* config = 0;
    for(uint8_t i = 0; i < sizeof(CAN_Config) / sizeof(CAN_Config[0]); ++i){
        if(CAN_Config[i].Mode == mode){
            config = &CAN_Config[i];
            break;
        }
    }
    if(!config) return;

    RCC_APB2PeriphClockCmd(config->GPIO_RCC, ENABLE);
    RCC_APB1PeriphClockCmd(config->RCC_Periph, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = config->TX_Pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(config->TX_Port, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = config->RX_Pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(config->RX_Port, &GPIO_InitStructure);

    CAN_InitTypeDef CAN_InitStructure;
    CAN_InitStructure.CAN_TTCM = DISABLE;
    CAN_InitStructure.CAN_ABOM = ENABLE;
    CAN_InitStructure.CAN_AWUM = DISABLE;
    CAN_InitStructure.CAN_NART = config->NART;
    CAN_InitStructure.CAN_RFLM = DISABLE;
    CAN_InitStructure.CAN_TXFP = DISABLE;
    switch(mode){
        case CAN_MODE_NORMAL:
            CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
            break;
        case CAN_MODE_LOOPBACK:
            CAN_InitStructure.CAN_Mode = CAN_Mode_LoopBack;
            break;
        case CAN_MODE_SILENT:
            CAN_InitStructure.CAN_Mode = CAN_Mode_Silent;
            break;
        case CAN_MODE_SILENT_LOOPBACK:
            CAN_InitStructure.CAN_Mode = CAN_Mode_Silent_LoopBack;
            break;
    }
    CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
    CAN_InitStructure.CAN_BS1 = config->BS1;
    CAN_InitStructure.CAN_BS2 = config->BS2;
    CAN_InitStructure.CAN_Prescaler = config->Prescaler;

    CAN_Init(config->CANx, &CAN_InitStructure);

    CAN_FilterInitTypeDef CAN_FilterInitStructure;
    CAN_FilterInitStructure.CAN_FilterNumber = 0;
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
    CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
    CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FilterFIFO0;
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);

    if(config->RX0_IRQn != NO_IRQN){
        NVIC_InitTypeDef NVIC_InitStructure;
        NVIC_InitStructure.NVIC_IRQChannel = config->RX0_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);

        CAN_ITConfig(config->CANx, CAN_IT_FMP0, ENABLE);
    }
}

bool CAN_SendStd(uint16_t stdId, const uint8_t* data, uint8_t len)
{
    if(len > 8) return false;

    CanTxMsg TxMessage;
    TxMessage.StdId = stdId & 0x7FF;
    TxMessage.ExtId = 0;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = len;
    for(uint8_t i = 0; i < len; ++i){
        TxMessage.Data[i] = data[i];
    }

    uint8_t mail_box = CAN_Transmit(CAN1, &TxMessage);
    if(mail_box == CAN_TxStatus_NoMailBox) return false;
    uint32_t timeout = 0;
    while((CAN_TransmitStatus(CAN1, mail_box) != CAN_TxStatus_Ok) && (timeout < 0xFFFF)){
        ++timeout;
    }
    return (timeout < 0xFFFF);
}

void CAN_RegisterRxCallback(void(*callback)(const CanRxMsg* msg))
{
    CAN_RxCallback = callback;
}

/* ========================= 私 有 函 数 实 现 ========================= */

void USB_LP_CAN1_RX0_IRQHandler(void)
{
    if(CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET){
        CanRxMsg RxMessage;
        CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
        if(CAN_RxCallback){
            CAN_RxCallback(&RxMessage);
        }
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
    }
}
