#ifndef _CAN_H_
#define _CAN_H_

#include "stm32f10x.h"
#include <stdbool.h>

typedef enum{
    CAN_MODE_NORMAL = 0,
    CAN_MODE_LOOPBACK,
    CAN_MODE_SILENT,
    CAN_MODE_SILENT_LOOPBACK
} CAN_Mode_t;

void iCAN_Init(CAN_Mode_t mode);
bool CAN_SendStd(uint16_t stdId, const uint8_t* data, uint8_t len);
void CAN_RegisterRxCallback(void(* callback)(const CanRxMsg* msg));

#endif
