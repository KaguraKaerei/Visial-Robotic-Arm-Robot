#ifndef _GPIO_I2C_H_
#define _GPIO_I2C_H_

#include "stm32f10x.h"

void GPIO_I2C_Init(void);
void GPIO_I2C_Start(void);
void GPIO_I2C_Stop(void);
void GPIO_I2C_dataSend(uint8_t dataSend);
uint8_t GPIO_I2C_dataRcvd(void);
void GPIO_I2C_SendAck(uint8_t Ack);
uint8_t GPIO_I2C_RcvdAck(void);

#endif
