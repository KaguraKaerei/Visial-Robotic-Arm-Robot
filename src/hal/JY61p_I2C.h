#ifndef __MYI2C_H
#define __MYI2C_H

#include "stm32f10x.h" // Device header
#include "Delay_s.h"

#define ACK  0
#define NACK 1

void JY61p_I2C_Init(void);
void JY61p_I2C_Start(void);
void JY61p_I2C_Stop(void);
void JY61p_I2C_SendByte(uint8_t Byte);
uint8_t JY61p_I2C_ReceiveByte(void);
void JY61p_I2C_SendAck(uint8_t AckBit);
uint8_t JY61p_I2C_ReceiveAck(void);

#endif
