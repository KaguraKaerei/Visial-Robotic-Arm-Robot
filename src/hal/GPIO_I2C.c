#include "GPIO_I2C.h"

void GPIO_I2C_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_WriteBit(GPIOB, GPIO_Pin_10, (BitAction)1);
	GPIO_WriteBit(GPIOB, GPIO_Pin_11, (BitAction)1); 
}

void GPIO_I2C_W_SCL(uint8_t BitValue)
{
	GPIO_WriteBit(GPIOB, GPIO_Pin_10, (BitAction)BitValue);
}

void GPIO_I2C_W_SDA(uint8_t BitValue)
{
	GPIO_WriteBit(GPIOB, GPIO_Pin_11, (BitAction)BitValue);
}

uint8_t GPIO_I2C_R_SDA(void)
{
	return GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11);
}


void GPIO_I2C_Start(void)
{
	GPIO_I2C_W_SDA(1);
	GPIO_I2C_W_SCL(1);
	GPIO_I2C_W_SDA(0);
	GPIO_I2C_W_SCL(0);
}

void GPIO_I2C_Stop(void)
{
	GPIO_I2C_W_SDA(0);
	GPIO_I2C_W_SCL(1);
	GPIO_I2C_W_SDA(1);
}

void GPIO_I2C_dataSend(uint8_t dataSend)
{
	for(uint8_t i=0;i<8;i++)
	{
		GPIO_I2C_W_SDA( dataSend & (0x80 >> i) );
		GPIO_I2C_W_SCL(1);
		GPIO_I2C_W_SCL(0);
	}
}

uint8_t GPIO_I2C_dataRcvd(void)
{
	uint8_t dataRcvd = 0x00;	//0000 0000
	GPIO_I2C_W_SDA(1);
	for(uint8_t i = 0; i < 8; i++)
	{
		GPIO_I2C_W_SCL(1);
		if(GPIO_I2C_R_SDA() == 1){dataRcvd |= (0x80 >> i);}
		GPIO_I2C_W_SCL(0);
	}
	return dataRcvd;
}

void GPIO_I2C_SendAck(uint8_t Ack)
{
	GPIO_I2C_W_SDA(Ack);
	GPIO_I2C_W_SCL(1);
	GPIO_I2C_W_SCL(0);
}

uint8_t GPIO_I2C_RcvdAck(void)
{
	uint8_t Ack;
	GPIO_I2C_W_SDA(1);
	GPIO_I2C_W_SCL(1);
	Ack = GPIO_I2C_R_SDA();
	GPIO_I2C_W_SCL(0);
	return Ack;
}
