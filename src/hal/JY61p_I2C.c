#include "JY61p_I2C.h"

void JY61p_I2C_W_SCL(uint8_t BitValue)
{
    GPIO_WriteBit(GPIOB, GPIO_Pin_8, (BitAction)BitValue);
    Delay_us(10);
}

void JY61p_I2C_W_SDA(uint8_t BitValue)
{
    GPIO_WriteBit(GPIOB, GPIO_Pin_9, (BitAction)BitValue);
    Delay_us(10);
}

uint8_t JY61p_I2C_R_SDA(void)
{
    uint8_t BitValue;
    //    Delay_ms(100);
    BitValue = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_9);
    Delay_us(10);
    return BitValue;
}

void JY61p_I2C_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_OD;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_SetBits(GPIOB, GPIO_Pin_8 | GPIO_Pin_9);
}

void JY61p_I2C_Start(void)
{
    JY61p_I2C_W_SDA(1);
    JY61p_I2C_W_SCL(1);
    JY61p_I2C_W_SDA(0);
    JY61p_I2C_W_SCL(0);
}

void JY61p_I2C_Stop(void)
{
    JY61p_I2C_W_SDA(0);
    JY61p_I2C_W_SCL(1);
    JY61p_I2C_W_SDA(1);
}

void JY61p_I2C_SendByte(uint8_t Byte)
{
    uint8_t i;
    for (i = 0; i < 8; i++) {
        JY61p_I2C_W_SDA(!!(Byte & (0x80 >> i)));
        JY61p_I2C_W_SCL(1);
        JY61p_I2C_W_SCL(0);
    }
}

uint8_t JY61p_I2C_ReceiveByte(void)
{
    uint8_t i, Byte = 0x00;
    JY61p_I2C_W_SDA(1);
    for (i = 0; i < 8; i++) {
        JY61p_I2C_W_SCL(1);
        if (JY61p_I2C_R_SDA()) { Byte |= (0x80 >> i); }
        JY61p_I2C_W_SCL(0);
    }
    return Byte;
}

void JY61p_I2C_SendAck(uint8_t AckBit)
{
    JY61p_I2C_W_SDA(AckBit);
    JY61p_I2C_W_SCL(1);
    JY61p_I2C_W_SCL(0);
}

uint8_t JY61p_I2C_ReceiveAck(void)
{
    uint8_t AckBit;
    JY61p_I2C_W_SDA(1);
    JY61p_I2C_W_SCL(1);
    AckBit = JY61p_I2C_R_SDA();
    JY61p_I2C_W_SCL(0);
    return AckBit;
}
