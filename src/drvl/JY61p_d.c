#include "JY61p_d.h"

/* JY61p指定寄存器写1个字节 */
void JY61p_WriteReg(uint8_t RegAddress, uint8_t Data)
{
    JY61p_I2C_Start();
    JY61p_I2C_SendByte(JY61P_ADDRESS << 1 | 0x00); // 从机地址写
    JY61p_I2C_ReceiveAck();
    JY61p_I2C_SendByte(RegAddress);
    JY61p_I2C_ReceiveAck();
    JY61p_I2C_SendByte(Data);
    JY61p_I2C_ReceiveAck();
    JY61p_I2C_Stop();
}

/* JY61p指定地址读1个字节 */
uint8_t JY61p_ReadReg(uint8_t RegAddress)
{
    uint8_t data;

    JY61p_I2C_Start();
    JY61p_I2C_SendByte(JY61P_ADDRESS << 1 | 0x00); // 从机地址写
    JY61p_I2C_ReceiveAck();
    JY61p_I2C_SendByte(RegAddress);
    JY61p_I2C_ReceiveAck();

    JY61p_I2C_Start();
    JY61p_I2C_SendByte(JY61P_ADDRESS << 1 | 0x01); // 从机地址读
    JY61p_I2C_ReceiveAck();
    data = JY61p_I2C_ReceiveByte();
    JY61p_I2C_SendAck(NACK);
    JY61p_I2C_Stop();

    return data;
}

/* JY61p指定地址读指定个字节 */
void JY61p_ReadReg_Mul(uint8_t RegAddress, uint8_t *data, uint8_t len)
{
    uint8_t i;
    JY61p_I2C_Start();
    JY61p_I2C_SendByte(JY61P_ADDRESS << 1 | 0x00); // 从机地址写
    JY61p_I2C_ReceiveAck();
    JY61p_I2C_SendByte(RegAddress);
    JY61p_I2C_ReceiveAck();
    JY61p_I2C_Start();
    JY61p_I2C_SendByte(JY61P_ADDRESS << 1 | 0x01); // 从机地址读
    JY61p_I2C_ReceiveAck();
    for (i = 0; i < len; i++) {
        data[i] = JY61p_I2C_ReceiveByte();
        if (i < len - 1) {
            JY61p_I2C_SendAck(ACK);
        } else {
            JY61p_I2C_SendAck(NACK); // 发送非应答，从机停止发送数据
        }
    }
    JY61p_I2C_Stop();
}

/* 检测数据是否正常传输		1~不正常	0~正常 */
uint8_t JY61p_Check(void)
{
    uint8_t ack;
    JY61p_I2C_Start();
    JY61p_I2C_SendByte((JY61P_ADDRESS) << 1 | 0); // 1010 000 0 设备地址写
    ack = JY61p_I2C_ReceiveAck();
    JY61p_I2C_Stop();

    return ack;
}

/* 返回0说明初始化成功，反之失败 */
uint8_t JY61p_Init(void)
{
    JY61p_I2C_Init();
    // 其他初始化操作

    return JY61p_Check();
}

/* 传入结构体指针，获取JY61p的各项数据 */
void JY61p_GetData(JY61P_Data_t *data)
{
    uint8_t data_acc[6];
    uint8_t data_gyro[6];
    uint8_t data_angle[6];

    JY61p_ReadReg_Mul(JY61P_ACC_X_L, data_acc, 6);
    JY61p_ReadReg_Mul(JY61P_GYRO_X_L, data_gyro, 6);
    JY61p_ReadReg_Mul(JY61P_ANGLE_X_L, data_angle, 6);

    // 解算加速度
    data->acc_x = ((int16_t)((data_acc[1] << 8) | data_acc[0])) / 32768.0f * 16.0f;
    data->acc_y = ((int16_t)((data_acc[3] << 8) | data_acc[2])) / 32768.0f * 16.0f;
    data->acc_z = ((int16_t)((data_acc[5] << 8) | data_acc[4])) / 32768.0f * 16.0f;

    // 解算角速度
    data->gyro_x = ((int16_t)((data_gyro[1] << 8) | data_gyro[0])) / 32768.0f * 2000.0f;
    data->gyro_y = ((int16_t)((data_gyro[3] << 8) | data_gyro[2])) / 32768.0f * 2000.0f;
    data->gyro_z = ((int16_t)((data_gyro[5] << 8) | data_gyro[4])) / 32768.0f * 2000.0f;

    // 解算欧拉角
    data->angle_x = ((int16_t)((data_angle[1] << 8) | data_angle[0])) / 32768.0f * 180.0f;
    data->angle_y = ((int16_t)((data_angle[3] << 8) | data_angle[2])) / 32768.0f * 180.0f;
    data->angle_z = ((int16_t)((data_angle[5] << 8) | data_angle[4])) / 32768.0f * 180.0f;
}
