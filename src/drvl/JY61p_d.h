#ifndef __JY61p_D_H
#define __JY61p_D_H

#include "stm32f10x.h"
#include "JY61p_I2C.h"
#include "JY61p_Reg.h"

// 数据结构体
typedef struct{
    float acc_x;   // X轴加速度 (g)
    float acc_y;   // Y轴加速度 (g)
    float acc_z;   // Z轴加速度 (g)
    float gyro_x;  // X轴角速度 (°/s)
    float gyro_y;  // Y轴角速度 (°/s)
    float gyro_z;  // Z轴角速度 (°/s)
    float angle_x; // X轴欧拉角 (°)
    float angle_y; // Y轴欧拉角 (°)
    float angle_z; // Z轴欧拉角 (°)
} JY61P_Data_t;

// 函数声明
uint8_t JY61p_Init(void);
uint8_t JY61p_ReadReg(uint8_t RegAddress);
uint8_t JY61p_Check(void);
void JY61p_GetData(JY61P_Data_t* data);

#endif
