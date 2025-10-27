#ifndef _Servo_d_H_
#define _Servo_d_H_

#include "stm32f10x.h"
#include <stdbool.h>

typedef enum{
    SERVO_CHASSIS = 1,
    SERVO_JOINT_1 = 2,
    SERVO_JOINT_2 = 3,
    SERVO_JOINT_3 = 4,
    SERVO_JOINT_GRIPPER = 5,
} Servo_ID_t;

void Servo_Init(void);
bool Servo_SetCCR(Servo_ID_t servo_id, uint16_t ccr);
uint16_t Servo_GetCCR(Servo_ID_t servo_id);

#endif
