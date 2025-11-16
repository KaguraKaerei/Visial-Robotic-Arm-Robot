#ifndef _A_ARM_H_
#define _A_ARM_H_

#include "stm32f10x.h"

typedef enum{
    ARM_STATE_IDLE = 0,
    ARM_FIND_QR,
    ARM_FIND_TASK,
    ARM_STATE_AIM_TARGET,
    ARM_STATE_AIM_LASER,
    ARM_STATE_TARGET,
    ARM_STATE_GRASPING,
    ARM_STATE_PUT_IN,
    ARM_STATE_ANY,
    ARM_STATE_END,
    ARM_STATE_MAX
} Arm_State_t;

typedef struct{
    float x;
    float y;
    float z;
    float r;
    float depth;
    float angle;
} Arm_Param_t;

extern Arm_State_t arm_state;
extern Arm_Param_t arm_param;

void Arm_Process(void);

#endif
