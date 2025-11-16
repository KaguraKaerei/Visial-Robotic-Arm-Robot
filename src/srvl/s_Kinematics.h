#ifndef _s_Kinematics_H_
#define _s_Kinematics_H_

#include "stm32f10x.h"
#include <stdbool.h>

typedef struct{
    float x;
    float y;
    float z;
    float angle;
} Coord_3D_t;

typedef struct{
    float r;
    float z;
    float angle;
} Coord_2D_t;

/* ========================= 接 口 函 数 声 明 ========================= */

void Arm_Kinematics_Init(void);
bool Arm_AimAtTarget(Coord_2D_t target, bool use_laser);
bool Arm_TF_TargetToBase(float depth, Coord_3D_t* target_base);
bool Arm_InverseKinematicsWithAim(float depth, float angle);
bool Arm_InverseKinematics(float x, float y, float z, float angle);
bool Arm_SetGripperAngle(float angle);

#endif
