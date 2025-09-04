#ifndef _PID_S_H_
#define _PID_S_H_

#include "stm32f10x.h"

/* ========================= P I D 类 定 义 ========================= */

typedef struct{
    float p, i, d;
    float output;
    float integral;
    float lastActual, lastDiff;
    float alpha;
    float intLimit, outputLimit;
    float intThreshold;
    float deadZone;
} PID_Param_t;

/* ========================= 接 口 函 数 实 现 ========================= */

void PID_Init(PID_Param_t* const PID);
void PID_SetPID(PID_Param_t* const PID, float p, float i, float d);
void PID_SetLimit(PID_Param_t* const PID, float intLimit, float outputLimit);
void PID_SetFilter(PID_Param_t* const PID, float alpha, float intThreshold, float deadZone);
void PID_Controller(PID_Param_t* const PID, float target, float actual, float dt_s);
void PID_ControllerWithFF(PID_Param_t* const PID, float target, float actual, float feedForward, float dt_s);
void PID_InfoParam(PID_Param_t* const PID);

#endif
