#include "Servo_d.h"

/* ========================= 私 有 变 量 声 明 ========================= */

#define SERVO_MIN_CCR   500
#define SERVO_MID_CCR   1500
#define SERVO_MAX_CCR   2500

/* ========================= 私 有 函 数 声 明 ========================= */


/* ========================= 接 口 函 数 实 现 ========================= */

void Servo_Init(void)
{
    // TODO:默认角度
    TIM_SetCompare1(TIM2, SERVO_MID_CCR);
    TIM_SetCompare2(TIM2, SERVO_MID_CCR);
    TIM_SetCompare2(TIM3, SERVO_MID_CCR);
    TIM_SetCompare3(TIM3, SERVO_MID_CCR);
    TIM_SetCompare4(TIM3, SERVO_MID_CCR);
    TIM_SetCounter(TIM2, 0);
    TIM_SetCounter(TIM3, 0);
}

bool Servo_SetCCR(Servo_ID_t servo_id, uint16_t ccr)
{
    bool res = true;
    if(ccr < SERVO_MIN_CCR){ ccr = SERVO_MIN_CCR; res = false; }
    if(ccr > SERVO_MAX_CCR){ ccr = SERVO_MAX_CCR; res = false; }

    switch(servo_id){
        case SERVO_CHASSIS:
            TIM_SetCompare1(TIM2, ccr);
            break;
        case SERVO_JOINT_1:
            TIM_SetCompare2(TIM2, ccr);
            break;
        case SERVO_JOINT_2:
            TIM_SetCompare2(TIM3, ccr);
            break;
        case SERVO_JOINT_3:
            TIM_SetCompare3(TIM3, ccr);
            break;
        case SERVO_JOINT_GRIPPER:
            TIM_SetCompare4(TIM3, ccr);
            break;
        default:
            break;
    }

    return res;
}

uint16_t Servo_GetCCR(Servo_ID_t servo_id)
{
    uint16_t ccr = 0;
    switch(servo_id){
        case SERVO_CHASSIS:
            ccr = TIM2->CCR1;
            break;
        case SERVO_JOINT_1:
            ccr = TIM2->CCR2;
            break;
        case SERVO_JOINT_2:
            ccr = TIM3->CCR2;
            break;
        case SERVO_JOINT_3:
            ccr = TIM3->CCR3;
            break;
        case SERVO_JOINT_GRIPPER:
            ccr = TIM3->CCR4;
            break;
        default:
            break;
    }

    return ccr;
}

/* ========================= 私 有 函 数 实 现 ========================= */
