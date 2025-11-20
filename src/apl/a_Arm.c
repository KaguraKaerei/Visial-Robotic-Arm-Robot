#include "a_Arm.h"
#include "s_Kinematics.h"
#include "UART.h"

/* ========================= 接 口 变 量 声 明 ========================= */

Arm_Param_t arm_param = { 0 };
Arm_State_t arm_state = ARM_STATE_IDLE;

/* ========================= 私 有 变 量 声 明 ========================= */



/* ========================= 私 有 函 数 声 明 ========================= */



/* ========================= 接 口 函 数 实 现 ========================= */

void Arm_Process(void)
{
    bool res = false;

    switch(arm_state){
        case ARM_STATE_IDLE:
            break;
        case ARM_FIND_QR:
        {
            // 寻找二维码期间保持不变, 直到找到二维码后会收到信息切换到ARM_FIND_TASK状态
            Arm_InverseKinematics(0.0f, -0.10f, 0.10f, -30.0f);
            break;
        }
        case ARM_FIND_TASK:
        {
            // 寻找任务期间保持不变, 直到找到任务物体后会切换到对应的任务状态
            Arm_InverseKinematics(0.0f, 0.10f, 0.10f, -30.0f);
            break;
        }
        case ARM_STATE_AIM_TARGET:
        {
            Coord_2D_t target;
            target.r = arm_param.r;
            target.z = arm_param.z;
            target.angle = 0.0f;
            res = Arm_AimAtTarget(target, false);
            if(res) USART_Printf(USART2, "$ARM:AIM_AT_OK#");
            else USART_Printf(USART2, "$ARM:AIM_AT_ERR#");
            arm_state = ARM_STATE_IDLE;
            break;
        }
        case ARM_STATE_AIM_LASER:
        {
            Coord_2D_t laser_target;
            laser_target.r = arm_param.r;
            laser_target.z = arm_param.z;
            laser_target.angle = 0.0f;
            res = Arm_AimAtTarget(laser_target, true);
            if(res) USART_Printf(USART2, "$ARM:AIM_AT_OK#");
            else USART_Printf(USART2, "$ARM:AIM_AT_ERR#");
            arm_state = ARM_STATE_IDLE;
            break;
        }
        case ARM_STATE_TARGET:
        {
            res = Arm_InverseKinematicsWithAim(arm_param.depth, 0.0f);
            if(res) USART_Printf(USART2, "$ARM:TARGET_OK#");
            else USART_Printf(USART2, "$ARM:TARGET_ERR#");
            arm_state = ARM_STATE_IDLE;
            break;
        }
        case ARM_STATE_GRASPING:
        {
            // TODO: 抓住时实时判断一半是不是白色（是否抓住）并移动到指定地方
            Arm_SetGripperAngle(30.0f);     // 抓住
            arm_state = ARM_STATE_IDLE;
            break;
        }
        case ARM_STATE_PUT_IN:
        {
            Arm_SetGripperAngle(90.0f);     // 放下
            arm_state = ARM_STATE_IDLE;
            break;
        }
        case ARM_STATE_ANY:
        {
            res = Arm_InverseKinematics(arm_param.x, arm_param.y, arm_param.z, arm_param.angle);
            if(res) USART_Printf(USART2, "$ARM:ANY_OK#");
            else USART_Printf(USART2, "$ARM:ANY_ERR#");
            arm_state = ARM_STATE_IDLE;
            break;
        }
        case ARM_STATE_END:
        {
            Arm_Kinematics_Init();
            arm_state = ARM_STATE_IDLE;
            break;
        }
    }
}

/* ========================= 私 有 函 数 实 现 ========================= */


