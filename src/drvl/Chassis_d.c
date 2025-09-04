#include "Chassis_d.h"
#include "UART.h"
#include "LOG_s.h"
#include "Delay_s.h"
#include "PID_s.h"

#define CHASSIS_WHEEL_PERIMETER     26.69       // 轮周长(cm)
#define CHASSIS_WHEEL_TRACK         170.0f      // 轮距
#define CHASSIS_MAX_SPEED           1000        // 最大速度
#define PI                          3.14159f

ChassisParam_t chassisParam;
PID_Param_t selfCtrlPID[CHASSIS_WHEEL_MAX];

/* ========================= 私 有 函 数 声 明 ========================= */

static void Chassis_DifferentialIK(ChassisParam_t* const param);
static void Chassis_DifferentialFK(ChassisParam_t* const param);
static void Chassis_SelfCtrl(ChassisParam_t* const param);


/* ========================= 接 口 函 数 实 现 ========================= */

/**
 * @brief 底盘初始化
 */
void Chassis_Init(void)
{
    // 初始化串口
    iUSART_Init(iUSART3, USART_MODE_BASIC);
    // 配置底盘
    USART_Printf(USART3, "$mtype:1#");          // 520电机
    // USART_Printf(USART3, "$deadzone:#");        // PWM死区
    USART_Printf(USART3, "$mline:500#");        // 分辨率
    USART_Printf(USART3, "$mphase:30#");        // 减速比
    USART_Printf(USART3, "$wdiameter:85#");     // 轮子直径
}
/**
 * @brief 设置底盘速度
 * @param param 底盘参数结构体指针
 */
void Chassis_SetSpeed(const ChassisParam_t* const param)
{
    if(!param){
        _WARN("Chassis_SetSpeed: param is NULL");
        return;
    }
    USART_Printf(USART3, "$spd:%d,%d,%d,%d#", param->speed[CHASSIS_WHEEL_LF],
                                                param->speed[CHASSIS_WHEEL_RF],
                                                param->speed[CHASSIS_WHEEL_LR],
                                                param->speed[CHASSIS_WHEEL_RR]);
    _INFO("Chassis speed set: LF=%d, RF=%d, LR=%d, RR=%d", param->speed[CHASSIS_WHEEL_LF],
                                                            param->speed[CHASSIS_WHEEL_RF],
                                                            param->speed[CHASSIS_WHEEL_LR],
                                                            param->speed[CHASSIS_WHEEL_RR]);
}
/**
 * @brief 设置底盘PWM
 * @param param 底盘参数结构体指针
 */
void Chassis_SetPWM(const ChassisParam_t* const param)
{
    if(!param){
        _WARN("Chassis_SetPWM: param is NULL");
        return;
    }
    USART_Printf(USART3, "$pwm:%u,%u,%u,%u#", param->pwm[CHASSIS_WHEEL_LF],
                                        param->pwm[CHASSIS_WHEEL_RF],
                                        param->pwm[CHASSIS_WHEEL_LR],
                                        param->pwm[CHASSIS_WHEEL_RR]);
    _INFO("Chassis PWM set: LF=%u, RF=%u, LR=%u, RR=%u", param->pwm[CHASSIS_WHEEL_LF],
                                                    param->pwm[CHASSIS_WHEEL_RF],
                                                    param->pwm[CHASSIS_WHEEL_LR],
                                                    param->pwm[CHASSIS_WHEEL_RR]);
}
/**
 * @brief 设置底盘PID
 * @param param 底盘参数结构体指针
 */
void Chassis_SetPID(const ChassisParam_t* const param)
{
    if(!param){
        _WARN("Chassis_SetPID: param is NULL");
        return;
    }
    USART_Printf(USART3, "$MPID:%f,%f,%f#", param->p, param->i, param->d);
    _INFO("Chassis PID set: P=%f, I=%f, D=%f", param->p, param->i, param->d);
}
/**
 * @brief 获取底盘数据
 * @param param 底盘参数结构体指针
 */
void Chassis_GetData(ChassisParam_t* const param)
{
    // 清空接收缓冲区
    while(USART_GetFlagStatus(USART3, USART_FLAG_RXNE) == SET){
        USART_ReceiveData(USART3);
    }
    // 发送指令并接受数据
    USART_Printf(USART3, "$upload:1,1,1#");
    char res[3][32] = {0};
    uint8_t index = 0;
    uint8_t line = 0;
    uint32_t timeout = 10000;
    while(--timeout){
        if(USART_GetFlagStatus(USART3, USART_FLAG_RXNE) == SET){
            char ch = USART_ReceiveData(USART3);
            res[line][index++] = ch;
            if(ch == '#' || index >= sizeof(res[0]) - 1){
                res[line][index] = '\0';
                ++line;
                index = 0;
            }
            if(line >= 3) break;
        }
    }
    if(timeout == 0){
        _WARN("Chassis_GetData: timeout");
        return;
    }
    // 解析接收的数据
    sscanf(res[0], "$MALL:%d,%d,%d,%d#", &param->encorderAll[CHASSIS_WHEEL_LF], 
                                        &param->encorderAll[CHASSIS_WHEEL_RF], 
                                        &param->encorderAll[CHASSIS_WHEEL_LR], 
                                        &param->encorderAll[CHASSIS_WHEEL_RR]);
    sscanf(res[1], "$MTEP:%d,%d,%d,%d#", &param->encorder10ms[CHASSIS_WHEEL_LF], 
                                        &param->encorder10ms[CHASSIS_WHEEL_RF], 
                                        &param->encorder10ms[CHASSIS_WHEEL_LR], 
                                        &param->encorder10ms[CHASSIS_WHEEL_RR]);
    sscanf(res[2], "$MSPD:%d,%d,%d,%d#", &param->encorderSpeed[CHASSIS_WHEEL_LF], 
                                        &param->encorderSpeed[CHASSIS_WHEEL_RF], 
                                        &param->encorderSpeed[CHASSIS_WHEEL_LR], 
                                        &param->encorderSpeed[CHASSIS_WHEEL_RR]);
    _INFO("Chassis_GetEncorderAll: LF=%d, RF=%d, LR=%d, RR=%d", param->encorderAll[CHASSIS_WHEEL_LF], 
                                                                param->encorderAll[CHASSIS_WHEEL_RF], 
                                                                param->encorderAll[CHASSIS_WHEEL_LR], 
                                                                param->encorderAll[CHASSIS_WHEEL_RR]);
    _INFO("Chassis_GetEncorder10ms: LF=%d, RF=%d, LR=%d, RR=%d", param->encorder10ms[CHASSIS_WHEEL_LF], 
                                                                param->encorder10ms[CHASSIS_WHEEL_RF], 
                                                                param->encorder10ms[CHASSIS_WHEEL_LR], 
                                                                param->encorder10ms[CHASSIS_WHEEL_RR]);
    _INFO("Chassis_GetSpeed: LF=%d, RF=%d, LR=%d, RR=%d", param->encorderSpeed[CHASSIS_WHEEL_LF], 
                                                        param->encorderSpeed[CHASSIS_WHEEL_RF], 
                                                        param->encorderSpeed[CHASSIS_WHEEL_LR], 
                                                        param->encorderSpeed[CHASSIS_WHEEL_RR]);
}
/**
 * @brief 移动底盘
 * @param linearVel 线速度
 * @param angularVel 角速度(度/秒)
 */
void Chassis_Move(int linearVel, int angularVel)
{
    // 角度转弧度
    float angularVelRad = angularVel * PI / 180.0f;
    // 逆运动学解算
    chassisParam.linearVel = linearVel;
    chassisParam.angularVel = angularVelRad;
    Chassis_DifferentialIK(&chassisParam);
    // 设置速度
    Chassis_SetSpeed(&chassisParam);
}
/**
 * @brief 停止底盘
 */
void Chassis_Stop(void)
{
    Chassis_Move(0, 0);
}
/**
 * @brief 直线移动
 * @param speed 速度(前进>0, 后退<0)
 */
void Chassis_GoStraight(int speed)
{
    Chassis_Move(speed, 0);
}
/**
 * @brief 转向运动
 * @param angle 角度(单位度, 顺时针>0, 逆时针<0)
 * @param angularVel 角速度(度/秒)
 */
void Chassis_Turn(int angle, int angularVel)
{
    if(angularVel == 0){
        _WARN("Chassis_Turn: angularVel is zero! Please use Chassis_GoStraight.");
        return;
    }
    uint32_t duration = (angle * 1000 < 0) ? (-angle * 1000 / angularVel) : (angle * 1000 / angularVel);
    Chassis_Move(0, angularVel);
    Delay_ms(duration);
}
/**
 * @brief 非驱动板闭环控制初始化
 */
void Chassis_SelfCtrl_Init(void)
{
    for(uint8_t i = CHASSIS_WHEEL_LF; i < CHASSIS_WHEEL_MAX; ++i){
        PID_Init(&selfCtrlPID[i]);
        PID_SetPID(&selfCtrlPID[i], 0.1f, 0.001f, 0.05f);
        PID_SetLimit(&selfCtrlPID[i], 100.0f, 1000.0f);
    }
}
/**
 * @brief 
 */
void Chassis_SelfCtrl_Move(int linearVel, int angularVel)
{
    // 角度转弧度
    float angularVelRad = angularVel * PI / 180.0f;
    // 逆运动学解算
    chassisParam.linearVel = linearVel;
    chassisParam.angularVel = angularVelRad;
    Chassis_DifferentialIK(&chassisParam);
    // 自解算闭环控制速度
    Chassis_SelfCtrl(&chassisParam);
}

/* ========================= 私 有 函 数 实 现 ========================= */

/**
 * @brief 差速驱动逆运动学解算
 * @param param 底盘参数结构体指针
 * @param param.linearVel   期望线速度
 * @param param.angularVel  期望角速度
 */
static void Chassis_DifferentialIK(ChassisParam_t* const param)
{
    if(!param){
        _WARN("Chassis_DifferentialIK: param is NULL");
        return;
    }

    // 逆运动学解算
    float leftVel = param->linearVel - (param->angularVel * CHASSIS_WHEEL_TRACK / 2.0f);
    float rightVel = param->linearVel + (param->angularVel * CHASSIS_WHEEL_TRACK / 2.0f);
    // 速度限幅
    leftVel = leftVel > CHASSIS_MAX_SPEED ? CHASSIS_MAX_SPEED : (leftVel < -CHASSIS_MAX_SPEED ? -CHASSIS_MAX_SPEED : leftVel);
    rightVel = rightVel > CHASSIS_MAX_SPEED ? CHASSIS_MAX_SPEED : (rightVel < -CHASSIS_MAX_SPEED ? -CHASSIS_MAX_SPEED : rightVel);
    // 写入数据
    param->speed[CHASSIS_WHEEL_LF] = (int)leftVel;
    param->speed[CHASSIS_WHEEL_RF] = (int)rightVel;
    param->speed[CHASSIS_WHEEL_LR] = (int)leftVel;
    param->speed[CHASSIS_WHEEL_RR] = (int)rightVel;
    // 打印
    _INFO("Chassis_DifferentialIK: LF=%d, RF=%d, LR=%d, RR=%d", 
          param->speed[CHASSIS_WHEEL_LF], 
          param->speed[CHASSIS_WHEEL_RF], 
          param->speed[CHASSIS_WHEEL_LR], 
          param->speed[CHASSIS_WHEEL_RR]);
}
/**
 * @brief 差速驱动正运动学解算
 * @param param 底盘参数结构体指针
 * @param param.speed[]   各轮速度
 */
static void Chassis_DifferentialFK(ChassisParam_t* const param)
{
    if(!param){
        _WARN("Chassis_DifferentialFK: param is NULL");
        return;
    }

    // 正运动学解算
    param->linearVel = (param->encorderSpeed[CHASSIS_WHEEL_RF] + param->encorderSpeed[CHASSIS_WHEEL_LF]) / 2.0f;
    param->angularVel = (param->encorderSpeed[CHASSIS_WHEEL_RF] - param->encorderSpeed[CHASSIS_WHEEL_LF]) / CHASSIS_WHEEL_TRACK;
    // 打印
    _INFO("Chassis_DifferentialFK: linearVel=%d, angularVel=%d", (int)param->linearVel, (int)param->angularVel);
}
/**
 * @brief 非驱动板闭环控制
 * @param param 底盘参数结构体指针
 */
static void Chassis_SelfCtrl(ChassisParam_t* const param)
{
    if(!param){
        _WARN("Chassis_SelfCtrl: param is NULL");
        return;
    }
    // 获取数据
    Chassis_GetData(param);
    for(uint8_t i = CHASSIS_WHEEL_LF; i < CHASSIS_WHEEL_MAX; ++i){
        param->encorderSpeed[i] = param->encorder10ms[i] * CHASSIS_WHEEL_PERIMETER / 600.0f;
    }
    // 计算PWM
    for(uint8_t i = CHASSIS_WHEEL_LF; i < CHASSIS_WHEEL_MAX; ++i){
        PID_Controller(&selfCtrlPID[i], param->speed[i], param->encorder10ms[i], 0.01f);
        param->pwm[i] = selfCtrlPID[i].output;
    }
    Chassis_SetPWM(param);
}
