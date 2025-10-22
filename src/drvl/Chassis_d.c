#include "Chassis_d.h"
#include "UART.h"
#include "LOG_s.h"
#include "Delay_s.h"
#include "PID_s.h"

#define CHASSIS_WHEEL_PERIMETER     26.69       // 轮周长(cm)
#define CHASSIS_WHEEL_TRACK         18.18f      // 轮距(cm)
#define CHASSIS_MAX_SPEED           100         // 最大速度(cm/s)
#define PI                          3.14159f

ChassisParam_t chassisParam;
PID_Param_t selfCtrlPID[CHASSIS_WHEEL_MAX];
JY61P_Data_t jy61pData;
PID_Param_t jy61pYawPID_selfCtrl;
PID_Param_t jy61pYawPID;

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
    USART_Printf(USART3, "$flash_reset#");      // 重置参数
    USART_Printf(USART3, "$mtype:1#");          // 520电机
    // USART_Printf(USART3, "$deadzone:#");        // PWM死区
    USART_Printf(USART3, "$mline:11#");         // 分辨率
    USART_Printf(USART3, "$mphase:56#");        // 减速比
    USART_Printf(USART3, "$wdiameter:85#");     // 轮子直径

    PID_Init(&jy61pYawPID);
    PID_SetPID(&jy61pYawPID, 2.0f, 0.01f, 0.5f);
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
        -param->speed[CHASSIS_WHEEL_RF],
        -param->speed[CHASSIS_WHEEL_LR],
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
        -param->pwm[CHASSIS_WHEEL_RF],
        -param->pwm[CHASSIS_WHEEL_LR],
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
}
/**
 * @brief 获取底盘数据
 * @param param 底盘参数结构体指针
 */
void Chassis_GetData(ChassisParam_t* const param)
{
    if(!param){
        _WARN("Chassis_GetData: param is NULL");
        return;
    }
    // 清空接收缓冲区
    while(USART_GetFlagStatus(USART3, USART_FLAG_RXNE) == SET){
        USART_ReceiveData(USART3);
    }
    // 发送指令并接受数据
    USART_Printf(USART3, "$upload:1,1,1#");
    char res[3][32] = { 0 };
    uint8_t index = 0;
    uint8_t line = 0;
    uint32_t timeout = 1000;
    while(timeout > 0){
        if(USART_GetFlagStatus(USART3, USART_FLAG_RXNE) == SET){
            char ch = USART_ReceiveData(USART3);
            if(index < sizeof(res[0]) - 1){
                res[line][index++] = ch;
            }
            if(ch == '#' || index >= sizeof(res[0]) - 1){
                res[line][index] = '\0';
                ++line;
                index = 0;
            }
            if(line >= 3) break;
        }
        else{
            Delay_us(100);
            --timeout;
        }
    }
    _INFO("Chassis_GetData: recv %s, %s, %s", res[0], res[1], res[2]);
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
    // USART_Printf(USART3, "$upload:0,1,0#");
    // char res[32] = {0};
    // uint8_t index = 0;
    // uint32_t timeout = 10000;
    // while(timeout > 0){
    //     if(USART_GetFlagStatus(USART3, USART_FLAG_RXNE) == SET){
    //         char ch = USART_ReceiveData(USART3);
    //         if(index < sizeof(res) - 1){
    //             res[index++] = ch;
    //         }
    //         if(ch == '#' || index >= sizeof(res) - 1){
    //             res[index] = '\0';
    //             index = 0;
    //             break;
    //         }
    //     }
    //     else{
    //         Delay_us(100);
    //         --timeout;
    //     }
    // }
    // _INFO("Chassis_GetData: recv %s", res);
    // if(timeout == 0){
    //     _WARN("Chassis_GetData: timeout");
    //     return;
    // }
    // // 解析接收的数据
    // sscanf(res, "$MALL:%d,%d,%d,%d#", &param->encorderAll[CHASSIS_WHEEL_LF], 
    //                                     &param->encorderAll[CHASSIS_WHEEL_RF], 
    //                                     &param->encorderAll[CHASSIS_WHEEL_LR], 
    //                                     &param->encorderAll[CHASSIS_WHEEL_RR]);
}
/**
 * @brief 移动底盘
 * @param linearVel 线速度(cm/s)
 * @param angularVel 角速度(度/秒)
 */
void Chassis_Move(int linearVel, int angularVel)
{
    // 角度转弧度
    float angularVelRad = angularVel * PI / 180.0f;
    // 逆运动学解算
    chassisParam.linearVel = linearVel / 10;
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
 * @param angle 角度(单位度, 顺时针<0, 逆时针>0)
 * @param angularVel 角速度(度/秒)
 */
void Chassis_Turn(int angle, int angularVel)
{
    if(angularVel == 0){
        _WARN("Chassis_Turn: angularVel is zero! Please use Chassis_GoStraight.");
        return;
    }
    uint32_t duration = (angle * 1000 < 0) ? (-angle * 1000 / angularVel) : (angle * 1000 / angularVel);
    if(angle < 0) angularVel = -angularVel;
    Chassis_Move(0, angularVel);

    Delay_ms(duration);

    Chassis_Stop();
}
/**
 * @brief 角度闭环转向运动
 * @param angle:角度
 */
void Chassis_Turn_JY61P(int angle)
{
    // PID闭环
    JY61p_GetData(&jy61pData);
    int currentAngle = jy61pData.angle_z;
    int targetAngle = currentAngle + angle;
    while(targetAngle >= 360) targetAngle -= 360;
    while(targetAngle < 0) targetAngle += 360;

    uint16_t timeout = 500;

    do{
        JY61p_GetData(&jy61pData);
        currentAngle = jy61pData.angle_z;
        int err = targetAngle - currentAngle;
        // 取最短路径
        while(err > 180) err -= 360;
        while(err < -180) err += 360;

        PID_Controller(&jy61pYawPID, 0, err, 0.001f);       // 1ms周期
        int outAngularVel = (int)jy61pYawPID.output;

        Chassis_Move(0, outAngularVel);
        Delay_ms(1);
    }
    while(--timeout && (currentAngle - targetAngle > 1 || currentAngle - targetAngle < -1));

    Chassis_Stop();
}
/**
 * @brief 非驱动板闭环控制初始化
 */
void Chassis_SelfCtrl_Init(void)
{
    for(uint8_t i = CHASSIS_WHEEL_LF; i < CHASSIS_WHEEL_MAX; ++i){
        PID_Init(&selfCtrlPID[i]);
        PID_SetPID(&selfCtrlPID[i], 0.1f, 0.001f, 0.05f);
        PID_SetLimit(&selfCtrlPID[i], 50.0f, 200.0f);
    }
    PID_Init(&jy61pYawPID_selfCtrl);
    PID_SetPID(&jy61pYawPID_selfCtrl, 2.0f, 0.01f, 0.5f);
    PID_SetLimit(&jy61pYawPID_selfCtrl, 80.0f, 300.0f);
}
/**
 * @brief 非驱动板闭环控制移动
 * @param linearVel 线速度
 * @param angularVel 角速度(度/秒)
 */
void Chassis_SelfCtrl_Move(int linearVel, int angularVel)
{
    // 角度转弧度
    float angularVelRad = angularVel * PI / 180.0f;
    // 逆运动学解算
    chassisParam.linearVel = linearVel / 10;
    chassisParam.angularVel = angularVelRad;
    Chassis_DifferentialIK(&chassisParam);
    // 自解算闭环控制速度
    Chassis_SelfCtrl(&chassisParam);
}
/**
 * @brief 非驱动板闭环控制停止
 */
void Chassis_SelfCtrl_Stop(void)
{
    Chassis_SelfCtrl_Move(0, 0);
}
/**
 * @brief 非驱动板闭环控制直线移动
 * @param speed 速度(前进>0, 后退<0)
 */
void Chassis_SelfCtrl_GoStraight(int speed)
{
    Chassis_SelfCtrl_Move(speed, 0);
}
/**
 * @brief 非驱动板闭环控制转向运动
 * @param angle 角度(单位度, 顺时针>0, 逆时针<0)
 * @param angularVel 角速度(度/秒)
 */
void Chassis_SelfCtrl_Turn(int angle, int angularVel)
{
    if(angularVel == 0){
        _WARN("Chassis_SelfCtrl_Turn: angularVel is zero! Please use Chassis_SelfCtrl_GoStraight.");
        return;
    }
    uint32_t duration = (angle * 1000 < 0) ? (-angle * 1000 / angularVel) : (angle * 1000 / angularVel);
    Chassis_SelfCtrl_Move(0, angularVel);
    Delay_ms(duration);
}
/**
 * @brief 非驱动板闭环控制转向运动(JY61P陀螺仪角度闭环)
 * @param relativeFlag 目标角度相对标志(1:相对当前角度, 0:绝对角度)
 * @param targetAngle 目标角度(单位度, 0~360)
 * @param tolerance 允许误差(单位度)
 * @param jyData JY61P数据结构体指针
 * @param timeout 超时时间(单位10ms)
 */
void Chassis_SelfCtrl_Turn_JY61(uint8_t relativeFlag, int targetAngle, uint8_t tolerance, JY61P_Data_t* const jyData, int timeout)
{
    if(!jyData){
        _WARN("Chassis_SelfCtrl_Turn_JY61: jyData is NULL!");
        return;
    }
    if(!tolerance) tolerance = 1;

    // 初始角度
    JY61p_GetData(jyData);
    float initAngle = jyData->angle_z;
    float currentAngle = initAngle;
    if(relativeFlag){
        targetAngle += initAngle;
        while(targetAngle >= 360.0f) targetAngle -= 360.0f;
        while(targetAngle < 0.0f) targetAngle += 360.0f;
    }
    // 向目标角度转动
    do{
        JY61p_GetData(jyData);
        currentAngle = jyData->angle_z;
        float err = targetAngle - currentAngle;

        PID_Controller(&jy61pYawPID_selfCtrl, 0, err, 0.01f);
        int targetAngularVel = (int)jy61pYawPID_selfCtrl.output;
        Chassis_SelfCtrl_Move(0, targetAngularVel);
        Delay_ms(10);
    }
    while(--timeout && ((int)(targetAngle - currentAngle) > tolerance || (int)(targetAngle - currentAngle) < -tolerance));


    Chassis_Stop();
    if(timeout == 0){
        _WARN("Chassis_SelfCtrl_Turn_JY61: timeout!");
    }
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
    // TODO： cm/s 转换为 车板单位+补偿系数
    leftVel *= 30;
    rightVel *= 30;
    // 写入数据(cm/s -> 车板单位+补偿系数)
    param->speed[CHASSIS_WHEEL_LF] = (int)leftVel;
    param->speed[CHASSIS_WHEEL_RF] = (int)rightVel;
    param->speed[CHASSIS_WHEEL_LR] = (int)leftVel;
    param->speed[CHASSIS_WHEEL_RR] = (int)rightVel;
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
    // 获取数据
    Chassis_GetData(param);
    for(uint8_t i = CHASSIS_WHEEL_LF; i < CHASSIS_WHEEL_MAX; ++i){
        param->encorderSpeed[i] = param->encorder10ms[i] * CHASSIS_WHEEL_PERIMETER / 24.64f;
    }
    // 正运动学解算
    param->linearVel = (param->encorderSpeed[CHASSIS_WHEEL_RF] + param->encorderSpeed[CHASSIS_WHEEL_LF]) / 2.0f;
    param->angularVel = (param->encorderSpeed[CHASSIS_WHEEL_RF] - param->encorderSpeed[CHASSIS_WHEEL_LF]) / CHASSIS_WHEEL_TRACK;
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
        param->encorderSpeed[i] = param->encorder10ms[i] * CHASSIS_WHEEL_PERIMETER / 24.64f;
    }
    // 计算PWM
    for(uint8_t i = CHASSIS_WHEEL_LF; i < CHASSIS_WHEEL_MAX; ++i){
        PID_Controller(&selfCtrlPID[i], param->speed[i], param->encorder10ms[i], 0.01f);
        param->pwm[i] = selfCtrlPID[i].output;
    }
    Chassis_SetPWM(param);
}
