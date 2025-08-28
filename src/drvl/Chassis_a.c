#include "Chassis_a.h"
#include "UART.h"
#include "LOG_s.h"

ChassisParam_t chassisParam;

/**
 * @brief 底盘初始化
 */
void Chassis_Init(void)
{
    // 初始化串口
    iUSART_Init(iUSART3, USART_MODE_BASIC);
    // 配置底盘
    USART_Printf(USART3, "$mtype:1#");      // 520电机
    // USART_Printf(USART3, "$deadzone:#")     // PWM死区
    USART_Printf(USART3, "$mline:30#");      // 减速比
    USART_Printf(USART3, "$wdiameter:50#");  // 轮子直径

    _INFO("Chassis initialized");
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
    sscanf(res[2], "$MSPD:%d,%d,%d,%d#", &param->speed[CHASSIS_WHEEL_LF], 
                                        &param->speed[CHASSIS_WHEEL_RF], 
                                        &param->speed[CHASSIS_WHEEL_LR], 
                                        &param->speed[CHASSIS_WHEEL_RR]);
    _INFO("Chassis_GetEncorderAll: LF=%d, RF=%d, LR=%d, RR=%d", param->encorderAll[CHASSIS_WHEEL_LF], 
                                                                param->encorderAll[CHASSIS_WHEEL_RF], 
                                                                param->encorderAll[CHASSIS_WHEEL_LR], 
                                                                param->encorderAll[CHASSIS_WHEEL_RR]);
    _INFO("Chassis_GetEncorder10ms: LF=%d, RF=%d, LR=%d, RR=%d", param->encorder10ms[CHASSIS_WHEEL_LF], 
                                                                param->encorder10ms[CHASSIS_WHEEL_RF], 
                                                                param->encorder10ms[CHASSIS_WHEEL_LR], 
                                                                param->encorder10ms[CHASSIS_WHEEL_RR]);
    _INFO("Chassis_GetSpeed: LF=%d, RF=%d, LR=%d, RR=%d", param->speed[CHASSIS_WHEEL_LF], 
                                                        param->speed[CHASSIS_WHEEL_RF], 
                                                        param->speed[CHASSIS_WHEEL_LR], 
                                                        param->speed[CHASSIS_WHEEL_RR]);
}
