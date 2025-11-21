#include "PID_s.h"
#include "s_LOG.h"

#define _constrain(val, min, max)  ((val) > (max) ? (max) : ((val) < (min) ? (min) : (val)))

/* ========================= 接 口 函 数 实 现 ========================= */

/**
 * @brief PID控制器初始化函数
 * @param PID 指向PID类实例的指针
 */
void PID_Init(PID_Param_t* const PID)
{
    if(PID == 0)
        return;

    PID->output = 0.0f;
    PID->integral = 0.0f;
    PID->lastActual = 0.0f;
    PID->lastDiff = 0.0f;

    PID->p = 0.0f;
    PID->i = 0.0f;
    PID->d = 0.0f;
    PID->alpha = 1.0f;
    PID->deadZone = 0.0f;
    PID->intLimit = 0.0f;
    PID->outputLimit = 0.0f;
    PID->intThreshold = 0.0f;
}
/**
 * @brief PID核心参数配置函数
 * @note 用于配置比例P、积分I和微分D三项参数
 * @param PID 指向PID类实例的指针
 * @param p P参数
 * @param i I参数
 * @param d D参数
 */
void PID_SetPID(PID_Param_t* const PID, float p, float i, float d)
{
    if(PID == 0)
        return;
    PID->p = p;
    PID->i = i;
    PID->d = d;
}
/**
 * @brief PID限幅参数配置函数
 * @note 配置积分项和输出项的最大允许范围，防止积分饱和和控制量过大
 * @param PID 指向PID类实例的指针
 * @param intLimit 积分限幅
 * @param outputLimit 输出限幅
 */
void PID_SetLimit(PID_Param_t* const PID, float intLimit, float outputLimit)
{
    if(PID == 0)
        return;
    PID->intLimit = intLimit;
    PID->outputLimit = outputLimit;
}
/**
 * @brief PID滤波与阈值参数配置函数
 * @note 配置微分项滤波系数、积分分离阈值和死区范围，用于增强控制器的稳定性和鲁棒性
 * @param PID 指向PID类实例的指针
 * @param alpha 微分项滤波系数(0 <= alpha <= 1)
 * @param intThreshold 积分分离阈值
 * @param deadZone 死区范围
 */
void PID_SetFilter(PID_Param_t* const PID, float alpha, float intThreshold, float deadZone)
{
    if(PID == 0)
        return;
    PID->alpha = alpha;
    PID->intThreshold = intThreshold;
    PID->deadZone = deadZone;
}
/**
 * @brief PID控制器
 * @param PID 指向PID类实例的指针
 * @param target 目标值
 * @param actual 实际值
 * @param dt_s 控制周期(秒)
 */
void PID_Controller(PID_Param_t* const PID, float target, float actual, float dt_s)
{
    if(PID == 0)
        return;

    if(dt_s <= 1e-6f) dt_s = 1e-6f;
    else if(dt_s > 0.1f) dt_s = 0.1f;
    // 比例项
    float err = target - actual;
    if(err < PID->deadZone && err > -PID->deadZone){
        PID->output = 0.0f;
        return;
    }
    // 微分项
    float diff = -(actual - PID->lastActual) / dt_s;
    PID->lastActual = actual;
    float outDiff = PID->alpha * diff + (1.0f - PID->alpha) * PID->lastDiff;
    PID->lastDiff = outDiff;
    // 积分项
    PID->integral += err * dt_s;
    _constrain(PID->integral, -PID->intLimit, PID->intLimit);
    if(PID->intThreshold > 0.0f){
        if(err > PID->intThreshold || err < -PID->intThreshold){
            PID->integral = 0.0f;
        }
    }
    // PID输出
    PID->output = PID->p * err + PID->i * PID->integral + PID->d * outDiff;
    _constrain(PID->output, -PID->outputLimit, PID->outputLimit);
}
/**
 * @brief PI控制器
 * @param PID 指向PID类实例的指针
 * @param target 目标值
 * @param actual 实际值
 */
void PI_Controller(PID_Param_t* const PID, float target, float actual)
{
    // 比例项
    float err = target - actual;
    // 积分项
    PID->integral += err;
    _constrain(PID->integral, -PID->intLimit, PID->intLimit);
    // PID输出
    PID->output = PID->p * err + PID->i * PID->integral;
    _constrain(PID->output, -PID->outputLimit, PID->outputLimit);
}
/**
 * @brief 前馈PID控制器
 * @param PID 指向PID类实例的指针
 * @param target 目标值
 * @param actual 实际值
 * @param feedForward 前馈值
 * @param dt_s 控制周期(秒)
 */
void PID_ControllerWithFF(PID_Param_t* const PID, float target, float actual, float feedForward, float dt_s)
{
    PID_Controller(PID, target, actual, dt_s);
    PID->output += feedForward;
    _constrain(PID->output, -PID->outputLimit, PID->outputLimit);
}
/**
 * @brief PID类成员数值打印函数
 * @param PID
 */
void PID_InfoParam(PID_Param_t* const PID)
{
    if(PID == 0)
        return;
    _INFO("<---------- PID Info ---------->");
    _INFO("P: %f; I: %f; D: %f", PID->p, PID->i, PID->d);
    _INFO("output: %f", PID->output);
    _INFO("integral: %f", PID->integral);
    _INFO("lastActual: %f", PID->lastActual);
    _INFO("lastDiff: %f", PID->lastDiff);
    _INFO("alpha: %f", PID->alpha);
    _INFO("intLimit: %f", PID->intLimit);
    _INFO("outputLimit: %f", PID->outputLimit);
    _INFO("intThreshold: %f", PID->intThreshold);
    _INFO("deadZone: %f", PID->deadZone);
    _INFO("<------------------------------->");
}
