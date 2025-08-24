#include "Delay.h"

/**
  * @brief 阻塞延时 微秒级
  */
void Delay_us(uint32_t us)
{
    uint32_t start_time = DWT_GetUs();
    while(!DWT_IsTimeout(start_time, us));
}
/**
  * @brief 阻塞延时 毫秒级
  */
void Delay_ms(uint32_t ms)
{
    sysTick_t start_time = sysTick_GetMs();
    while(!sysTick_IsTimeout(start_time, ms));
}
/**
  * @brief 阻塞延时 秒级
  */
void Delay_s(uint32_t s)
{
    sysTick_t start_time = sysTick_GetMs();
    while(!sysTick_IsTimeout(start_time, s * 1000));
}
/**
  * @brief 非阻塞延时 微秒级
  */
uint8_t DWT_DelayUs(uint32_t* start_time, uint32_t interval_us)
{
    if(*start_time == 0){
        *start_time = DWT_GetUs();
        return 0;
    }
    else if(DWT_IsTimeout(*start_time, interval_us)){
        *start_time = 0;
        return 1;
    }
    
    return 0;
}
/**
  * @brief 非阻塞延时 毫秒级
  */
uint8_t sysTick_DelayMs(sysTick_t* start_time, uint32_t interval_ms)
{
    if(*start_time == 0){
        *start_time = sysTick_GetMs();
        return 0;
    }
    else if(sysTick_IsTimeout(*start_time, interval_ms)){
        *start_time = 0;
        return 1;
    }
    
    return 0;
}
