#include "sysTick.h"

static volatile sysTick_t sysTick_ms = 0;

/**
  * @brief SysTick初始化，配置为1ms心跳
  * @note 假设系统时钟为72MHz，SysTick时钟源为HCLK
  */
void sysTick_Init(void)
{
    // 配置SysTick为1ms中断
    SysTick_Config(72000);
    // 设置SysTick中断优先级为最低
    NVIC_SetPriority(SysTick_IRQn, 15);
    // 重置计数器
    sysTick_ms = 0;
}
/**
  * @brief SysTick中断处理函数
  * @note 每1ms调用一次
  */
void SysTick_Handler(void)
{
    sysTick_ms++;
}
/**
  * @brief 获取系统心跳计数（ms）
  */
sysTick_t sysTick_GetMs(void)
{
    return sysTick_ms;
}
/**
  * @brief 获取系统心跳计数（s）
  */
sysTick_t sysTick_GetS(void)
{
    return sysTick_ms / 1000;
}
/**
  * @brief 检查是否超时
  */
uint8_t sysTick_IsTimeout(sysTick_t start_time, uint32_t timeout_ms)
{
    sysTick_t current_time = sysTick_GetMs();
    // 处理时间溢出的情况
    if(current_time >= start_time){
        return (current_time - start_time) >= timeout_ms ? 1 : 0;
    }
    else{
        // 时间溢出了（约49.7天后会溢出）
        return ((0xFFFFFFFF - start_time) + current_time + 1) >= timeout_ms ? 1 : 0;
    }
}
