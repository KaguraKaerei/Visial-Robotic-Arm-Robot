#include "DWT.h"

/**
  * @brief DWT初始化，作为微秒级心跳
  * @note 系统时钟为72MHz，DWT时钟源为HCLK
  */
void DWT_Init(void)
{
    // 使能DWT
    if(!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)){
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    }
    // 使能循环计数器
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    // 清零计数器
    DWT->CYCCNT = 0;
}
/**
  * @brief DWT获取微秒级时间
  * @return 返回当前时间（单位：微秒）
  */
uint32_t DWT_GetUs(void)
{
    return DWT->CYCCNT / CPU_FREQ_MHZ;
}
/**
  * @brief DWT检查是否超时
  * @param start_time 起始时间（单位：微秒）
  * @param timeout_us 超时时间（单位：微秒）
  * @return 超时状态（1：超时，0：未超时）
  */
uint8_t DWT_IsTimeout(uint32_t start_time, uint32_t timeout_us)
{
    uint32_t current_time = DWT_GetUs();
    // 处理时间溢出的情况
    if(current_time >= start_time){
        return (current_time - start_time) >= timeout_us ? 1 : 0;
    }
    else{
        // 时间溢出了（约59.6s后会溢出）
        return ((0xFFFFFFFF - start_time) + current_time + 1) >= timeout_us ? 1 : 0;
    }
}
