#ifndef _DWT_H_
#define _DWT_H_

#include "stm32f10x.h"
// 定义CPU频率（单位：MHz）
#define CPU_FREQ_MHZ    72
// DWT寄存器地址
#ifndef DWT_BASE
#define DWT_BASE        (0xE0001000UL)
#define DWT             ((DWT_Type*)DWT_BASE)
// 地址以字节为单位，寄存器以位为单位
typedef struct{
    volatile uint32_t CTRL;     // 控制寄存器：0xE0001000
    volatile uint32_t CYCCNT;   // CPU周期计数器：0xE0001004
    volatile uint32_t CPICNT;   // CPU指令计数器：0xE0001008
    volatile uint32_t EXCCNT;   // 异常计数器：0xE000100C
    volatile uint32_t SLEEPCNT; // 睡眠计数器：0xE0001010
    volatile uint32_t LSUCNT;   // 低功耗计数器：0xE0001014
    volatile uint32_t FOLDCNT;  // 折叠计数器：0xE0001018
    volatile uint32_t PCSR;     // 程序计数器状态寄存器：0xE000101C
}DWT_Type;

#endif

#ifndef DWT_CTRL_CYCCNTENA_Msk
#define DWT_CTRL_CYCCNTENA_Msk  (1UL << 0)
#endif

#ifndef CoreDebug_DEMCR_TRCENA_Msk  
#define CoreDebug_DEMCR_TRCENA_Msk  (1UL << 24)
#endif

void DWT_Init(void);
uint32_t DWT_GetUs(void);
uint8_t DWT_IsTimeout(uint32_t start_time, uint32_t timeout_us);

#endif
