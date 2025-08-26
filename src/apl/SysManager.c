#include "SysManager.h"

// 私有变量
static SysState_t sysState = SYS_STATE_INIT;
static SysCheck_t sysCheck = SYS_CHECK_USART;
static bool sysCheckResults[SYS_CHECK_MAX] = {false};
// 私有函数声明
static void SysInit(void);
static bool SysCheck_USART(void);
static bool SysCheck_TIM(void);
static bool SysCheck_EXTI(void);
static bool SysCheck_I2C(void);
static bool SysCheck_SYSTICK(void);
static bool SysCheck_DWT(void);

/**
 * @brief 系统管理器初始化函数
 */
void SysManager_Init(void)
{
    sysState = SYS_STATE_INIT;
    sysCheck = SYS_CHECK_USART;
    for(uint8_t i = 0; i < SYS_CHECK_MAX; ++i){
        sysCheckResults[i] = false;
    }

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_ResetBits(GPIOC, GPIO_Pin_13);
}
/**
 * @brief 系统管理器进程函数
 */
void SysManager_Process(void)
{
    switch(sysState)
    {
        case SYS_STATE_INIT:
            SysInit();
            break;
        case SYS_STATE_READY:
            _INFO("System Ready!");
            GPIO_SetBits(GPIOC, GPIO_Pin_13);
            sysState = SYS_STATE_RUNNING;
            break;
        case SYS_STATE_RUNNING:
            
            break;
        case SYS_STATE_ERROR:
            _ERROR("System Error!");
            break;
        default:
            break;
    }
}
/**
 * 
 */
bool System_Is_Ready(void)
{
    return sysState == SYS_STATE_RUNNING;
}

/**
 * @brief 系统初始化函数
 */
void SysInit(void)
{
    bool(* checkFunctions[SYS_CHECK_MAX])(void) = {
        SysCheck_USART,
        SysCheck_TIM,
        SysCheck_EXTI,
        SysCheck_I2C,
        SysCheck_SYSTICK,
        SysCheck_DWT
    };
    const char* checkNames[SYS_CHECK_MAX] = {
        "USART", "TIM", "EXTI", "I2C", "SYSTICK", "DWT"
    };
    
    for( ; sysCheck < SYS_CHECK_MAX; ++sysCheck){
        sysCheckResults[sysCheck] = checkFunctions[sysCheck]();
        if(!sysCheckResults[sysCheck]){
            sysState = SYS_STATE_ERROR;
            _ERROR("System Check Failed: %s", checkNames[sysCheck]);
            return;
        }
    }
    
    // 自检通过
    sysState = SYS_STATE_READY;
    _INFO("All System Checks Passed");
}

bool SysCheck_USART(void)
{
    iUSART_Init(iUSART1, USART_MODE_BASIC);
    // USART_RegisterCallback();

    if(!(USART1->CR1 & USART_CR1_UE)){
        return false;
    }

    return true;
}

bool SysCheck_TIM(void)
{
    iTIM_Init(iTIM2, TIM_MODE_BASIC);
    // TIM_RegisterCallback();

    if(!(RCC->APB1ENR & RCC_APB1ENR_TIM2EN)){
        _ERROR("TIM2 Clock Enable Failed");
        return false;
    }
    else if(!(TIM2->CR1 & TIM_CR1_CEN)){
        _ERROR("TIM2 Start Failed");
        return false;
    }

    return true;
}

bool SysCheck_EXTI(void)
{
    iEXTI_Init(GPIOA, GPIO_Pin_0, GPIO_Mode_IN_FLOATING, EXTI_Trigger_Rising);
    // EXTI_RegisterCallback();

    if(!(RCC->APB2ENR & RCC_APB2ENR_IOPAEN)){
        _ERROR("GPIOA Clock Not Enabled");
        return false;
    }
    else if(!(RCC->APB2ENR & RCC_APB2ENR_AFIOEN)){
        _ERROR("AFIO Clock Not Enabled");
        return false;
    }
    else if(!(EXTI->IMR & EXTI_Line0)){
        _ERROR("EXTI0 Line Not Enabled");
        return false;
    }
    else if(!(NVIC->ISER[EXTI0_IRQn >> 5] & (1 << (EXTI0_IRQn & 0x1F)))){
        _ERROR("EXTI0 NVIC Not Enabled");
        return false;
    }

    return true;
}

bool SysCheck_I2C(void)
{
	GPIO_I2C_Init();

    return true;
}

bool SysCheck_SYSTICK(void)
{
	sysTick_Init();

    if(!(SysTick->CTRL & SysTick_CTRL_ENABLE_Msk)){
        _ERROR("SysTick Not Enabled");
        return false;
    }
    else if(!(SysTick->CTRL & SysTick_CTRL_TICKINT_Msk)){
        _ERROR("SysTick Interrupt Not Enabled");
        return false;
    }

    return true;
}

bool SysCheck_DWT(void)
{
    DWT_Init();

    if(!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)){
        _ERROR("DWT Trace Not Enabled");
        return false;
    }
    else if(!(DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk)){
        _ERROR("DWT CYCCNT Not Enabled");
        return false;
    }

    return true;
}
