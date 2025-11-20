#include "SysManager_a.h"

/* ========================= 私 有 变 量 声 明 ========================= */

static SysState_t sysState = SYS_STATE_INIT;
static bool sysReady = false;

/* ========================= 私 有 函 数 声 明 ========================= */

static void SysManager_InitHardware(void);
static void SysManager_InitModules(void);

/* ========================= 接 口 函 数 实 现 ========================= */

/**
 * @brief 系统管理器初始化函数
 */
void SysManager_Init(void)
{
    sysState = SYS_STATE_INIT;
    sysReady = false;

    // 基础硬件初始化
    SysManager_InitHardware();
    // 功能模块初始化
    SysManager_InitModules();

    sysState = SYS_STATE_RUNNING;
    sysReady = true;
    LED_Off();
    Delay_s(1);
    _INFO("SysManager_Init: System initialized successfully");
}

/**
 * @brief 系统管理器进程函数
 */
void SysManager_Process(void)
{
    if(!sysReady) return;

    switch(sysState){
        case SYS_STATE_RUNNING:
            // 系统正常运行
			
            
			break;

        case SYS_STATE_ERROR:
            _ERROR("System in error state!");
            LED_On();
            break;

        default:
            break;
    }
}

/**
 * @brief 检查系统是否就绪
 */
bool System_Is_Ready(void)
{
    return sysReady && (sysState != SYS_STATE_ERROR);
}

/**
 * @brief 基础硬件初始化
 */
static void SysManager_InitHardware(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    LED_Init();
    LED_On();

    sysTick_Init();
    DWT_Init();

    // 串口通信
    iUSART_Init(iUSART1, USART_MODE_BASIC);     // 蓝牙
    iUSART_Init(iUSART2, USART_MODE_BASIC);     // 视觉
    iUSART_Init(iUSART3, USART_MODE_BASIC);     // 底盘

    // 定时器初始化
    iTIM_Init(iTIM2, TIM_MODE_PWM);             // PA0/PA1 —— 舵机1/舵机2
    iTIM_Init(iTIM3, TIM_MODE_PWM);             // PA7/PB0/PB1 —— 舵机3/舵机4/舵机5
}

/**
 * @brief 功能模块初始化
 */
static void SysManager_InitModules(void)
{
    // 底盘
    Chassis_Init();
    //Chassis_SelfCtrl_Init();
    Chassis_closeloop_Init();
    // 蓝牙
    BlueTooth_Init();
    // 陀螺仪
    JY61p_Init();
    // 视觉
    VisionProtocol_Init();
    // 舵机
    Servo_Init();
    // 机械臂运动学
    Arm_Kinematics_Init();
}
