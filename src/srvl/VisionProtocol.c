#include "VisionProtocol.h"
#include "UART.h"
#include <string.h>
#include "Chassis_d.h"

/* ========================= 私 有 变 量 声 明 ========================= */

static VisionState_t visionState = VISION_STATE_IDLE;

/* ========================= 私 有 函 数 声 明 ========================= */

static void VisionProtocol_RX_Callback(void);
static void VisionProtocol_Parse(const char* cmd);

/* ========================= 接 口 函 数 实 现 ========================= */

void VisionProtocol_Init(void)
{
    USART_RegisterCallback(iUSART2, VisionProtocol_RX_Callback);
}

void VisionProtocol_Process(void)
{
    switch(visionState){
        case VISION_STATE_IDLE:
            break;
        case VISION_STATE_GO:
            Chassis_GoStraight(500);
            break;
        case VISION_STATE_STOP:
            Chassis_Stop();
            visionState = VISION_STATE_IDLE;
            break;
        case VISION_STATE_LEFT:
            Chassis_Turn(90, 90);
            visionState = VISION_STATE_IDLE;
            break;
        case VISION_STATE_RIGHT:
            Chassis_Turn(-90, 90);
            visionState = VISION_STATE_IDLE;
            break;
    }
}

/* ========================= 私 有 函 数 实 现 ========================= */

static void VisionProtocol_RX_Callback(void)
{
    uint8_t res[32] = {0};
    uint8_t index = 0;
    uint8_t receiving = 0;
    while(USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == SET){
        char ch = USART_ReceiveData(USART2);
        if(ch == '$'){
            receiving = 1;
            index = 0;
            res[index++] = ch;
            continue;
        }
        else if(receiving){
            if(index < sizeof(res) - 1){
                res[index++] = ch;
                if(ch == '#'){
                    res[index] = '\0';
                    receiving = 0;
                    // 处理命令
                    VisionProtocol_Parse(res);
                    index = 0;
                }
            }
            else{
                index = 0;
                receiving = 0;
                _WARN("VisionProtocol_RX_Callback: Command too long");
            }
        }
        else{
            // 忽略多余数据
            index = 0;
            receiving = 0;
            _WARN("VisionProtocol_RX_Callback: Ignoring extra data: '%c'", ch);
        }
    }
}

static void VisionProtocol_Parse(const char* cmd)
{

    if(!cmd){
        _WARN("VisionProtocol_Parse: cmd is NULL");
        return;
    }
    if(strcmp(cmd, "$Trail:GO") == 0){
        visionState = VISION_STATE_GO;
    }
    else if(strcmp(cmd, "$Trail:STOP") == 0){
        visionState = VISION_STATE_STOP;
    }
    else if(strcmp(cmd, "$Trail:LEFT") == 0){
        visionState = VISION_STATE_LEFT;
    }
    else if(strcmp(cmd, "$Trail:RIGHT") == 0){
        visionState = VISION_STATE_RIGHT;
    }
    else{
        _WARN("VisionProtocol_Parse: Unknown command '%s'", cmd);
    }
}
