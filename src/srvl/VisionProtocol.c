#include "VisionProtocol.h"
#include "UART.h"
#include <string.h>
#include "LOG_s.h"
#include "Chassis_d.h"
#include "a_Arm.h"

/* ========================= 私 有 变 量 声 明 ========================= */

static VisionState_t visionState = VISION_STATE_IDLE;
static char res[64] = { 0 };
static uint8_t index = 0;
static uint8_t receiving = 0;

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
            Chassis_GoStraight(100);
            _INFO("GO");
            break;
        case VISION_STATE_STOP:
            Chassis_Stop();
            _INFO("STOP");
            USART_Printf(USART2, "$ACK:OK#");
            visionState = VISION_STATE_IDLE;
            break;
        case VISION_STATE_LEFT:
            Chassis_Turn(90, 90);
            _INFO("LEFT");
            visionState = VISION_STATE_IDLE;
            break;
        case VISION_STATE_RIGHT:
            Chassis_Turn(-90, 90);
            _INFO("RIGHT");
            visionState = VISION_STATE_IDLE;
            break;
    }
}

/* ========================= 私 有 函 数 实 现 ========================= */

static void VisionProtocol_RX_Callback(void)
{
    char ch = (char)USART_ReceiveData(USART2);

    if(ch == '$'){
        receiving = 1;
        index = 0;
        res[index++] = ch;
    }
    else if(receiving){
        if(index < sizeof(res) - 1){
            res[index++] = ch;
            if(ch == '#'){
                res[index] = '\0';
                receiving = 0;
                // 处理命令
                printf("Received command: %s\n", res);
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

static void VisionProtocol_Parse(const char* cmd)
{
    int target_x_pixel, target_y_pixel;

    if(!cmd){
        _WARN("VisionProtocol_Parse: cmd is NULL");
        return;
    }
    if(strcmp(cmd, "$TRACK:GO#") == 0){
        visionState = VISION_STATE_GO;
    }
    else if(strcmp(cmd, "$TRACK:STOP#") == 0){
        visionState = VISION_STATE_STOP;
    }
    else if(strcmp(cmd, "$TRACK:LEFT#") == 0){
        visionState = VISION_STATE_LEFT;
    }
    else if(strcmp(cmd, "$TRACK:RIGHT#") == 0){
        visionState = VISION_STATE_RIGHT;
    }
    else if(strcmp(cmd, "$ARM:FIND_QR#") == 0){
        arm_state = ARM_FIND_QR;
    }
    else if(strcmp(cmd, "$ARM:FIND_TASK#") == 0){
        arm_state = ARM_FIND_TASK;
    }
    else if(sscanf(cmd, "$ARM:AIM_TARGET:%d,%d#", &target_x_pixel, &target_y_pixel) == 2){
        arm_param.r = (float)(target_x_pixel * 1920 / 640);
        arm_param.z = (float)(target_y_pixel * 1080 / 360);
        arm_state = ARM_STATE_AIM_TARGET;
    }
    else if(sscanf(cmd, "$ARM:AIM_LASER:%d,%d#", &target_x_pixel, &target_y_pixel) == 2){
        arm_param.r = (float)(target_x_pixel * 1920 / 640);
        arm_param.z = (float)(target_y_pixel * 1080 / 360);
        arm_state = ARM_STATE_AIM_LASER;
    }
    else if(sscanf(cmd, "$ARM:TARGET:%f#", &arm_param.depth) == 1){
        arm_state = ARM_STATE_TARGET;
    }
    else if(strcmp(cmd, "$ARM:GRASPING#") == 0){
        // TODO: 抓取目标物移动到指定位置：分不同任务情况
        
        arm_state = ARM_STATE_GRASPING;
    }
    else if(strcmp(cmd, "$ARM:PUT_IN#") == 0){
        arm_state = ARM_STATE_PUT_IN;
    }
    else if(sscanf(cmd, "$ARM:ANY:%f,%f,%f,%f#", &arm_param.x, &arm_param.y, &arm_param.z, &arm_param.angle) == 4){
        arm_state = ARM_STATE_ANY;
    }
    else{
        _WARN("VisionProtocol_Parse: Unknown command '%s'", cmd);
    }
}
