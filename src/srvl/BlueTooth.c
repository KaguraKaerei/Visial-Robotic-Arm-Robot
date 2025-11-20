#include "BlueTooth.h"
#include "s_LOG.h"
#include "Delay_s.h"
#include "UART.h"
#include "LED_d.h"
#include "Chassis_d.h"
#include "Servo_d.h"
#include <stdio.h>
#include <string.h>
#include "a_Arm.h"
#include "a_Chassis_closeloop.h"

/* ========================= 私 有 变 量 声 明 ========================= */

static RingBuffer_t btRingBuffer;
static uint8_t btBuffer[256];
static char cmdBuffer[64];
static uint8_t cmdIndex = 0;
static uint8_t cmdReceiving = 0;

/* ========================= 私 有 函 数 声 明 ========================= */

static void BlueTooth_Callback(void);
static void BlueTooth_Parse(const char* cmd);

static void RingBuffer_Init(RingBuffer_t* const rb, uint8_t* buffer, uint16_t size);
static uint8_t RingBuffer_GetStatus(const RingBuffer_t* const rb);
static uint8_t RingBuffer_Write(RingBuffer_t* const rb, const uint8_t* data, uint16_t length, RingBuffer_Mode_t mode);
static int RingBuffer_Read(RingBuffer_t* const rb, uint8_t* data, uint16_t length, RingBuffer_Mode_t mode);

/* ========================= 接 口 函 数 实 现 ========================= */

void BlueTooth_Init(void)
{
    RingBuffer_Init(&btRingBuffer, btBuffer, sizeof(btBuffer));
    USART_RegisterCallback(iUSART1, BlueTooth_Callback);
}

void BlueTooth_Process(void)
{
    uint8_t data;
    while(RingBuffer_Read(&btRingBuffer, &data, 1, NOBLOCKREAD) == 1){
        if(data == '$'){
            cmdReceiving = 1;
            cmdIndex = 0;
            cmdBuffer[cmdIndex++] = data;
        }
        else if(cmdReceiving){
            if(cmdIndex < sizeof(cmdBuffer) - 1){
                cmdBuffer[cmdIndex++] = data;
                if(data == '#'){
                    cmdBuffer[cmdIndex] = '\0';
                    cmdReceiving = 0;
                    // 处理命令
                    BlueTooth_Parse(cmdBuffer);
                    cmdIndex = 0;
                }
            }
            else{
                // 命令过长，重置
                cmdIndex = 0;
                cmdReceiving = 0;
                _WARN("BlueTooth_Process: Command too long");
            }
        }
        else{
            // 忽略多余数据
            cmdIndex = 0;
            cmdReceiving = 0;
            _WARN("BlueTooth_Process: Ignored data '%c'", data);
        }
    }
}

/* ========================= 私 有 函 数 实 现 ========================= */

static void BlueTooth_Callback(void)
{
    uint8_t byte = USART_ReceiveData(USART1);
    RingBuffer_Write(&btRingBuffer, &byte, 1, OVERWRITE);
}

#include "s_Kinematics.h"
static void BlueTooth_Parse(const char* cmd)
{
    float p, i, d;
    int speed, angle, angularVel;
    int dataInfoFlag = 0;
    uint16_t servoCCR;
    int targetangle;

    if(!cmd){
        _WARN("BlueTooth_Parse: cmd is NULL");
        return;
    }
    if(sscanf(cmd, "$MODE:GO:%d#", &speed) == 1){
        Chassis_GoStraight(speed);
    }
    else if(sscanf(cmd, "$MODE:TURN:%d,%d#", &angle, &angularVel) == 2){
        Chassis_Turn(angle, angularVel);
    }
    else if(strcmp(cmd, "$MODE:STOP#") == 0){
        Chassis_Stop();
    }
    else if(sscanf(cmd, "$MODE:DATAINFO:%d#", &dataInfoFlag) == 1 ||
        sscanf(cmd, "$MODE:DATAINFO:%d:%d#", &dataInfoFlag, &speed) == 2 ||
        sscanf(cmd, "$MODE:DATAINFO:%d:%d,%d#", &dataInfoFlag, &angle, &angularVel) == 3){
        switch(dataInfoFlag){
            case 0: break;
            case 1:
                Chassis_GoStraight(speed);
                break;
            case 2:
                Chassis_Turn(angle, angularVel);
                break;
            case 3:
                Chassis_Turn_JY61P(angle);
                break;
        }
        Chassis_GetData(&chassisParam);
        _INFO("Chassis: [LF, RF, LR, RR][10ms, speed]: "
            "%d, %d, %d, %d, "
            "%d, %d, %d, %d  ",
            chassisParam.encorder10ms[0], chassisParam.encorderSpeed[0],
            chassisParam.encorder10ms[1], chassisParam.encorderSpeed[1],
            chassisParam.encorder10ms[2], chassisParam.encorderSpeed[2],
            chassisParam.encorder10ms[3], chassisParam.encorderSpeed[3]);
    }
    else if(sscanf(cmd, "$PID:%f,%f,%f#", &p, &i, &d) == 3){
        anglepid.p    = p;
        anglepid.i    = i;
        anglepid.d    = d;
    } else if (sscanf(cmd, "$TARGETANGLE:%d#", &targetangle) == 1) {
        temptemptemp = targetangle;
    } else if (strcmp(cmd, "$LAUNCH#") == 0) {
        //basicspeed = 3000;
    else if(sscanf(cmd, "$PID:Kp:%f#", &p) == 1){
        aim_pid_yaw.p = p;
    }
    else if(sscanf(cmd, "$PID:Ki:%f#", &i) == 1){
        aim_pid_yaw.i = i;
    }
    else if(sscanf(cmd, "$PID:Kd:%f#", &d) == 1){
        aim_pid_yaw.d = d;
    }

    else if (sscanf(cmd, "$SERVO:CCR:%hu#", &servoCCR) == 1) {
        Servo_SetCCR(SERVO_CHASSIS, servoCCR);
    } else if (sscanf(cmd, "$ARM:ANY:%f,%f,%f,%f#", &arm_param.x, &arm_param.y, &arm_param.z, &arm_param.angle) == 4) {
        arm_state = ARM_STATE_ANY;
    } else if (strcmp(cmd, "$ARM:END#") == 0) {
        arm_state = ARM_STATE_END;
    } else {
        _WARN("BlueTooth_Parse: Unknown command '%s'", cmd);
    }
}

static void RingBuffer_Init(RingBuffer_t* const rb, uint8_t* buffer, uint16_t size)
{
    if(!rb || !buffer || size == 0){
        _WARN("RingBuffer_Init: Invalid parameters");
        return;
    }

    rb->size = size;
    rb->buffer = buffer;
    if(!rb->buffer){
        _WARN("RingBuffer_Init: Memory allocation failed");
        rb->size = 0;
        return;
    }
    rb->writeIndex = 0;
    rb->readIndex = 0;
    rb->count = 0;
}

static uint8_t RingBuffer_GetStatus(const RingBuffer_t* const rb)
{
    if(!rb){
        _WARN("RingBuffer_GetStatus: rb is NULL");
        return 0;
    }
    if(rb->count == 0) return 0;                // 空
    else if(rb->count == rb->size) return 2;    // 满
    else return 1;                              // 有数据
}

static uint8_t RingBuffer_Write(RingBuffer_t* const rb, const uint8_t* data, uint16_t length, RingBuffer_Mode_t mode)
{
    switch(mode){
        case BLOCKWRITE:
        {
            for(uint16_t i = 0; i < length; ++i){
                uint8_t timeout = 100;
                while(RingBuffer_GetStatus(rb) == 2 && timeout > 0){
                    Delay_ms(1);
                    timeout--;
                }
                if(timeout == 0){
                    _WARN("RingBuffer_Write: timeout");
                    return 1;
                }
                rb->buffer[rb->writeIndex] = data[i];
                rb->writeIndex = (rb->writeIndex + 1) % rb->size;
                rb->count++;
            }
            return 0;
        }
        case OVERWRITE:
        {
            for(uint16_t i = 0; i < length; ++i){
                if(RingBuffer_GetStatus(rb) == 2){
                    rb->readIndex = (rb->readIndex + 1) % rb->size;
                    rb->count--;
                }
                rb->buffer[rb->writeIndex] = data[i];
                rb->writeIndex = (rb->writeIndex + 1) % rb->size;
                rb->count++;
            }
            return 0;
        }
        default:
            _WARN("RingBuffer_Write: Unsupported mode");
            return 1;
    }
}

static int RingBuffer_Read(RingBuffer_t* const rb, uint8_t* data, uint16_t length, RingBuffer_Mode_t mode)
{
    switch(mode){
        case BLOCKREAD:
        {
            for(uint16_t i = 0; i < length; ++i){
                uint8_t timeout = 100;
                while(RingBuffer_GetStatus(rb) == 0 && timeout > 0){
                    Delay_ms(1);
                    timeout--;
                }
                if(timeout == 0){
                    _WARN("RingBuffer_Read: timeout");
                    return -1;
                }
                data[i] = rb->buffer[rb->readIndex];
                rb->readIndex = (rb->readIndex + 1) % rb->size;
                rb->count--;
            }
            return 0;
        }
        case NOBLOCKREAD:
        {
            uint16_t bytesRead = 0;
            for(uint16_t i = 0; i < length; ++i){
                if(RingBuffer_GetStatus(rb) == 0) break;
                data[i] = rb->buffer[rb->readIndex];
                rb->readIndex = (rb->readIndex + 1) % rb->size;
                rb->count--;
                bytesRead++;
            }
            return bytesRead;
        }
        default:
            _WARN("RingBuffer_Read: Unsupported mode");
            return -1;
    }
}
