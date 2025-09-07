#include "BlueTooth.h"
#include "LOG_s.h"
#include "Delay_s.h"
#include "UART.h"
#include "LED_d.h"
#include "Chassis_d.h"
#include <stdio.h>
#include <string.h>

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

static void BlueTooth_Parse(const char* cmd)
{
    float p, i, d;
    if(!cmd){
        _WARN("BlueTooth_Parse: cmd is NULL");
        return;
    }
    if(strcmp(cmd, "$CMD:LED_ON#") == 0){
        LED_On();
        _INFO("BlueTooth_Parse: LED turned ON");
    }
    else if(strcmp(cmd, "$CMD:LED_OFF#") == 0){
        LED_Off();
        _INFO("BlueTooth_Parse: LED turned OFF");
    }
	else if(sscanf(cmd, "$PID:%f,%f,%f", &p, &i, &d) == 3){
        jy61pYawPID.p = p;
        jy61pYawPID.i = i;
        jy61pYawPID.d = d;
    }
    else{
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
    switch(mode)
    {
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
    switch(mode)
    {
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
