#ifndef _BLUETOOTH_H_
#define _BLUETOOTH_H_

#include "stm32f10x.h"

/* ========================= 环 形 缓 冲 区 类 定 义 ========================= */

typedef struct{
    uint16_t size;
    uint8_t* buffer;
    volatile uint16_t writeIndex;
    volatile uint16_t readIndex;
    volatile uint16_t count;
} RingBuffer_t;
typedef enum{
    BLOCKWRITE,
    OVERWRITE,
    BLOCKREAD,
    NOBLOCKREAD
} RingBuffer_Mode_t;

/* ========================= 接 口 函 数 声 明 ========================= */

void BlueTooth_Init(void);
void BlueTooth_Process(void);

#endif
