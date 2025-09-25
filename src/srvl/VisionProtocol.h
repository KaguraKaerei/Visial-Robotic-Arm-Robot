#ifndef _VISIONPROTOCOL_H_
#define _VISIONPROTOCOL_H_

typedef enum{
    VISION_STATE_IDLE = 0,
    VISION_STATE_GO,
    VISION_STATE_STOP,
    VISION_STATE_LEFT,
    VISION_STATE_RIGHT,
    VISION_STATE_MAX
} VisionState_t;

void VisionProtocol_Init(void);
void VisionProtocol_Process(void);

#endif
