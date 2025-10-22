import time
import sensor
from machine import LED

import Identify

#####  设备初始化  #####
def OpenMV_Init():
    # LED
    led = LED("LED_BLUE")
    # 时钟
    clock = time.clock()
    # 摄像头
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    sensor.set_auto_gain(False)
    sensor.set_auto_whitebal(False)
    sensor.set_auto_exposure(False, exposure_us=10000)
    sensor.skip_frames(time=1000)
    # 初始化赛道识别器和串口通信器
    identifier = Identify.TaskIdentifier()

    return led, clock, identifier

#####  主循环  #####
def main():
    led, clock, identifier= OpenMV_Init()

    img = sensor.snapshot()
    identifier.trackIdentifier.SetGrayThreshold(img)

    lastToggle = time.ticks_ms()
    
    while True:
        clock.tick()
        img = sensor.snapshot()

        # 任务识别状态机进程
        identifier.IdentifyProcess(img)

        # 500ms翻转LED
        if time.ticks_diff(time.ticks_ms(), lastToggle) > 500:
            led.toggle()
            lastToggle = time.ticks_ms()

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        import sys
        sys.print_exception(e)
