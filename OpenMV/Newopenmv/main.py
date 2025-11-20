THRESHOLD = (36, 58, -29, 1, -17, 3)

import sensor, image, time
import math
from pyb import LED
from machine import UART

def get_result(line):
    theta_hudu = line.theta() / 180.0 * 3.14159
    return (line.rho() - 60 * math.sin(theta_hudu)) / (math.cos(theta_hudu))

LED(1).on()
LED(2).on()
LED(3).on()
sensor.reset()
sensor.set_vflip(False)
sensor.set_hmirror(False)
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQQVGA)
sensor.skip_frames(time = 2000)
clock = time.clock()

uart = UART(1,115200)

while(True):
    clock.tick()
    img = sensor.snapshot().binary([THRESHOLD])
    line = img.get_regression([(100,100)], robust = True)
    img.draw_line(line.line(),color = [127,0,0])
    result = get_result(line)
    uart.write("$TRACK:%.3f#"%result)
    print(clock.fps(),"theta:",line.theta(),"rho",line.rho(),"result",result)
