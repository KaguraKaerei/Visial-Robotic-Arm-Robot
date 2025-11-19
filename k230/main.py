# ========================  导入所需的库 ========================= #

# 标准微库
import time, os, sys
import image

# k230多媒体相关库
from media.sensor import *
from media.display import *
from media.media import *

# k230外设相关库
from machine import Pin
from machine import FPIOA

import Identify
import Serial

# =========================  创建FPIOA对象+分配引脚功能 ========================= #

fpioa = FPIOA()
fpioa.set_function(62, FPIOA.GPIO62)
fpioa.set_function(20, FPIOA.GPIO20)
fpioa.set_function(63, FPIOA.GPIO63)
fpioa.set_function(53, FPIOA.GPIO53)

# 初始化引脚
ledR = Pin(62, Pin.OUT, pull=Pin.PULL_NONE, drive=7)    # 板载红色LED灯
ledG = Pin(20, Pin.OUT, pull=Pin.PULL_NONE, drive=7)    # 板载绿色LED灯
ledB = Pin(63, Pin.OUT, pull=Pin.PULL_NONE, drive=7)    # 板载蓝色LED灯
button = Pin(53, Pin.IN, Pin.PULL_DOWN)                 # 板载按键

# 相关变量
debounceDelay = 200                                     # 按键消抖延时(ms)
lastPressTime = 0                                       # 上次按键按下的时间(ms)
buttonLastState = 0                                     # 上次按键状态

# 默认熄灭LED灯
ledR.high()
ledG.high()
ledB.high()

# 拍照指示灯
photoLED = ledG

# =========================  显示配置 ========================= #

display_mode = "hdmi"
if display_mode == "lcd":
    DISPLAY_WIDTH = ALIGN_UP(800, 16)
    DISPLAY_HEIGHT = 480
else:
    DISPLAY_WIDTH = ALIGN_UP(1920, 16)
    DISPLAY_HEIGHT = 1080

OUT_RGB888P_WIDTH = ALIGN_UP(640, 16)
OUT_RGB888P_HEIGHT = 360

def SaveJpg(img, filename, quality=95):
    """
    拍下当前画面并保存为JPG文件
    :param _type_ img: 传入的图像对象
    :param _type_ filename: 保存的文件名
    :param int quality: 图像质量，默认为95
    """
    compressedData = img.compress(quality=quality)

    with open(filename, "wb") as f:
        f.write(compressedData)

    print(f"已保存: {filename}")

# =========================  自动创建图片保存文件夹 & 计算已有图片数量 ========================= #

imgFolder = "/data/images"

try:
    os.stat(imgFolder)
except OSError:
    os.mkdir(imgFolder)

# 统计当前目录下以"img_XXX.jpg"开头的图片数量，自动从最大编号继续
imgCount = 0
existingImgs = [fname for fname in os.listdir(imgFolder)
                if fname.startswith("img_") and fname.endswith(".jpg")]
if existingImgs:
    # 提取编号并找到最大值
    numbers = []
    for fname in existingImgs:
        # 提取中间XX部分转为数字
        try:
            numPart = fname[4:-4]
            numbers.append(int(numPart))
        except:
            pass
    if numbers:
        imgCount = max(numbers) + 1

# =========================  主程序 ========================= #

try:
    sensor = Sensor(id=2)
    sensor.reset()

    # 通道0用于除模型检测外的任务，格式为RGB565
    sensor.set_framesize(width=OUT_RGB888P_WIDTH, height=OUT_RGB888P_HEIGHT, chn=CAM_CHN_ID_0)
    sensor.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_0)

    # 通道1用于模型检测，格式为RGB888P
    sensor.set_framesize(width=OUT_RGB888P_WIDTH, height=OUT_RGB888P_HEIGHT, chn=CAM_CHN_ID_1)
    sensor.set_pixformat(PIXEL_FORMAT_RGB_888_PLANAR, chn=CAM_CHN_ID_1)

    # 通道2用于显示，格式为YUV420
    sensor.set_framesize(width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT, chn=CAM_CHN_ID_2)
    sensor.set_pixformat(PIXEL_FORMAT_YUV_SEMIPLANAR_420, chn=CAM_CHN_ID_2)

    # 绑定通道2到显示（即无需捕获，直接显示传感器数据）
    sensor_bind_info = sensor.bind_info(x=0, y=0, chn=CAM_CHN_ID_2)
    Display.bind_layer(**sensor_bind_info, layer=Display.LAYER_VIDEO1)

    if display_mode == "lcd":
        # 设置为ST7701显示，默认800x480
        Display.init(Display.ST7701, to_ide=True)
    else:
        # 设置为LT9611显示，默认1920x1080
        Display.init(Display.LT9611, to_ide=True)

    # 初始化媒体管理器
    MediaManager.init()

    # 启动传感器
    sensor.run()

    # 初始化模型检测
    Identify.ModelDetect_Init()

    # 构造fps
    fps = time.clock()

    # 初始化串口通信
    Serial.Serial_Init()

    # 启动时开始让机械臂准备检测二维码
    Serial.SendCmd(f"$ARM:FIND_QR#")

    while True:
        fps.tick()
        os.exitpoint()

        # 捕获通道0的图像用于其他任务，通道1的图像用于模型检测
        img = sensor.snapshot(chn=CAM_CHN_ID_0)
        rgb888p_img = sensor.snapshot(chn=CAM_CHN_ID_1)

        Identify.IdentifyProcess(img, rgb888p_img)

        # 检测按键状态变化
        currentTime = time.ticks_ms()
        buttonState = button.value()

        if buttonState == 1 and buttonLastState == 0:    # 上升沿
            if currentTime - lastPressTime > debounceDelay:
                photoLED.low()                          # 点亮拍照指示灯
                time.sleep_ms(20)
                photoLED.high()                         # 熄灭拍照指示灯

                # 拍照并保存
                imgCount += 1
                filename = f"{imgFolder}/img_{imgCount:03d}.jpg"
                print("拍照: ", filename)

                SaveJpg(img, filename)

                lastPressTime = currentTime

        buttonLastState = buttonState

        # 打印帧率
        # print("FPS: ", fps.fps())

except KeyboardInterrupt as e:
    print("用户停止: ", e)
except BaseException as e:
    print(f"异常: {e}")
finally:
    # 停止传感器运行
    if isinstance(sensor, Sensor):
        sensor.stop()
    # 反初始化显示模块
    Display.deinit()
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
    # 释放媒体缓冲区
    MediaManager.deinit()
