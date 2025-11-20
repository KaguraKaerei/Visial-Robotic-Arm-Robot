# ======================== 导入所需的库 ========================= #

# 标准微库
import time
import os
import sys
import gc
import ujson

# k230多媒体相关库
from media.sensor import *
from media.display import *
from media.media import *
import image

# k230外设相关库
from machine import Pin
from machine import FPIOA

# k230 AI相关库
import aicube
import nncase_runtime as nn
import ulab.numpy as np
from libs.PipeLine import ScopedTiming
from libs.Utils import *

# 自定义库
import Serial

# ======================== 初 始 化 引 脚 ========================== #

fpioa = FPIOA()
fpioa.set_function(3, FPIOA.GPIO3)
shooter = Pin(3, Pin.OUT, pull=Pin.PULL_NONE, drive=7)

# ======================== 识别任务状态机 ========================= #

# 状态枚举
class TaskState:
    IDLE = 0
    TRACK = 1
    QR = 2
    EXPLOSIVE = 3
    TARGET = 4
    HOSTAGE = 5

state = TaskState.IDLE

_transition = {
    TaskState.IDLE: {
        "startTrack": TaskState.TRACK,
    },
    TaskState.TRACK: {
        "stopTrack": TaskState.IDLE,
        "findQR": TaskState.QR,
        "findExplosive": TaskState.EXPLOSIVE,
        "findTarget": TaskState.TARGET,
        "findHostage": TaskState.HOSTAGE
    },
    TaskState.QR: {
        "endQR": TaskState.TRACK
    },
    TaskState.EXPLOSIVE: {
        "endExplosive": TaskState.TRACK
    },
    TaskState.TARGET: {
        "endTarget": TaskState.TRACK
    },
    TaskState.HOSTAGE: {
        "endHostage": TaskState.TRACK
    }
}

# 赛道识别状态
last_track_state = "STOP"
# 个任务识别结果缓存
_qrPayloads = (0, 0, 0)
_explosive = (0, 0)
_target = (0, 0)
_hostage = (0, 0)
# 任务完成数量标志
_taskDoneFlags = {
    "QR": False,
    "Explosive": False,
    "Target": False,
    "Hostage": False
}

# 识别任务状态机进程函数
def IdentifyProcess(img: image.Image, rgb888p_img: image.Image):
    global state
    global last_track_state
    global _qrPayloads, _explosive, _target, _hostage

    # TODO: 目前默认进入寻迹状态，需要判断起始条件
    if state == TaskState.TRACK:
        track_state = Serial.ReciveTrackState()
        
        if track_state is not None and track_state != last_track_state:
            Serial.SendTrackState(track_state)
            print(f"[TRACK] State changed to: {track_state}")
        last_track_state = track_state
    
        if _taskDoneFlags["QR"] is not True:
            found, payloads = FindQR(img)
            if found:
                print(f"[QR] Detected QR codes: {payloads}")
                _qrPayloads = payloads
                Trigger("findQR")

        # 识别到二维码后才执行其他任务
        if _taskDoneFlags["QR"] is True:
            if _taskDoneFlags["Explosive"] is not True:
                found, position = FindExplosive(img)
                if found:
                    _explosive = position
                    Trigger("findExplosive")

            if _taskDoneFlags["Target"] is not True:
                found, position = FindTarget(img)
                if found:
                    _target = position
                    Trigger("findTarget")

            if _taskDoneFlags["Hostage"] is not True:
                found, position = FindHostage(rgb888p_img)
                if found:
                    _hostage = position
                    Trigger("findHostage")
        
    # 识别到二维码后的处理
    elif state == TaskState.QR:
        if len(_qrPayloads) == 3:
            # 识别到二维码后开始寻找任务
            Serial.SendCmd(f"$ARM:FIND_TASK#")
            _taskDoneFlags["QR"] = True
        else:
            _taskDoneFlags["QR"] = False
        Trigger("endQR")
    
    # TODO:识别到爆炸物后的处理：停车——发送位置——等待瞄准完毕——获取深度——抓取——放下——标记完成
    elif state == TaskState.EXPLOSIVE:
        if len(_explosive) == 2:
            data_written = Serial.uartMain.write("$TRACK:STOP#")
            if data_written == len("$TRACK:STOP#"):
                read = Serial.uartMain.read()
                last_time = time.ticks_ms()
                while read is None:
                    read = Serial.uartMain.read()
                    if time.ticks_ms() - last_time > 1000:
                        break
                last_time = time.ticks_ms()
                if read == "$ACK:OK#".encode("utf-8"):
                    while read is None:
                        read = Serial.uartMain.read()
                        found, position = FindExplosive(img)
                        if found:
                            _explosive = position
                            data = f"$ARM:AIM_TARGET:{_explosive[0]},{_explosive[1]}#"
                            Serial.uartMain.write(data)
                        last_time = time.ticks_ms()
                        if time.ticks_ms() - last_time > 10000:
                            break
                    if read == "$ARM:TARGET_OK#".encode("utf-8"):
                        pass
                    else:
                        _taskDoneFlags["Explosive"] = False
                else:
                    _taskDoneFlags["Explosive"] = False
            else:
                _taskDoneFlags["Explosive"] = False
        else:
            _taskDoneFlags["Explosive"] = False
        Trigger("endExplosive")

    # 识别到标靶后的处理：停车——发送位置——等待瞄准完毕——射击——标记完成
    elif state == TaskState.TARGET:
        if len(_target) == 2:
            data_written = Serial.uartMain.write(f"$TRACK:STOP#")
            read = None
            if data_written == len("$TRACK:STOP#"):
                read = Serial.uartMain.read()
                last_time = time.ticks_ms()
                while read is None:
                    read = Serial.uartMain.read()
                    if time.ticks_ms() - last_time > 1000:
                        break
                print("Received:", read)
                if read == "$ACK:OK#".encode("utf-8"):
                    read = None
                    last_time = time.ticks_ms()
                    while read is None:
                        print("Rcvd:", read)
                        read = Serial.uartMain.read()
                        found, position = FindTarget(img)
                        if found:
                            _target = position
                            data = f"$ARM:AIM_LASER:{_target[0]},{_target[1]}#"
                            Serial.uartMain.write(data)
                        if time.ticks_ms() - last_time > 10000:
                            break
                    if read == "$ARM:AIM_AT_OK#".encode("utf-8"):
                        # 射击
                        shooter.low()
                        time.sleep_ms(500)
                        shooter.high()
                        _taskDoneFlags["Target"] = True
                    else:
                        _taskDoneFlags["Target"] = False
                else:
                    _taskDoneFlags["Target"] = False
            else:
                _taskDoneFlags["Target"] = False
        else:
            _taskDoneFlags["Target"] = False
        Trigger("endTarget")

    # TODO:识别到人质后的处理：停车——发送位置——等待瞄准完毕——获取深度——抓取——放下——标记完成
    elif state == TaskState.HOSTAGE:
        if len(_hostage) == 2:
            data_written = Serial.uartMain.write(f"$TRACK:STOP#")
            if data_written == len("$TRACK:STOP#"):
                read = Serial.uartMain.read()
                last_time = time.ticks_ms()
                while read is None:
                    read = Serial.uartMain.read()
                    if time.ticks_ms() - last_time > 1000:
                        break
                if read == "$ACK:OK#".encode("utf-8"):
                    found, position = FindHostage(rgb888p_img)
                    if found:
                        _hostage = position
                        data = f"$HOSTAGE:{_hostage[0]},{_hostage[1]}#"
                        data_written = Serial.uartMain.write(data)
                        if data_written == len(data):
                            _taskDoneFlags["Hostage"] = True
                        else:
                            _taskDoneFlags["Hostage"] = False
                    else:
                        _taskDoneFlags["Hostage"] = False
                else:
                    _taskDoneFlags["Hostage"] = False
            else:
                _taskDoneFlags["Hostage"] = False
        else:
            _taskDoneFlags["Hostage"] = False

        Trigger("endHostage")

    # 空闲状态
    elif state == TaskState.IDLE:
        Trigger("startTrack")

# 触发状态转移
def Trigger(event: str):
    """
    触发状态转移
    依据当前状态和事件查找下一个状态，并执行状态转移
    :param str event: 事件名称
    :return bool: 是否成功触发状态转移
    """
    global state

    # 查找事件对应的下一个状态
    nextState = _transition.get(state, {}).get(event)
    if nextState is None:
        print(f"[STATE-ERROR] Invalid transition: {state} --{event}--> ?")
        return False
    # 执行状态转移
    state = nextState
    print(f"[STATE] Transition: {state} --{event}--> {nextState}")

    return True

# 识别二维码
def FindQR(img: image.Image):
    """
    识别图像中的二维码
    :param _type_ img: _description_
    :return bool, list: 是否识别到二维码, 识别到的二维码内容列表
    """
    if img.format() != image.GRAYSCALE:
        img = img.to_grayscale(copy=True)
        
    codes = img.find_qrcodes()
    if not codes: return False, []
    payloads = []
    for code in codes:
        try:
            txt = code.payload()
            values = tuple([int(c) for c in txt])

            if len(values) == 3:
                return True, values
        except:
            txt = ""

    return False, None

# 识别爆炸物
def FindExplosive(img: image.Image):
    """
    识别图像中的爆炸物
    :param _type_ img: _description_
    :return bool, tuple: 是否识别到爆炸物, 识别到的爆炸物位置
    """
    global _qrPayloads
    # LAB颜色阈值
    explosiveColor = (0, 0, 0, 0, 0, 0)
    if _qrPayloads[0] == 1:
        explosiveColor = (5, 80, 15, 70, 5, 55)             # 红色
    elif _qrPayloads[0] == 2:
        explosiveColor = (5, 80, -70, -15, 0, 70)           # 绿色
    elif _qrPayloads[0] == 3:
        explosiveColor = (5, 80, 5, 55, -95, -10)           # 蓝色
    else:
        return False, None

    # 找色块
    blobs = img.find_blobs([explosiveColor], pixels_threshold=12000, area_threshold=12500, merge=True)
    if not blobs: return False, None
    
    # 判断是否为实心圆
    valid_blobs = []
    for blob in blobs:
        # 圆度检查
        if blob.roundness() > 0.6:
            # 长宽比检查
            aspect_ratio = blob.w() / blob.h() if blob.h() > 0 else 0
            if 0.6 < aspect_ratio < 1.4:
                # 实心度检查
                density = blob.density()
                if 0.6 < density < 0.9:
                    valid_blobs.append(blob)
    
    if not valid_blobs:
        return False, None
    
    blob = max(valid_blobs, key=lambda b: b.pixels())

    x, y = blob.cx(), blob.cy()

    return True, (x, y)

# 识别标靶
def FindTarget(img: image.Image):
    """
    识别图像中的标靶
    :param _type_ img: _description_
    :return bool, tuple: 是否识别到标靶, 识别到的标靶位置
    """
    global _qrPayloads
    # LAB颜色阈值
    targetColor = (0, 0, 0, 0, 0, 0)
    if _qrPayloads[1] == 1:
        targetColor = (5, 80, 15, 70, 5, 55)            # 红色
    elif _qrPayloads[1] == 2:
        targetColor = (5, 80, -70, -15, 0, 70)          # 绿色
    elif _qrPayloads[1] == 3:
        targetColor = (5, 80, 5, 55, -95, -10)          # 蓝色
    else:
        return False, None

    blobs = img.find_blobs([targetColor], pixels_threshold=8000, area_threshold=12500, merge=True)
    if not blobs: return False, None

    # 判断是否为圆环
    valid_blobs = []
    for blob in blobs:
        # 圆度检查
        if blob.roundness() > 0.6:
            # 长宽比检查与密度检查
            aspect_ratio = blob.w() / blob.h() if blob.h() > 0 else 0
            density = blob.density()
            if 0.6 < aspect_ratio < 1.4 and 0.4 < density < 0.7:
                # 识别圆环内部空洞
                inner_circles = img.find_circles(roi=blob.rect(), threshold=1000, x_margin=10, y_margin=10, r_margin=10, r_min=blob.w()//10, r_max=blob.w()//3)
                if len(inner_circles) > 1:
                    valid_blobs.append(blob)
    
    if not valid_blobs:
        return False, None
    
    blob = max(valid_blobs, key=lambda b: b.pixels())

    x, y = blob.cx(), blob.cy()

    return True, (x, y)

# 识别人质(训练模型: 圆柱、圆梯、腰鼓)
def FindHostage(rgb888p_img: image.Image):
    """
    识别图像中的人质
    :param rgb888p_img: 输入的RGB888P格式图像
    :return bool, tuple: 是否识别到人质, 识别到的人质位置
    """
    global _qrPayloads
    targetHostages = ""
    if _qrPayloads[2] == 1:
        targetHostages = "圆柱"
    elif _qrPayloads[2] == 2:
        targetHostages = "圆梯"
    elif _qrPayloads[2] == 3:
        targetHostages = "腰鼓"
    else:
        return False, None
    
    results = ModelDetect_Run(rgb888p_img)
    
    if not results:
        return False, None
    
    valid_det = []
    for i in results: 
        if i["class"] == targetHostages and i["confidence"] > 0.7:
            valid_det.append(i)

    if not valid_det:
        return False, None

    best_det = max(valid_det, key=lambda x: x["confidence"], default=None)
    if best_det is None:
        return False, None
    
    cx = best_det["x"] + best_det["w"] // 2
    cy = best_det["y"] + best_det["h"] // 2

    return True, (cx, cy)

# ======================== 人质检测模型 ========================= #

# 显示相关参数
display_mode = "hdmi"
if display_mode == "lcd":
    DISPLAY_WIDTH = ALIGN_UP(800, 16)
    DISPLAY_HEIGHT = 480
else:
    DISPLAY_WIDTH = ALIGN_UP(1920, 16)
    DISPLAY_HEIGHT = 1080

OUT_RGB888P_WIDTH = ALIGN_UP(640, 16)
OUT_RGB888P_HEIGHT = 360

# 路径和配置
root_path = "/data/model/ThreeHostages/"
config_path = root_path + "deploy_config.json"
deploy_conf = {}
debug_mode = 0

# 人质检查模型初始化函数
def ModelDetect_Init():
    """
    初始化人质检测系统
    包括加载模型、配置传感器和显示等
    """
    global _detection_initialized, _kpu, _ai2d_builder, _osd_img
    global _ai2d_output_tensor, _deploy_conf, _kmodel_frame_size, _frame_size
    global _labels, _num_classes, _color_four, _confidence_threshold
    global _nms_threshold, _anchors, _nms_option, _strides
    
    if _detection_initialized:
        print("[WARNING] Detection already initialized")
        return
    
    # 使用json读取内容初始化部署变量
    _deploy_conf = _read_deploy_config(config_path)
    kmodel_name = _deploy_conf["kmodel_path"]
    _labels = _deploy_conf["categories"]
    _confidence_threshold = _deploy_conf["confidence_threshold"]
    _nms_threshold = _deploy_conf["nms_threshold"]
    img_size = _deploy_conf["img_size"]
    _num_classes = _deploy_conf["num_classes"]
    _color_four = get_colors(_num_classes)
    _nms_option = _deploy_conf["nms_option"]
    model_type = _deploy_conf["model_type"]
    if model_type == "AnchorBaseDet":
        _anchors = _deploy_conf["anchors"][0] + _deploy_conf["anchors"][1] + _deploy_conf["anchors"][2]
    _kmodel_frame_size = img_size
    _frame_size = [OUT_RGB888P_WIDTH, OUT_RGB888P_HEIGHT]
    _strides = [8, 16, 32]

    # 计算padding值
    top, bottom, left, right, ratio = _two_side_pad_param(_frame_size, _kmodel_frame_size)

    # 初始化kpu
    _kpu = nn.kpu()
    _kpu.load_kmodel(root_path + kmodel_name)
    
    # 初始化ai2d
    ai2d = nn.ai2d()
    ai2d.set_dtype(nn.ai2d_format.NCHW_FMT, nn.ai2d_format.NCHW_FMT, np.uint8, np.uint8)
    ai2d.set_pad_param(True, [0, 0, 0, 0, top, bottom, left, right], 0, [114, 114, 114])
    ai2d.set_resize_param(True, nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
    _ai2d_builder = ai2d.build(
        [1, 3, OUT_RGB888P_HEIGHT, OUT_RGB888P_WIDTH], 
        [1, 3, _kmodel_frame_size[1], _kmodel_frame_size[0]]
    )
    
    # 创建OSD图像
    _osd_img = image.Image(DISPLAY_WIDTH, DISPLAY_HEIGHT, image.ARGB8888)
    
    # 准备ai2d输出tensor
    data = np.ones((1, 3, _kmodel_frame_size[1], _kmodel_frame_size[0]), dtype=np.uint8)
    _ai2d_output_tensor = nn.from_numpy(data)
    
    _detection_initialized = True

# 人质检测运行函数
def ModelDetect_Run(rgb888p_img):
    """
    运行一次人质检测
    返回检测到的目标列表
    :param image.Image rgb888p_img: 输入的RGB888P格式图像
    :return list: 检测结果列表, 每个元素为 (标签, 置信度, x, y, w, h)
    """
    global _detection_initialized, _kpu, _ai2d_builder, _osd_img
    global _ai2d_output_tensor, _kmodel_frame_size, _frame_size
    global _labels, _num_classes, _color_four, _confidence_threshold
    global _nms_threshold, _anchors, _nms_option, _strides
    
    if not _detection_initialized:
        print("[ERROR] Detection not initialized. Call ModelDetect_Init() first")
        return []
    
    det_results = []
    
    with ScopedTiming("total", debug_mode > 0):
        
        if rgb888p_img.format() == image.RGBP888:
            ai2d_input = rgb888p_img.to_numpy_ref()
            ai2d_input_tensor = nn.from_numpy(ai2d_input)
            
            # 使用ai2d进行预处理
            _ai2d_builder.run(ai2d_input_tensor, _ai2d_output_tensor)
            
            # 设置模型输入
            _kpu.set_input_tensor(0, _ai2d_output_tensor)
            
            # 模型推理
            _kpu.run()
            
            # 获取模型输出
            results = []
            for i in range(_kpu.outputs_size()):
                out_data = _kpu.get_output_tensor(i)
                result = out_data.to_numpy()
                result = result.reshape((result.shape[0] * result.shape[1] * result.shape[2] * result.shape[3]))
                del out_data
                results.append(result)
            
            # 使用aicube模块封装的接口进行后处理
            det_boxes = aicube.anchorbasedet_post_process(
                results[0],
                results[1],
                results[2],
                _kmodel_frame_size,
                _frame_size,
                _strides,
                _num_classes,
                _confidence_threshold,
                _nms_threshold,
                _anchors,
                _nms_option,
            )
            
            # 绘制结果
            _osd_img.clear()
            if det_boxes:
                for det_boxe in det_boxes:
                    x1, y1, x2, y2 = det_boxe[2], det_boxe[3], det_boxe[4], det_boxe[5]

                    x1 = x1 * DISPLAY_WIDTH // OUT_RGB888P_WIDTH
                    y1 = y1 * DISPLAY_HEIGHT // OUT_RGB888P_HEIGHT
                    x2 = x2 * DISPLAY_WIDTH // OUT_RGB888P_WIDTH
                    y2 = y2 * DISPLAY_HEIGHT // OUT_RGB888P_HEIGHT
                    
                    w = x2 - x1
                    h = y2 - y1

                    _osd_img.draw_rectangle(x1, y1, w, h, color=_color_four[det_boxe[0]][1:])
                    text = _labels[det_boxe[0]] + " " + str(round(det_boxe[1], 2))
                    _osd_img.draw_string_advanced(x1, y1 - 40, 32, text, color=_color_four[det_boxe[0]][1:])
                    
                    # 保存检测结果
                    det_results.append({
                        "class": _labels[det_boxe[0]],
                        "confidence": det_boxe[1],
                        "x": x1,
                        "y": y1,
                        "w": w,
                        "h": h
                    })
            
            Display.show_image(_osd_img, 0, 0, Display.LAYER_OSD3)
            gc.collect()
    
    return det_results

# 两侧填充参数计算函数
def _two_side_pad_param(input_size, output_size):
    ratio_w = output_size[0] / input_size[0]  # 宽度缩放比例
    ratio_h = output_size[1] / input_size[1]  # 高度缩放比例
    ratio = min(ratio_w, ratio_h)  # 取较小的缩放比例
    new_w = int(ratio * input_size[0])  # 新宽度
    new_h = int(ratio * input_size[1])  # 新高度
    dw = (output_size[0] - new_w) / 2  # 宽度差
    dh = (output_size[1] - new_h) / 2  # 高度差
    top = int(round(dh - 0.1))
    bottom = int(round(dh + 0.1))
    left = int(round(dw - 0.1))
    right = int(round(dw - 0.1))
    return top, bottom, left, right, ratio

# 读取deploy_config.json配置文件
def _read_deploy_config(config_path):
    # 打开JSON文件以进行读取deploy_config
    with open(config_path, "r") as json_file:
        try:
            # 从文件中加载JSON数据
            config = ujson.load(json_file)
        except ValueError as e:
            print("JSON 解析错误:", e)
    return config

# 模型相关变量
_detection_initialized = False
_kpu = None
_ai2d_builder = None
_sensor = None
_osd_img = None
_ai2d_output_tensor = None
_deploy_conf = None
_kmodel_frame_size = None
_frame_size = None
_labels = None
_num_classes = None
_color_four = None
_confidence_threshold = None
_nms_threshold = None
_anchors = None
_nms_option = None
_strides = None
