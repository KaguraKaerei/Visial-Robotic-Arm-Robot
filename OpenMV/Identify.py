import sensor
import image

import Track
import Serial

class State:
    IDLE = 0
    TRACK = 1
    QR = 2
    EXPLOSIVE = 3
    TARGET = 4
    HOSTAGE = 5

class TaskIdentifier:
    def __init__(self):
        self.state = State.IDLE
        # 状态转移表
        self._transition = {
            State.IDLE: {
                "startTrack": State.TRACK,
            },
            State.TRACK: {
                "stopTrack": State.IDLE,
                "findQR": State.QR,
                "findExplosive": State.EXPLOSIVE,
                "findTarget": State.TARGET,
                "findHostage": State.HOSTAGE
            },
            State.QR: {
                "endQR": State.TRACK
            },
            State.EXPLOSIVE: {
                "endExplosive": State.TRACK
            },
            State.TARGET: {
                "endTarget": State.TRACK
            },
            State.HOSTAGE: {
                "endHostage": State.TRACK
            }
        }
        # 赛道识别器实例化
        self.trackIdentifier = Track.TrackIdentifier()
        self.lastTrackState = "STOP"
        self.serialer = Serial.Serialer()
        # 各任务识别结果缓存
        self._qrPayloads = [0, 0, 0]
        self._explosive = [0, 0]
        self._target = [0, 0]
        self._hostage = [0, 0]
        # 任务完成数量标志
        self._taskDoneFlags = {
            "QR": False,
            "Explosive": False,
            "Target": False,
            "Hostage": False
        }

    def IdentifyProcess(self, img:image.Image):
        if self.state == State.IDLE:
            self.Trigger("startTrack")

        # TODO: 这里要做成在赛道识别路上识别到左右两边有其他任务：二维码、爆炸物、标靶、人质 时转动云台再进行具体识别
        elif self.state == State.TRACK:
            currentTrackState = self.trackIdentifier.TrackDetect(img)
            if currentTrackState != self.lastTrackState:
                self.serialer.SendTrackState(currentTrackState)
                self.lastTrackState = currentTrackState

            if not self._taskDoneFlags["QR"]:
                res, payloads = self.FindQR(img)
                if res:
                    self._qrPayloads = payloads
                    self.Trigger("findQR")

            if not self._taskDoneFlags["Explosive"]:
                res, pos = self.FindExplosive(img)
                if res:
                    self._explosive = pos
                    self.Trigger("findExplosive")

            if not self._taskDoneFlags["Target"]:
                res, pos = self.FindTarget(img)
                if res:
                    self._target = pos
                    self.Trigger("findTarget")

            # if not self._taskDoneFlags["Hostage"]:
            res, pos = self.FindHostage(img)
                # if res:
                #     self._hostage = pos
                #     self.Trigger("findHostage")

        # TODO: 这些是真正处理任务的地方
        elif self.state == State.QR:

            print(f"Find QR Codes: {self._qrPayloads}")
            self._taskDoneFlags["QR"] = True
            self.Trigger("endQR")

        elif self.state == State.EXPLOSIVE:

            print(f"Find Explosive at: {self._explosive}")
            self._taskDoneFlags["Explosive"] = True
            self.Trigger("endExplosive")

        elif self.state == State.TARGET:
            offset = 5
            self._target = [self._target[0], self._target[1]+offset]

            print(f"Find Target at: {self._target}")
            self._taskDoneFlags["Target"] = True
            self.Trigger("endTarget")

        elif self.state == State.HOSTAGE:

            print(f"Find Hostage at: {self._hostage}")
            self._taskDoneFlags["Hostage"] = True
            self.Trigger("endHostage")

    def Trigger(self, event: str):
        nextState = self._transition.get(self.state, {}).get(event)
        if nextState is None:
            print(f"Invalid transition: {self.state} --{event}--> ?")
            return False
        self.state = nextState
        print(f"State changed to: {self.state}")
        return True

    def FindQR(self, img:image.Image):
        codes = img.find_qrcodes()
        if not codes: return False, []
        payloads = []
        for code in codes:
            try:
                txt = code.payload()
            except:
                txt = ""
            payloads.append(txt)

        return True, payloads

    def FindExplosive(self, img:image.Image):
        explosiveColor = (0, 0, 0, 0, 0, 0)
        if self._qrPayloads[0] == 1:
            explosiveColor = (30, 100, -64, -8, -32, 32)   # 红色
        elif self._qrPayloads[0] == 2:
            explosiveColor = (30, 100, -64, -8, -32, 32)   # 绿色
        elif self._qrPayloads[0] == 3:
            explosiveColor = (30, 100, -64, -8, -32, 32)   # 蓝色
        else:
            return False, None

        blobs = img.find_blobs([explosiveColor], pixels_threshold=100, area_threshold=100, merge=True)
        if not blobs: return False, None

        blob = max(blobs, key=lambda blob: blob.pixels())
        x, y = blob.cx(), blob.cy()

        return True, (x, y)

    def FindTarget(self, img:image.Image):
        targetColor = (0, 0, 0, 0, 0, 0)
        if self._qrPayloads[1] == 1:
            targetColor = (30, 100, -64, -8, -32, 32)   # 红色
        elif self._qrPayloads[1] == 2:
            targetColor = (30, 100, -64, -8, -32, 32)   # 绿色
        elif self._qrPayloads[1] == 3:
            targetColor = (30, 100, -64, -8, -32, 32)   # 蓝色
        else:
            return False, None

        blobs = img.find_blobs([targetColor], pixels_threshold=100, area_threshold=100, merge=True)
        if not blobs: return False, None

        for blob in blobs:
            circles = img.find_circles(roi=blob.rect(), threshold=2000, x_margin=10, y_margin=10, r_margin=10, r_min=10, r_max=10, r_step=2)
            if circles:
                circle = circles[0]
                cx, cy = circle.x(), circle.y()

                return True, (cx, cy)

        return False, None

    def FindHostage(self, img:image.Image):

        # XXX:
        self._qrPayloads = [0, 0, 3]   # for test

        hostageShape = " "
        if self._qrPayloads[2] == 1:
            hostageShape = "cylinder"       # 圆柱
        elif self._qrPayloads[2] == 2:
            hostageShape = "trapezoid"      # 圆梯
        elif self._qrPayloads[2] == 3:
            hostageShape = "waist_drum"     # 腰鼓
        else:
            return False, None

        WHITE_THRESHOLD = (70, 98, -12, 10, -18, 0)
        blobs = img.find_blobs([WHITE_THRESHOLD], pixels_threshold=100, area_threshold=100, merge=True)
        if not blobs: return False, None

        for blob in blobs:
            roiTop = (blob.x(), blob.y() + int(blob.h() * 0.1), blob.w(), 5)
            roiMid = (blob.x(), blob.y() + int(blob.h() * 0.5), blob.w(), 5)
            roiBot = (blob.x(), blob.y() + int(blob.h() * 0.9), blob.w(), 5)
            roiLeft = (blob.x() + int(blob.w() * 0.1), blob.y(), 5, blob.h())
            roiCenter = (blob.x() + int(blob.w() * 0.5), blob.y(), 5, blob.h())
            roiRight = (blob.x() + int(blob.w() * 0.9), blob.y(), 5, blob.h())

            blobsTop = img.find_blobs([WHITE_THRESHOLD], roi=roiTop, pixels_threshold=5, area_threshold=5, merge=True)
            blobsMid = img.find_blobs([WHITE_THRESHOLD], roi=roiMid, pixels_threshold=5, area_threshold=5, merge=True)
            blobsBot = img.find_blobs([WHITE_THRESHOLD], roi=roiBot, pixels_threshold=5, area_threshold=5, merge=True)
            blobsLeft = img.find_blobs([WHITE_THRESHOLD], roi=roiLeft, pixels_threshold=5, area_threshold=5, merge=True)
            blobsCenter = img.find_blobs([WHITE_THRESHOLD], roi=roiCenter, pixels_threshold=5, area_threshold=5, merge=True)
            blobsRight = img.find_blobs([WHITE_THRESHOLD], roi=roiRight, pixels_threshold=5, area_threshold=5, merge=True)

            widthTop = sum(b.pixels() for b in blobsTop)
            widthMid = sum(b.pixels() for b in blobsMid)
            widthBot = sum(b.pixels() for b in blobsBot)
            widthLeft = sum(b.pixels() for b in blobsLeft)
            widthCenter = sum(b.pixels() for b in blobsCenter)
            widthRight = sum(b.pixels() for b in blobsRight)

            if not (widthTop and widthMid and widthBot): continue
            if not (widthLeft and widthCenter and widthRight): continue

            ratioTop = widthTop / widthMid
            ratioBottom = widthBot / widthMid
            ratioLeft = widthLeft / widthCenter
            ratioRight = widthRight / widthCenter

            if hostageShape == "cylinder":
                if 0.9 < ratioTop < 1.1 and 0.9 < ratioBottom < 1.1 and 0.9 < ratioLeft < 1.1 and 0.9 < ratioRight < 1.1:
                    print(f"Hostage Ratios: Top {ratioTop:.2f}, Bottom {ratioBottom:.2f}, Left {ratioLeft:.2f}, Right {ratioRight:.2f}")
                    print(f"x, y: {blob.cx()}, {blob.cy()}")
                    return True, (blob.cx(), blob.cy())
            elif hostageShape == "trapezoid":
                if ratioTop < 0.9 and ratioBottom > 1.1 and ratioLeft > 1.1 and ratioRight > 1.1:
                    print(f"Hostage Ratios: Top {ratioTop:.2f}, Bottom {ratioBottom:.2f}, Left {ratioLeft:.2f}, Right {ratioRight:.2f}")
                    print(f"x, y: {blob.cx()}, {blob.cy()}")
                    return True, (blob.cx(), blob.cy())
            elif hostageShape == "waist_drum":
                if ratioTop < 0.9 and ratioBottom < 0.9 and ratioLeft > 1.1 and ratioRight > 1.1:
                    print(f"Hostage Ratios: Top {ratioTop:.2f}, Bottom {ratioBottom:.2f}, Left {ratioLeft:.2f}, Right {ratioRight:.2f}")
                    print(f"x, y: {blob.cx()}, {blob.cy()}")
                    return True, (blob.cx(), blob.cy())

        return False, None
