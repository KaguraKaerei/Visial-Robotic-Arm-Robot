import sensor

class TrackIdentifier:
    def __init__(self):
        # 默认灰色赛道阈值(L Min, L Max, A Min, A Max, B Min, B Max)
        self.GRAY_THRESHOLD_GO = (50, 85,  -10, 10,  -8, 8)
        self.GRAY_THRESHOLD_LEFT = (50, 85, -10, 10,  -8, 8)
        self.GRAY_THRESHOLD_RIGHT = (50, 85, -10, 10,  -8, 8)
        # ROI区域定义
        self.IMG_W = 320
        self.IMG_H = 240
        self.ROI_GO = (self.IMG_W//2-50, self.IMG_H//2-40, 100, 60)         # 前进区域
        self.ROI_LEFT = (0, self.IMG_H//2-40, 100, 60)                      # 左转区域
        self.ROI_RIGHT = (self.IMG_W-100, self.IMG_H//2-40, 100, 60)        # 右转区域
        # 状态判断阈值
        self.GO_THRESHOLD = 0.6
        self.LEFT_THRESHOLD = 0.5
        self.RIGHT_THRESHOLD = 0.5

    def SetGrayThreshold(self, img):
        # 取第一次看到的图像作为灰色参考
        grayRef = img.get_statistics(roi=self.ROI_GO)
        lMean = grayRef.l_mean()
        aMean = grayRef.a_mean()
        bMean = grayRef.b_mean()
        lStd = grayRef.l_stdev()
        aStd = grayRef.a_stdev()
        bStd = grayRef.b_stdev()

        factor = 4.8
        lMin = max(0, lMean - int(factor * lStd))
        lMax = min(255, lMean + int(factor * lStd))
        aMin = max(-128, aMean - int(factor * aStd * 0.5))
        aMax = min(127, aMean + int(factor * aStd * 0.5))
        bMin = max(-128, bMean - int(factor * bStd * 0.5))
        bMax = min(127, bMean + int(factor * bStd * 0.5))

        self.GRAY_THRESHOLD_GO = (lMin, lMax, aMin, aMax, bMin, bMax)
        self.GRAY_THRESHOLD_LEFT = (int(lMin * 0.7), int(lMax * 0.7), 
                                    int(aMin * 0.7), int(aMax * 0.7), 
                                    int(bMin * 0.7), int(bMax * 0.7))
        self.GRAY_THRESHOLD_RIGHT = (int(lMin * 0.7), int(lMax * 0.7),
                                    int(aMin * 0.7), int(aMax * 0.7),
                                    int(bMin * 0.7), int(bMax * 0.7))

        print(f"Set Gray Threshold GO: {self.GRAY_THRESHOLD_GO}")
        print(f"Set Gray Threshold LEFT: {self.GRAY_THRESHOLD_LEFT}")
        print(f"Set Gray Threshold RIGHT: {self.GRAY_THRESHOLD_RIGHT}")

    def GetGrayTrack(self, img, roi, GRAY_THRESHOLD):
        minBlobSize = max(10, (roi[2]*roi[3])//100)     # 最小灰色面积
        blobs = img.find_blobs([GRAY_THRESHOLD], roi=roi, pixels_threshold=minBlobSize, area_threshold=minBlobSize, merge=True)
        if not blobs: return 0

        totalGrayArea = sum(blob.pixels() for blob in blobs)
        roiArea = roi[2] * roi[3]

        grayRatio = min(1.0, totalGrayArea / roiArea)
        return grayRatio
    
    def TrackDetect(self, img):
        # 计算各区域灰色比例
        grayGo = self.GetGrayTrack(img, self.ROI_GO, self.GRAY_THRESHOLD_GO)
        grayLeft = self.GetGrayTrack(img, self.ROI_LEFT, self.GRAY_THRESHOLD_LEFT)
        grayRight = self.GetGrayTrack(img, self.ROI_RIGHT, self.GRAY_THRESHOLD_RIGHT)
        # 状态判断
        if grayLeft > self.LEFT_THRESHOLD:
            state = "LEFT"
        elif grayRight > self.RIGHT_THRESHOLD:
            state = "RIGHT"
        elif grayGo > self.GO_THRESHOLD:
            state = "GO"
        else:
            state = "STOP"
        # 可视化
        img.draw_rectangle(self.ROI_GO, color=(255, 255, 255))
        img.draw_rectangle(self.ROI_LEFT, color=(255, 255, 255))
        img.draw_rectangle(self.ROI_RIGHT, color=(255, 255, 255))
        img.draw_string(self.IMG_W//2-30, 10, state, color=(255, 255, 255), scale=2)

        return state
