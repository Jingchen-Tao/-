"""
调节hsv


Multi-blob detector (robust version)
------------------------------------
* HSV + 反光掩膜去高光
* morphologyEx(CLOSE) 合并碎块
* 检出所有轮廓，过滤 MIN_AREA
* 标注：
    ▸ 红色   – 轮廓外形
    ▸ 绿色点 – 默认外接矩形几何中心
    ▸ 白色字 – 毫米位移 & 编号
"""

import cv2
import numpy as np
import time

# =============== 参数区 ===============
# ROI
X_START, X_END = 170, 510
Y_START, Y_END = 0,   545

# 初始 HSV 阈值（可通过 TrackBar 实时调）
LOWER_H, LOWER_S, LOWER_V = 0, 45, 135
UPPER_H, UPPER_S, UPPER_V = 11, 120, 255

# 反光 (高亮 + 低饱和) 阈值
REFLECT_LOWER = np.array([0,   0, 230])
REFLECT_UPPER = np.array([180, 50, 255])

# 像素→毫米标定
PX_ORIGIN = (285, 25)             # 机械臂当前像素原点
PX2MM     = 50 / 285              # ≈0.1754 mm/px

MIN_AREA  = 300                   # 面积下限
USE_MOMENTS_CENTER = False        # True=形心, False=外接矩形中心
# =====================================

def nothing(x):         # TrackBar 回调（空）
    pass

# TrackBars 方便在线调阈值
cv2.namedWindow("mask", cv2.WINDOW_NORMAL)
for name, init in zip(
        ["LH","LS","LV","UH","US","UV"],
        [LOWER_H,LOWER_S,LOWER_V,UPPER_H,UPPER_S,UPPER_V]):
    cv2.createTrackbar(name, "mask", init, 255, nothing)

cap = cv2.VideoCapture(0)
time.sleep(1)

kernel_close = np.ones((5,5), np.uint8)

while True:
    ok, frame = cap.read()
    if not ok:
        print("摄像头读取失败"); break

    roi = frame[Y_START:Y_END, X_START:X_END]

    # —— 1. 实时读 TrackBar，生成阈值 —— #
    lH = cv2.getTrackbarPos("LH", "mask")
    lS = cv2.getTrackbarPos("LS", "mask")
    lV = cv2.getTrackbarPos("LV", "mask")
    uH = cv2.getTrackbarPos("UH", "mask")
    uS = cv2.getTrackbarPos("US", "mask")
    uV = cv2.getTrackbarPos("UV", "mask")

    lower_hsv = np.array([lH, lS, lV])
    upper_hsv = np.array([uH, uS, uV])

    # —— 2. 颜色掩膜 & 反光剔除 —— #
    hsv   = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv, lower_hsv, upper_hsv)
    refl  = cv2.inRange(hsv, REFLECT_LOWER, REFLECT_UPPER)
    mask  = cv2.bitwise_and(mask1, cv2.bitwise_not(refl))

    # —— 3. 闭运算合并碎片 —— #
    mask_closed = cv2.morphologyEx(mask, cv2.MORPH_CLOSE,
                                   kernel_close, iterations=2)

    # —— 4. 找轮廓 —— #
    contours, _ = cv2.findContours(mask_closed,
                                   cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)

    # —— 5. 遍历并标注 —— #
    for i, cnt in enumerate(contours, 1):
        area = cv2.contourArea(cnt)
        if area < MIN_AREA:
            continue

        # ▸ 坐标中心
        if USE_MOMENTS_CENTER:
            M = cv2.moments(cnt)
            if M["m00"] == 0:
                continue
            cx = int(M["m10"]/M["m00"])
            cy = int(M["m01"]/M["m00"])
        else:
            x,y,w,h = cv2.boundingRect(cnt)
            cx, cy  = x + w//2, y + h//2

        # ▸ 像素差 → mm
        dx_px = -(cx - PX_ORIGIN[0])
        dy_px =  (cy - PX_ORIGIN[1])
        dx_mm = dx_px * PX2MM
        dy_mm = dy_px * PX2MM

        # ▸ 视觉叠加
        cv2.drawContours(roi, [cnt], -1, (0,0,255), 2)          # 红轮廓
        cv2.circle(roi, (cx,cy), 3, (0,255,0), -1)              # 绿中心
        cv2.putText(roi, f"#{i}", (cx+6, cy-6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
        cv2.putText(roi, f"({dx_mm:.1f},{dy_mm:.1f}mm)",
                    (cx+6, cy+14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (255,255,255), 1)

        # —— 如需发送给机械臂，可在此处串口 write(dx_mm,dy_mm) —— #

    # —— 6. 显示 —— #
    cv2.imshow("ROI Tracking (q quit)", roi)
    cv2.imshow("mask", mask_closed)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
