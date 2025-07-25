import cv2
import numpy as np

def nothing(x):
    pass

# 打开摄像头
cap = cv2.VideoCapture(0)

# === 填入你从 roi_crop_tuner.py 得到的裁剪参数 ===
# ======== 填入裁剪参数 (来自 roi_crop_tuner.py) ========
x_start = 209
x_end = 545
y_start = 0
y_end = 445


cv2.namedWindow('HSV Tuner')

# HSV滑动条
cv2.createTrackbar('H Min','HSV Tuner', 0, 179, nothing)
cv2.createTrackbar('H Max','HSV Tuner', 179, 179, nothing)
cv2.createTrackbar('S Min','HSV Tuner', 0, 255, nothing)
cv2.createTrackbar('S Max','HSV Tuner', 255, 255, nothing)
cv2.createTrackbar('V Min','HSV Tuner', 0, 255, nothing)
cv2.createTrackbar('V Max','HSV Tuner', 255, 255, nothing)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    roi = frame[y_start:y_end, x_start:x_end]

    # 获取滑动条值
    h_min = cv2.getTrackbarPos('H Min','HSV Tuner')
    h_max = cv2.getTrackbarPos('H Max','HSV Tuner')
    s_min = cv2.getTrackbarPos('S Min','HSV Tuner')
    s_max = cv2.getTrackbarPos('S Max','HSV Tuner')
    v_min = cv2.getTrackbarPos('V Min','HSV Tuner')
    v_max = cv2.getTrackbarPos('V Max','HSV Tuner')

    lower = np.array([h_min, s_min, v_min])
    upper = np.array([h_max, s_max, v_max])

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)
    result = cv2.bitwise_and(roi, roi, mask=mask)

    cv2.imshow('Mask', result)
    cv2.imshow('ROI', roi)

    # 实时打印方便你复制
    print(f'Lower HSV: {lower}, Upper HSV: {upper}', end='\r')

    if cv2.waitKey(1) & 0xFF == ord('q'):
        print(f'\n最终 HSV 参数: Lower={lower}, Upper={upper}')
        break

cap.release()
cv2.destroyAllWindows()
