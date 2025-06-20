import cv2
import numpy as np

# 你的裁剪参数（之前测量的）
# ROI裁剪参数
x_start = 209
x_end = 538
y_start = 0
y_end = 545

# 记录鼠标坐标的全局变量
mouse_x = 0
mouse_y = 0

# 鼠标回调函数
def mouse_callback(event, x, y, flags, param):
    global mouse_x, mouse_y
    if event == cv2.EVENT_MOUSEMOVE:
        mouse_x = x
        mouse_y = y

# 打开摄像头
cap = cv2.VideoCapture(0)
cv2.namedWindow("ROI Picker")
cv2.setMouseCallback("ROI Picker", mouse_callback)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    roi = frame[y_start:y_end, x_start:x_end].copy()

    # 在画面上实时显示鼠标像素坐标
    text = f"X: {mouse_x}, Y: {mouse_y}"
    cv2.putText(roi, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    # 显示ROI画面
    cv2.imshow("ROI Picker", roi)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        print("最终鼠标像素坐标:")
        print(f"cx_roi = {mouse_x}")
        print(f"cy_roi = {mouse_y}")
        break

cap.release()
cv2.destroyAllWindows()
