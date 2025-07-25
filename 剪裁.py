import cv2
import numpy as np

def nothing(x):
    pass

# 打开摄像头
cap = cv2.VideoCapture(0)
ret, frame = cap.read()
height, width, _ = frame.shape

cv2.namedWindow("Crop Tuner")

# 创建滑动条 (最大值直接用当前分辨率)
cv2.createTrackbar("x_start", "Crop Tuner", 0, width, nothing)
cv2.createTrackbar("x_end", "Crop Tuner", width, width, nothing)
cv2.createTrackbar("y_start", "Crop Tuner", 0, height, nothing)
cv2.createTrackbar("y_end", "Crop Tuner", height, height, nothing)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # 读取当前滑动条值
    x_start = cv2.getTrackbarPos("x_start", "Crop Tuner")
    x_end = cv2.getTrackbarPos("x_end", "Crop Tuner")
    y_start = cv2.getTrackbarPos("y_start", "Crop Tuner")
    y_end = cv2.getTrackbarPos("y_end", "Crop Tuner")

    # 防止调反
    if x_start > x_end:
        x_start, x_end = x_end, x_start
    if y_start > y_end:
        y_start, y_end = y_end, y_start

    # 绘制当前裁剪框
    frame_copy = frame.copy()
    cv2.rectangle(frame_copy, (x_start, y_start), (x_end, y_end), (0, 255, 0), 2)

    # 显示裁剪框
    cv2.imshow("Crop Tuner", frame_copy)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        print("\n最终裁剪参数:")
        print(f"x_start = {x_start}")
        print(f"x_end = {x_end}")
        print(f"y_start = {y_start}")
        print(f"y_end = {y_end}")
        break

cap.release()
cv2.destroyAllWindows()
