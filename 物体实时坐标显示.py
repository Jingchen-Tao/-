"检测一个"
import cv2
import numpy as np
import time

# ROI裁剪参数
x_start = 209
x_end = 538
y_start = 0
y_end = 545

# HSV检测参数
lower_hsv = np.array([17, 0, 0])
upper_hsv = np.array([90, 255, 255])


# 标定参数（全部修正）
pixel_origin_x = 285
pixel_origin_y = 25
pixel_to_mm_ratio = 50 / 285  # ≈ 0.1754 mm/px

# 启动摄像头
cap = cv2.VideoCapture(0)
time.sleep(1)

while True:
    ret, frame = cap.read()
    if not ret:
        print("摄像头读取失败")
        break

    roi = frame[y_start:y_end, x_start:x_end]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest_contour)
        if M['m00'] != 0:
            cx_roi = int(M['m10'] / M['m00'])
            cy_roi = int(M['m01'] / M['m00'])

            # 计算实际坐标（注意X方向反向）
            delta_x_pixel = -(cx_roi - pixel_origin_x)
            delta_y_pixel = cy_roi - pixel_origin_y

            delta_x_mm = delta_x_pixel * pixel_to_mm_ratio
            delta_y_mm = delta_y_pixel * pixel_to_mm_ratio

            # 在ROI图像上显示检测结果
            cv2.circle(roi, (cx_roi, cy_roi), 5, (0, 255, 0), -1)
            text_pixel = f"Pixel: ({cx_roi},{cy_roi})"
            text_mm = f"Real: ({delta_x_mm:.1f}mm, {delta_y_mm:.1f}mm)"
            cv2.putText(roi, text_pixel, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(roi, text_mm, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        else:
            cv2.putText(roi, "No target detected", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    else:
        cv2.putText(roi, "No target detected", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    cv2.imshow("ROI Tracking", roi)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
