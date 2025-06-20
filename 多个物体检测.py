import cv2
import numpy as np
import time

# ====== ROI裁剪区域 ======
x_start = 209
x_end = 538
y_start = 0
y_end = 545

# ====== 主目标颜色范围（放宽V上限）======
lower_hsv = np.array([100, 100, 80])   # H, S, V
upper_hsv = np.array([125, 255, 255])


# ====== 反光区域特征（高亮 + 低饱和）======
reflection_lower = np.array([0, 0, 230])
reflection_upper = np.array([180, 50, 255])

# ====== 坐标参数 ======
pixel_origin_x = 285
pixel_origin_y = 25
pixel_to_mm_ratio = 50 / 285  # ≈ 0.1754 mm/px

# ====== 用户输入想标记的最大目标数量 ======
target_count = 7

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

    # 主目标遮罩
    mask = cv2.inRange(hsv, lower_hsv, upper_hsv)

    # 柔和的反光剔除处理
    reflection_mask = cv2.inRange(hsv, reflection_lower, reflection_upper)
    kernel = np.ones((3, 3), np.uint8)
    reflection_mask = cv2.dilate(reflection_mask, kernel, iterations=1)
    mask[reflection_mask > 0] = 0

    # 滤波 + 形态学
    mask = cv2.medianBlur(mask, 5)
    mask = cv2.dilate(mask, kernel, iterations=1)
    mask = cv2.erode(mask, kernel, iterations=1)

    # 轮廓检测
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        contour_areas = [(cv2.contourArea(c), c) for c in contours if cv2.contourArea(c) >= 300]
        contour_areas.sort(reverse=True)
        top_contours = contour_areas[:target_count]

        # 分区边界（横向划分）
        roi_width = roi.shape[1]
        zone_width = roi_width / target_count

        # 可视化分区线
        for z in range(1, target_count):
            x_line = int(z * zone_width)
            cv2.line(roi, (x_line, 0), (x_line, roi.shape[0]), (128, 128, 128), 1, lineType=cv2.LINE_AA)

        for i, (area, contour) in enumerate(top_contours):
            M = cv2.moments(contour)
            if M['m00'] == 0:
                continue

            cx_roi = int(M['m10'] / M['m00'])
            cy_roi = int(M['m01'] / M['m00'])

            # 横向区域编号（0 ~ target_count-1）
            zone_id = int(cx_roi // zone_width)
            if zone_id >= target_count:
                zone_id = target_count - 1

            # 坐标换算（像素 → mm）
            delta_x_pixel = -(cx_roi - pixel_origin_x)
            delta_y_pixel = cy_roi - pixel_origin_y
            delta_x_mm = delta_x_pixel * pixel_to_mm_ratio
            delta_y_mm = delta_y_pixel * pixel_to_mm_ratio

            # 显示信息
            cv2.circle(roi, (cx_roi, cy_roi), 5, (0, 255, 0), -1)
            text_pixel = f"P{i}: ({cx_roi},{cy_roi})"
            text_mm = f"R{i}: ({delta_x_mm:.1f}mm,{delta_y_mm:.1f}mm)"
            text_zone = f"Z{i}: 区 {zone_id}"
            cv2.putText(roi, text_pixel, (cx_roi + 10, cy_roi - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(roi, text_mm, (cx_roi + 10, cy_roi),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(roi, text_zone, (cx_roi + 10, cy_roi + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

    else:
        cv2.putText(roi, "No target detected", (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    cv2.imshow("ROI Tracking", roi)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
