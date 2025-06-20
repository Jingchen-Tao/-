import cv2
import numpy as np
import serial
import time

# ====== 串口初始化 ======
arduino = serial.Serial(port='COM3', baudrate=115200, timeout=1)
time.sleep(2)

# ====== 参数设置 ======
x_start, x_end = 209, 538
y_start, y_end = 0, 545

lower_hsv = np.array([100, 100, 80])   # H, S, V
upper_hsv = np.array([125, 255, 255])


reflection_lower = np.array([0, 0, 230])
reflection_upper = np.array([180, 50, 255])
kernel = np.ones((3, 3), np.uint8)

pixel_to_mm_ratio = 0.8  # ≈0.1754 mm/px
x_steps_per_mm = 50
y_steps_per_mm = 50

pixel_origin_x = 236
pixel_origin_y = 61

target_count = 1

# ====== 摄像头读取 ======
cap = cv2.VideoCapture(0)
time.sleep(1)
ret, frame = cap.read()
if not ret:
    print("摄像头读取失败")
    cap.release()
    exit()

roi = frame[y_start:y_end, x_start:x_end]
hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
mask = cv2.inRange(hsv, lower_hsv, upper_hsv)

reflection_mask = cv2.inRange(hsv, reflection_lower, reflection_upper)
reflection_mask = cv2.dilate(reflection_mask, kernel, iterations=1)
mask[reflection_mask > 0] = 0

mask = cv2.medianBlur(mask, 5)
mask = cv2.dilate(mask, kernel, iterations=1)
mask = cv2.erode(mask, kernel, iterations=1)

# ====== 检测目标轮廓 ======
contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
contour_areas = [(cv2.contourArea(c), c) for c in contours if cv2.contourArea(c) >= 300]
contour_areas.sort(reverse=True)
top_contours = contour_areas[:target_count]

centers = []
for _, contour in top_contours:
    M = cv2.moments(contour)
    if M['m00'] != 0:
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        centers.append((cx, cy))

# 默认按 y 降序排序，先抓下面的
centers.sort(key=lambda c: c[1], reverse=True)

# ====== 移动操作部分 ======
last_grab_pixel = (pixel_origin_x, pixel_origin_y)
history = []

for idx, (cx_roi, cy_roi) in enumerate(centers):
    print(f"\n==== 处理第 {idx + 1} 个目标 ====")
    print(f"目标像素坐标: ({cx_roi}, {cy_roi})")

    # 当前机械臂像素位置 = 上一个目标的位置
    current_x_pixel, current_y_pixel = last_grab_pixel
    print(f"当前机械臂像素位置: ({current_x_pixel}, {current_y_pixel})")

    # 像素差
    delta_x_pixel = cx_roi - current_x_pixel
    delta_y_pixel = cy_roi-current_y_pixel   # 注意：图像向下为+，所以计算为“当前-目标”
    print(f"像素差: dx = {delta_x_pixel}, dy = {delta_y_pixel}")

    # 像素差 → mm
    move_x_mm = int(round(delta_x_pixel * pixel_to_mm_ratio))
    move_y_mm = int(round(delta_y_pixel * pixel_to_mm_ratio))
    print(f"对应 mm 移动: dx = {move_x_mm}, dy = {move_y_mm}")

    # mm → 步数
    move_x_steps = move_x_mm * x_steps_per_mm
    move_y_steps = move_y_mm * y_steps_per_mm
    command = f"x{move_x_steps}y{move_y_steps}\n"
    arduino.write(command.encode())
    print("已发送移动指令:", command.strip())

    time.sleep(10)

    print("Z轴下降..."); arduino.write(b"l\n"); time.sleep(1)
    print("Z轴抬起..."); arduino.write(b"r\n"); time.sleep(1)

    # 更新当前位置为这次抓取位置
    last_grab_pixel = (cx_roi, cy_roi)

    # 保存记录
    history.append({
        'target_pixel': (cx_roi, cy_roi),
        'move_mm': (move_x_mm, move_y_mm),
    })

    # 等待用户输入
    while True:
        choice = input("输入 'r' 回原点，'n' 去下一个目标，'q' 退出：").strip().lower()
        if choice == 'r':
            # 回原点指令
            back_x_pixel = pixel_origin_x - cx_roi
            back_y_pixel = pixel_origin_y - cy_roi
            back_x_mm = int(round(back_x_pixel * pixel_to_mm_ratio))
            back_y_mm = int(round(back_y_pixel * pixel_to_mm_ratio))
            command = f"x{back_x_mm * x_steps_per_mm}y{back_y_mm * y_steps_per_mm}\n"
            arduino.write(command.encode())
            print("已发送回原点指令:", command.strip())
            last_grab_pixel = (pixel_origin_x, pixel_origin_y)
            time.sleep(10)
            break
        elif choice == 'n':
            break
        elif choice == 'q':
            print("退出程序")
            cap.release()
            arduino.close()
            exit()
        else:
            print("无效输入，请重新输入。")

# ====== 显示历史记录 ======
print("\n==== 抓取历史 ====")
for i, h in enumerate(history):
    print(f"目标{i+1}: 像素 {h['target_pixel']} → mm 移动 {h['move_mm']}")

cap.release()
cv2.destroyAllWindows()
arduino.close()
