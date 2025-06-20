"""
暂时不做实时修正，还是常数参数t代替机械臂过去的时间，坐标也就一个，机械臂原点也就是坐标系0点，从检测到物体重心开始根据位移差，
过去的时间t，和物体在t内的修正坐标设计xy应该到达的位置（显然只有y变化）x不变，到达指定位置后随物体一起以v传送带运动
"""
#没有用
import cv2
import numpy as np
import serial
import time

# 串口初始化
arduino = serial.Serial(port='COM3', baudrate=9600, timeout=1)
time.sleep(2)

# ROI裁剪参数
x_start = 209
x_end = 545
y_start = 0
y_end = 445

# HSV检测参数
lower_hsv = np.array([0, 0, 125])
upper_hsv = np.array([152, 16, 255])

# 标定参数
pixel_origin_x = 60
pixel_origin_y = 18
pixel_to_mm_ratio = 1.15  # 像素 → mm

# 步进换算系数（每 1mm 多少步）
x_steps_per_mm = 390
y_steps_per_mm = 1979

# 系统控制参数
V_conveyor = 41  # 传送带速度 (mm/s)
T_fixed = 2.5  # 固定平均移动时间 (s)

# 启动摄像头
cap = cv2.VideoCapture(0)
time.sleep(1)

# 记录是否已检测到目标
target_locked = False

while True:
    ret, frame = cap.read()
    if not ret:
        print("摄像头读取失败")
        break

    roi = frame[y_start:y_end, x_start:x_end]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not target_locked and contours:
        # 第一次检测
        largest_contour = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest_contour)
        if M['m00'] != 0:
            cx_roi = int(M['m10'] / M['m00'])
            cy_roi = int(M['m01'] / M['m00'])

            print("检测到物体重心：", cx_roi, cy_roi)

            # 计算像素差
            delta_x_pixel = pixel_origin_x - cx_roi
            delta_y_pixel = pixel_origin_y - cy_roi

            # 像素差转物理距离
            delta_x_mm = delta_x_pixel * pixel_to_mm_ratio
            delta_y_mm = delta_y_pixel * pixel_to_mm_ratio

            # 预测未来Y坐标
            predicted_y_mm = delta_y_mm + V_conveyor * T_fixed

            # 发送第一次移动命令
            move_x_mm = int(round(delta_x_mm))
            move_y_mm = int(round(predicted_y_mm))

            move_x_steps = move_x_mm * x_steps_per_mm
            move_y_steps = move_y_mm * y_steps_per_mm

            command = f"x{move_x_steps}y{move_y_steps}\n"
            arduino.write(command.encode())
            print("发送首次定位指令:", command.strip())

            target_locked = True  # 只检测一次
            time.sleep(T_fixed + 0.5)  # 等机械臂定位完成

        else:
            print("质心计算失败")

    elif target_locked:
        # 进入恒速跟随模式
        move_y_mm = V_conveyor * 0.1  # 每次跟随 0.1秒的步长

        move_y_steps = int(round(move_y_mm * y_steps_per_mm))
        command = f"x0y{move_y_steps}\n"
        arduino.write(command.encode())
        print("发送跟随指令:", command.strip())
        time.sleep(0.1)

    else:
        print("等待检测物体...")

cap.release()
arduino.close()
cv2.destroyAllWindows()
