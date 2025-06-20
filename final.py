import cv2
import numpy as np
import serial
import time


# 串口初始化
arduino = serial.Serial(port='COM3', baudrate=115200, timeout=1)
time.sleep(2)

# ROI裁剪参数
x_start = 209
x_end = 538
y_start = 0
y_end = 545

# HSV检测参数
lower_hsv = np.array([17, 0, 0])
upper_hsv = np.array([90, 255, 255])

# 标定参数更新
pixel_origin_x = 269
pixel_origin_y = 0

pixel_to_mm_ratio = 1  # 暂时估算，未来可以通过完整标尺更新

# 步进换算系数（每 1mm 多少步）
x_steps_per_mm = 50
y_steps_per_mm = 50
# 定义当前位置记录
current_x_mm = 0
current_y_mm = 0

# 启动摄像头
cap = cv2.VideoCapture(0)
time.sleep(1)

# 采样图像
ret, frame = cap.read()
if not ret:
    print("摄像头读取失败")
    exit()

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

        print("============== 位移计算报告 ==============")
        print(f"图像原点像素位置: ({pixel_origin_x}, {pixel_origin_y})")
        print(f"检测目标像素位置: ({cx_roi}, {cy_roi})")

        # 注意这里反向了
        delta_x_pixel = pixel_origin_x - cx_roi
        delta_y_pixel = pixel_origin_y - cy_roi

        delta_x_mm = delta_x_pixel * pixel_to_mm_ratio
        delta_y_mm = delta_y_pixel * pixel_to_mm_ratio

        print(f"X偏移: {delta_x_mm:.2f} mm")
        print(f"Y偏移: {delta_y_mm:.2f} mm")

        # 四舍五入取整
        move_x_mm = int(round(delta_x_mm))
        move_y_mm = int(round(delta_y_mm))

        # 换算成步数
        move_x_steps = move_x_mm * x_steps_per_mm
        move_y_steps = move_y_mm * y_steps_per_mm

        # 发送一次性串口指令
        command = f"x{move_x_steps}y{move_y_steps}\n"
        arduino.write(command.encode())
        print("已发送指令:", command)

        # 更新当前位置
        current_x_mm += move_x_mm
        current_y_mm += move_y_mm

        time.sleep(10)

        print("目标移动完成")
    else:
        print("质心计算失败")
else:
    print("未检测到目标")

# ======== 新增：输入框触发回原点逻辑 ========

while True:
    cmd = input("\n输入 'r' 执行回归原点，输入 'q' 退出： ")
    if cmd == 'r':
        print("开始回归原点...")
        back_x_mm = -current_x_mm
        back_y_mm = -current_y_mm

        back_x_steps = back_x_mm * x_steps_per_mm
        back_y_steps = back_y_mm * y_steps_per_mm

        command = f"x{back_x_steps}y{back_y_steps}\n"
        arduino.write(command.encode())
        print("已发送回原点指令:", command)

        time.sleep(10)
        print("已返回原点")
        break
    elif cmd == 'q':
        print("退出程序")
        break
    else:
        print("无效指令，请重新输入。")

cap.release()
cv2.destroyAllWindows()
arduino.close()
