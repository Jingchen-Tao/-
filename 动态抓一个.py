#!/usr/bin/env python3
import cv2
import numpy as np
import serial
import time

# ===== 可调参数 =====
# 摄像头 ROI 区域（x0, y0, x1, y1）
X_START, Y_START = 170, 0
X_END,   Y_END   = 510, 545

# 目标物体的 HSV 颜色范围（根据实际颜色调整）
LOWER_HSV = np.array([100, 100, 80])   # 下界：色相 H, 饱和度 S, 亮度 V
UPPER_HSV = np.array([125, 255, 255])  # 上界

# 轮廓面积阈值：小于该面积的轮廓视为噪声
MIN_AREA = 300

# 去噪参数
BLUR_KERNEL_SIZE = 5                   # 中值滤波核大小
MORPH_KERNEL = np.ones((3, 3), np.uint8)  # 形态学操作核
DILATE_ITER = 1                        # 膨胀迭代次数
ERODE_ITER  = 1                        # 腐蚀迭代次数

# 运动控制阈值
X_ALIGN_THRESHOLD = 3      # 当 |cx - current_x| > 该值（像素）时，机械臂移动
Y_PICK_THRESHOLD  = 70     # 当 cy < 该值（像素）时触发取物
WAIT_BEFORE_LOWER = 0.4     # 触发后等待秒数再下降

# 像素坐标 ↔ 机械臂坐标 校准
PIXEL_ORIGIN   = np.array([223, 0])  # 机械臂 (0,0) 对应的 ROI 像素坐标
PIXEL_TO_MM    = 1                  # 像素到毫米比例（需根据标定修改）
X_STEPS_PER_MM = -49.75                # X 轴：每毫米对应的步进数
Y_STEPS_PER_MM = 49.75                # Y 轴：预留，当前未用

# 串口配置（根据实际端口修改）
SERIAL_PORT = 'COM3'
BAUD_RATE   = 115200

# ===== 初始化硬件 =====
cap = cv2.VideoCapture(0)
time.sleep(1)                       # 等摄像头曝光稳定
if not cap.isOpened():
    raise RuntimeError("无法打开摄像头")

arduino = serial.Serial(port=SERIAL_PORT, baudrate=BAUD_RATE, timeout=1)
time.sleep(2)                       # 等串口初始化

def send_xy(dx_mm: float, dy_mm: float):
    """发送 XY 相对位移指令到 Arduino"""
    dx_steps = int(round(dx_mm * X_STEPS_PER_MM))
    dy_steps = int(round(dy_mm * Y_STEPS_PER_MM))
    cmd = f"x{dx_steps}y{dy_steps}\n"
    arduino.write(cmd.encode())
    print(f"已发送：{cmd.strip()}")

# 当前机械臂像素坐标（初始化为校准原点）
current_px = PIXEL_ORIGIN.copy()

print("[INFO] 开始追踪，按 q 退出。")
try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("摄像头画面获取失败")
            break

        # 1. 裁剪 ROI 并转 HSV
        roi = frame[Y_START:Y_END, X_START:X_END]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # 2. 根据颜色阈值生成二值掩膜
        mask = cv2.inRange(hsv, LOWER_HSV, UPPER_HSV)

        # 3. （可选）滤除高亮反光
        # reflection_mask = cv2.inRange(hsv, np.array([0,0,230]), np.array([180,50,255]))
        # mask[reflection_mask > 0] = 0

        # 4. 去噪：中值滤波 + 膨胀 + 腐蚀
        mask = cv2.medianBlur(mask, BLUR_KERNEL_SIZE)
        mask = cv2.dilate(mask, MORPH_KERNEL, iterations=DILATE_ITER)
        mask = cv2.erode(mask,  MORPH_KERNEL, iterations=ERODE_ITER)

        # 5. 找轮廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            cv2.imshow("Mask", mask)
            cv2.imshow("ROI",  roi)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            continue

        # 取面积最大轮廓
        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < MIN_AREA:
            cv2.imshow("Mask", mask)
            cv2.imshow("ROI",  roi)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            continue

        # 6. 计算质心
        M = cv2.moments(largest)
        if M["m00"] == 0:
            continue
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])

        # 7. 画结果（调试可视）
        cv2.drawContours(roi, [largest], -1, (0,255,0), 2)
        cv2.circle(roi, (cx, cy), 5, (0,0,255), -1)
        cv2.putText(roi, f"Obj ({cx},{cy})", (cx+10, cy),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,0), 2)

        # 8. X 方向对准
        diff_x = cx - current_px[0]
        if abs(diff_x) > X_ALIGN_THRESHOLD:
            dx_mm = diff_x * PIXEL_TO_MM
            send_xy(dx_mm, 0)        # 仅移动 X
            current_px[0] = cx       # 更新当前 X 坐标

        # 9. 取物触发
        # 9. 取物触发
        if cy < Y_PICK_THRESHOLD:
            print(f"[INFO] 进入取物线 y={cy}，等待 {WAIT_BEFORE_LOWER}s 后启动抓取")
            time.sleep(WAIT_BEFORE_LOWER)

            arduino.write(b"a\n")  # 吸气泵开启
            print("[ACTION] 吸气泵已开启")

            arduino.write(b"l\n")  # Z 轴下降
            print("[ACTION] Z 轴下降中...")
            time.sleep(1.0)  # 等待下降完成

            arduino.write(b"r\n")  # Z 轴上升
            print("[ACTION] Z 轴上升中...")
            time.sleep(1.0)  # 等待上升完成

            # ===== 新增：返回原位置 =====
            return_dx = (PIXEL_ORIGIN[0] - current_px[0]) * PIXEL_TO_MM
            send_xy(return_dx, 0)
            print("[INFO] 机械臂已返回初始位置")

            arduino.write(b"b\n")  # 吸气泵关闭（释放）
            print("[ACTION] 吸气泵已关闭")

            break  # 本示例只抓一次

        # 显示调试窗口
        cv2.imshow("Mask", mask)
        cv2.imshow("ROI",  roi)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    cap.release()
    arduino.close()
    cv2.destroyAllWindows()
    print("[INFO] 资源已释放，程序结束。")
