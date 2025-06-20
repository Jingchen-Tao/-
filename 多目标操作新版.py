import cv2
import numpy as np
import serial
import time

"""
Multi‑target pick‑and‑place demo (v2)
-------------------------------------
• Detect up to TARGET_COUNT blobs, sort by ascending y‑coordinate.
• Move the arm sequentially; the current blob becomes the new pixel origin.
• Z‑axis pick (l) and place (r) at each point.
• **NEW**: optional return‑to‑origin feature after the last target.
"""

# ────────────── USER‑TUNABLE SETTINGS ──────────────
TARGET_COUNT = 7           # 同时处理的目标数量
AUTO_RETURN_HOME = True    # 全部完成后是否自动回到原点
FRAME_WAIT = 10            # 发送 XY 指令后等待时间 (s)

# ROI
X_START, X_END = 209, 538
Y_START, Y_END = 0, 545

# HSV 颜色阈值 (绿色示例)
LOWER_HSV = np.array([100, 100, 80])
UPPER_HSV = np.array([125, 255, 255])

# 反光区域阈值 (高亮 + 低饱和)
REFLECTION_LOWER = np.array([0, 0, 230])
REFLECTION_UPPER = np.array([180, 50, 255])

# 标定参数
PIXEL_ORIGIN = np.array([250, 53])  # 手臂当前像素参考点
PIXEL_TO_MM = 1              # mm/px

# 步进换算
X_STEPS_PER_MM = 50
Y_STEPS_PER_MM = 50
Z_STEPS = 1000                      # 每次 Z 轴动作步数

# ────────────── SERIAL & CAMERA ──────────────
arduino = serial.Serial(port='COM3', baudrate=115200, timeout=1)
time.sleep(2)  # 等待 Arduino 就绪

cap = cv2.VideoCapture(0)
time.sleep(1)  # 等待摄像头

# ────────────── HELPER FUNCTIONS ──────────────
def send_xy(dx_mm: float, dy_mm: float):
    """Convert mm to steps and send x±…y±…\n to Arduino."""
    dx_steps = int(round(dx_mm * X_STEPS_PER_MM))
    dy_steps = int(round(dy_mm * Y_STEPS_PER_MM))
    cmd = f"x{dx_steps}y{dy_steps}\n"
    arduino.write(cmd.encode())
    print("→", cmd.strip())


def z_down_up():
    arduino.write(b"l\n")  # Z 轴下降
    time.sleep(1)
    arduino.write(b"r\n")  # Z 轴抬起
    time.sleep(1)


# ────────────── MAIN ROUTINE ──────────────
try:
    # 只抓取首帧进行目标排序；如果需要实时帧更新，可放 while True
    ret, frame = cap.read()
    if not ret:
        raise RuntimeError("摄像头读取失败")

    roi = frame[Y_START:Y_END, X_START:X_END]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv, LOWER_HSV, UPPER_HSV)
    reflection = cv2.inRange(hsv, REFLECTION_LOWER, REFLECTION_UPPER)
    kernel = np.ones((3, 3), np.uint8)
    mask[cv2.dilate(reflection, kernel, 1) > 0] = 0
    mask = cv2.medianBlur(mask, 5)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = [c for c in contours if cv2.contourArea(c) >= 300]

    if not contours:
        raise RuntimeError("未检测到目标")

    # 按 y 坐标升序排序
    def contour_center_y(c):
        M = cv2.moments(c)
        return M['m01'] / M['m00'] if M['m00'] else 1e9

    targets = sorted(contours, key=contour_center_y)[:TARGET_COUNT]

    current_px = PIXEL_ORIGIN.copy()       # 当前参考像素点 (x, y)
    current_mm = np.array([0.0, 0.0])      # 机械臂实际已移动(mm)

    for idx, cnt in enumerate(targets):
        M = cv2.moments(cnt)
        if M['m00'] == 0:
            print(f"目标 {idx} 质心失败，跳过。")
            continue
        cx, cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])

        # 统一起点‑终点方向: 起点(current_px) − 终点(cx, cy)
        delta_px = current_px - np.array([cx, cy])
        delta_mm = delta_px * PIXEL_TO_MM

        print(f"[目标{idx}] 像素差 {delta_px} → 位移 {delta_mm} mm")
        send_xy(delta_mm[0], delta_mm[1])
        time.sleep(FRAME_WAIT)

        z_down_up()

        current_px = np.array([cx, cy])
        current_mm += delta_mm

    # ────────────── RETURN HOME ──────────────
    if AUTO_RETURN_HOME and (abs(current_mm[0]) > 0.1 or abs(current_mm[1]) > 0.1):
        print("返回原点…")
        send_xy(-current_mm[0], -current_mm[1])
        time.sleep(FRAME_WAIT)

finally:
    cap.release()
    arduino.close()
    print("资源已释放，程序结束。")
