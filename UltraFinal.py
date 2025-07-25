# UltraFinal.py  —  多目标车道均衡 & 机械臂抓取/放置
# ------------------------------------------------------------
# * 仅注释掉：反光剔除 & 形态学腐蚀
# * 其余算法保持不变
# ------------------------------------------------------------

import cv2
import numpy as np
import serial
import time
from typing import List, Tuple

# ───────────────── 参数设置 ─────────────────
TARGET_COUNT = 4            # 车道/目标总数
FRAME_WAIT   = 5            # XY 移动后等待时间 (秒)

# ROI 裁剪区域
X_START, X_END = 170, 510
Y_START, Y_END = 0, 545

# 颜色阈值 (粉色肉块)
LOWER_HSV = np.array([ 0,  45, 135])
UPPER_HSV = np.array([11, 120, 255])

# 反光阈值 (仍保留参数, 但后面已注释不再使用)
REFLECTION_LOWER = np.array([0, 0, 230])
REFLECTION_UPPER = np.array([180, 50, 255])

# 像素↔毫米标定参数
PIXEL_ORIGIN = np.array([187, 38])  # 机械臂在 ROI 内的像素原点
PIXEL_TO_MM  = 1.0                  # 1 像素 ≈ 1 mm (示例)

# 步进电机 mm→步 换算
X_STEPS_PER_MM = 49.75
Y_STEPS_PER_MM = 49.75

# 串口&摄像头初始化
arduino = serial.Serial(port='COM3', baudrate=115200, timeout=1)
time.sleep(2)
cap = cv2.VideoCapture(0)
time.sleep(1)

# ────────── 工具函数 ──────────
def send_xy(dx_mm: float, dy_mm: float):
    """发送 XY 移动指令 (dx,dy 单位:mm)"""
    dx_steps = int(round(dx_mm * X_STEPS_PER_MM))
    dy_steps = int(round(dy_mm * Y_STEPS_PER_MM))
    cmd = f"x{dx_steps}y{dy_steps}\n"
    arduino.write(cmd.encode())
    print("→ 发送XY移动指令:", cmd.strip())

# ────────── 主流程 ──────────
try:
    # 1) 获取一帧并裁剪 ROI
    ret, frame = cap.read()
    if not ret:
        raise RuntimeError("摄像头读取失败")
    roi = frame[Y_START:Y_END, X_START:X_END]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    # 2) 基于颜色阈值提取掩膜
    mask = cv2.inRange(hsv, LOWER_HSV, UPPER_HSV)

    # --- 以下是反光剔除的代码, 已注释 ---
    # reflection_mask = cv2.inRange(hsv, REFLECTION_LOWER, REFLECTION_UPPER)
    # kernel = np.ones((3, 3), np.uint8)
    # reflection_mask = cv2.dilate(reflection_mask, kernel, iterations=1)
    # mask[reflection_mask > 0] = 0  # 将反光区域剔除

    # 3) 中值滤波 & 膨胀
    mask = cv2.medianBlur(mask, 5)
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations=1)

    # --- 以下是腐蚀操作, 已注释 ---
    # mask = cv2.erode(mask, kernel, iterations=1)

    # 4) 轮廓提取 & 面积筛选
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = [c for c in contours if cv2.contourArea(c) >= 300]
    if not contours:
        raise RuntimeError("未检测到目标")

    # 取面积最大的 TARGET_COUNT 个目标
    contour_areas = sorted([(cv2.contourArea(c), c) for c in contours], key=lambda x: x[0], reverse=True)
    top_contours = contour_areas[:TARGET_COUNT]

    # 5) 按水平位置分配到车道
    lane_objects: List[List[Tuple[int, int]]] = [[] for _ in range(TARGET_COUNT)]
    zone_width = roi.shape[1] / TARGET_COUNT
    for area, c in top_contours:
        M = cv2.moments(c)
        if M['m00'] == 0:
            continue
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        zone_id = min(int(cx // zone_width), TARGET_COUNT - 1)
        lane_objects[zone_id].append((cx, cy))

    # 6) 生成均衡移动计划
    shift_plans = []  # (donor_lane, (ox,oy), target_lane, (tx,ty))
    for lane_idx in range(TARGET_COUNT):
        if len(lane_objects[lane_idx]) == 0:
            donor = None
            offset = 1
            while offset < TARGET_COUNT:
                left  = lane_idx - offset
                right = lane_idx + offset
                if left >= 0 and len(lane_objects[left]) > 1:
                    donor = left; break
                if right < TARGET_COUNT and len(lane_objects[right]) > 1:
                    donor = right; break
                offset += 1
            if donor is not None and lane_objects[donor]:
                ox, oy = lane_objects[donor].pop()
                target_x = int((lane_idx + 0.5) * zone_width)
                target_y = int(roi.shape[0] / 2)
                shift_plans.append((donor, (ox, oy), lane_idx, (target_x, target_y)))

    if not shift_plans:
        print("所有车道已平衡，无需移动。")
        exit()

    # 7) 执行移动方案
    current_px = PIXEL_ORIGIN.copy()
    current_mm = np.array([0.0, 0.0])

    for donor_lane, (ox, oy), target_lane, (tx, ty) in shift_plans:
        # → 移至物体
        delta_px = current_px - np.array([ox, oy])
        delta_mm = delta_px * PIXEL_TO_MM
        print(f"\n移动到物体: Δpx {delta_px} → {delta_mm} mm")
        send_xy(delta_mm[0], delta_mm[1])
        time.sleep(FRAME_WAIT)

        # Z 下降 + 吸附
        print("抓取...")
        arduino.write(b"l\n");     time.sleep(1)
        arduino.write(b"a\n");     time.sleep(0.2)
        arduino.write(b"r\n");     time.sleep(1)
        current_px = np.array([ox, oy])
        current_mm += delta_mm

        # → 移至目标
        delta_px = current_px - np.array([tx, ty])
        delta_mm = delta_px * PIXEL_TO_MM
        print(f"移动到目标: Δpx {delta_px} → {delta_mm} mm")
        send_xy(delta_mm[0], delta_mm[1])
        time.sleep(FRAME_WAIT)

        # Z 下降 + 释放
        print("放置...")
        arduino.write(b"l\n");     time.sleep(1)
        arduino.write(b"b\n");     time.sleep(0.2)
        arduino.write(b"r\n");     time.sleep(1)
        current_px = np.array([tx, ty])
        current_mm += delta_mm

    # 8) 返回原点 (可注释)
    send_xy(-current_mm[0], -current_mm[1])
    time.sleep(FRAME_WAIT)

finally:
    cap.release()
    arduino.close()
    print("\n资源已释放，程序结束。")
