#!/usr/bin/env python3
# detect_multi_green.py  —— 颜色 + 面积筛选目标检测（多目标，绿点标中心）
# ------------------------------------------------------------
# 功能：
#   1. ROI 裁剪后按 HSV 阈值分割；
#   2. 中值滤波 + 膨胀；面积 ≥ MIN_AREA 的轮廓全部保留；
#   3. 在每个目标几何中心绘制绿色实心圆点并标注面积（可选）；
#   4. 按 q 退出；按 s 保存截图到当前目录。
# ------------------------------------------------------------

import cv2
import numpy as np
import time
from pathlib import Path

# ────────────── 可调参数 ──────────────
ROI = (170, 0, 510, 545)                     # (x_start, y_start, x_end, y_end)
LOWER_HSV = np.array([100, 100, 80])
UPPER_HSV = np.array([125, 255, 255])

# LOWER_HSV = np.array([0, 45, 135])           # 鸡肉颜色
# UPPER_HSV = np.array([11, 120, 255])         # 目标颜色上界
MIN_AREA = 300                               # 面积阈值 (px²)
BLUR_KSIZE = 5                               # 中值滤波核尺寸
DILATE_ITER = 1                              # 膨胀次数
KERNEL = np.ones((3, 3), np.uint8)

DOT_RADIUS = 5                               # 绿点半径 (px)

# ────────────── 初始化摄像头 ──────────────
cap = cv2.VideoCapture(0)
time.sleep(1)
if not cap.isOpened():
    raise RuntimeError("无法打开摄像头")

print("[INFO] 按 q 退出，按 s 保存当前检测结果截图")
while True:
    ret, frame = cap.read()
    if not ret:
        print("[ERROR] 摄像头读取失败")
        break

    # 1) ROI 裁剪
    x0, y0, x1, y1 = ROI
    roi = frame[y0:y1, x0:x1]

    # 2) 颜色阈值分割
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, LOWER_HSV, UPPER_HSV)

    # 3) 去噪 + 膨胀
    mask = cv2.medianBlur(mask, BLUR_KSIZE)
    mask = cv2.dilate(mask, KERNEL, iterations=DILATE_ITER)

    # 4) 轮廓提取 + 面积筛选
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    targets = [c for c in contours if cv2.contourArea(c) >= MIN_AREA]

    # 5) 在 ROI 上绘制检测结果
    roi_vis = roi.copy()
    for c in targets:
        area = cv2.contourArea(c)
        M = cv2.moments(c)
        if M["m00"] == 0:
            continue
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])

        # 绘制轮廓
        cv2.drawContours(roi_vis, [c], -1, (0, 255, 0), 2)

        # 在中心绘制绿色圆点
        cv2.circle(roi_vis, (cx, cy), DOT_RADIUS, (0, 255, 0), -1)

        # 面积标签（若不需要可删掉）
        cv2.putText(
            roi_vis, f"{area:.0f}",
            (cx + 6, cy - 6),
            cv2.FONT_HERSHEY_SIMPLEX, 0.4,
            (0, 255, 255), 1
        )

    # 6) 显示窗口
    cv2.imshow("ROI detection", roi_vis)
    cv2.imshow("Mask", mask)

    # 7) 控制台信息 & 按键处理
    print(f"\r检测到目标数: {len(targets):>2}", end="")
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    if key == ord('s'):
        ts = time.strftime("%Y%m%d_%H%M%S")
        out_path = Path(f"detect_{ts}.png")
        cv2.imwrite(str(out_path), roi_vis)
        print(f"\n[INFO] 截图已保存 -> {out_path}")

cap.release()
cv2.destroyAllWindows()
print("\n[INFO] 资源已释放，程序结束")
