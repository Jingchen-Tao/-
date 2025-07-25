import cv2
import numpy as np
import time

# ====== ROI裁剪区域 ======
x_start, x_end = 209, 538
y_start, y_end = 0, 545

# ====== 主目标颜色范围（放宽V上限）======
lower_hsv = np.array([100, 100, 80])
upper_hsv = np.array([125, 255, 255])

# ====== 反光区域特征（高亮 + 低饱和）======
reflection_lower = np.array([0, 0, 230])
reflection_upper = np.array([180, 50, 255])

# ====== 坐标参数 ======
pixel_origin_x, pixel_origin_y = 285, 25
pixel_to_mm_ratio = 50 / 285  # ≈0.1754 mm/px

# ====== 车道/目标数量 ======
target_count = 5
# ──────────── 1. 取单帧 ────────────
cap = cv2.VideoCapture(0)
time.sleep(1)                       # 给摄像头一点曝光时间
ret, frame = cap.read()             # 只抓一帧
cap.release()

if not ret:
    print("摄像头读取失败，程序退出。")
    exit(1)

# ──────────── 2. ROI检测 ────────────
roi = frame[y_start:y_end, x_start:x_end]
hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

mask = cv2.inRange(hsv, lower_hsv, upper_hsv)

reflection_mask = cv2.inRange(hsv, reflection_lower, reflection_upper)
kernel = np.ones((3, 3), np.uint8)
reflection_mask = cv2.dilate(reflection_mask, kernel, iterations=1)
mask[reflection_mask > 0] = 0

mask = cv2.medianBlur(mask, 5)
mask = cv2.dilate(mask, kernel, iterations=1)
mask = cv2.erode(mask, kernel, iterations=1)

contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# ──────────── 3. 车道归类 ────────────
lane_objects = [[] for _ in range(target_count)]

if contours:
    contour_areas = [(cv2.contourArea(c), c) for c in contours if cv2.contourArea(c) >= 300]
    contour_areas.sort(key=lambda x: x[0], reverse=True)      # 只按面积排，避免 ndarray 比较
    top_contours = contour_areas[:target_count]

    roi_width = roi.shape[1]
    zone_width = roi_width / target_count

    for area, c in top_contours:
        M = cv2.moments(c)
        if M['m00'] == 0:
            continue
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        zone_id = min(int(cx // zone_width), target_count - 1)
        lane_objects[zone_id].append((cx, cy))

# ──────────── 4. 平移规划 ────────────
def lane_midpoint(idx):
    x_mid = int((idx + 0.5) * (roi.shape[1] / target_count))
    y_mid = int(roi.shape[0] / 2)
    return (x_mid, y_mid)

shift_plans = []
for lane_idx in range(target_count):
    if len(lane_objects[lane_idx]) == 0:                 # 该车道空
        donor = None
        offset = 1
        while offset < target_count:
            left = lane_idx - offset
            right = lane_idx + offset
            if left >= 0 and len(lane_objects[left]) > 1:
                donor = left; break
            if right < target_count and len(lane_objects[right]) > 1:
                donor = right; break
            offset += 1
        if donor is not None:
            moved = lane_objects[donor].pop()            # 弹出一个
            target_px = lane_midpoint(lane_idx)
            shift_plans.append((donor, moved, lane_idx, target_px))

# ──────────── 5. 打印结果 ────────────
if shift_plans:
    for d,(ox,oy),t,(tx,ty) in shift_plans:
        print(f"从道 {d+1} 平移物体 ({ox},{oy}) 到道 {t+1} 的中点 ({tx},{ty})")
else:
    print("所有车道已满足，无需平移。")

# （可选）在窗口里查看带标注的 ROI
# —— 把目标、分区线、车道中点等画上去再显示 ——
for z in range(1, target_count):            # 画分区线
    x_line = int(z * (roi.shape[1] / target_count))
    cv2.line(roi, (x_line, 0), (x_line, roi.shape[0]), (128,128,128), 1)

for lane_idx, pts in enumerate(lane_objects):
    for (cx, cy) in pts:
        cv2.circle(roi, (cx, cy), 5, (0,255,0), -1)
    # 画每条道的中点（可视化）
    mx, my = lane_midpoint(lane_idx)
    cv2.circle(roi, (mx, my), 3, (0,255,255), -1)

cv2.imshow("Single-Frame ROI", roi)
cv2.waitKey(0)        # 按任意键关闭
cv2.destroyAllWindows()
