
"""" 规划移动方案再打印的程序"""

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
target_count = 3

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


"""

#include <AccelStepper.h>
#include <Stepper.h>

// 插补步长配置
const float steps_per_mm = 49.75;

// Z轴配置
const int Z_STEPS_PER_REV = 1000;  // 可调
const int Z_PIN1 = 38;
const int Z_PIN2 = 40;

// 步进电机对象
AccelStepper stepperX(AccelStepper::DRIVER, 30, 31);
AccelStepper stepperY(AccelStepper::DRIVER, 39, 41);
Stepper stepperZ(Z_STEPS_PER_REV, Z_PIN1, Z_PIN2);  // Z轴使用普通 Stepper 控制

void setup() {
  Serial.begin(115200);
  Serial.println("Ready for Industrial Insert Control with Z");

  stepperX.setMaxSpeed(10000);
  stepperY.setMaxSpeed(10000);
  stepperX.setAcceleration(700);
  stepperY.setAcceleration(20000);

  stepperZ.setSpeed(100000);  // Z 轴速度可调
}

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    // 插补命令格式
    if (command.startsWith("x")) {
      long x_steps = 0, y_steps = 0;
      int result = sscanf(command.c_str(), "x%ldy%ld", &x_steps, &y_steps);
      if (result == 2) {
        Serial.print("Moving X: "); Serial.print(x_steps);
        Serial.print(" steps, Y: "); Serial.println(y_steps);

        stepperX.move(x_steps);
        stepperY.move(y_steps);
        while (stepperX.distanceToGo() != 0 || stepperY.distanceToGo() != 0) {
          stepperX.run();
          stepperY.run();
        }
        Serial.println("XY Move done.");
      } else {
        Serial.println("Invalid format for XY.");
      }

    // Z轴下降命令（'l'）
    } else if (command == "l") {
      Serial.println("Z轴下降");
      stepperZ.step(-1000);  // 你可以根据实际情况调整步数
    }

    // Z轴上升命令（'r'）
    else if (command == "r") {
      Serial.println("Z轴上升");
      stepperZ.step(1000);
    }

    else {
      Serial.println("Unknown command.");
    }
  }
}

"""