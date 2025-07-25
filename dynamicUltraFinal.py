#!/usr/bin/env python3
import cv2
import numpy as np
import serial
import time

# ===== 可调参数 =====
TARGET_COUNT = 3  # 车道数量（即目标物体所在的道数）
# 摄像头 ROI 区域 (根据相机视野调整)
X_START, Y_START = 170, 0
X_END,   Y_END   = 510, 545

# 目标物体的 HSV 颜色范围（根据实际颜色调整）
LOWER_HSV = np.array([100, 100, 80])
UPPER_HSV = np.array([125, 255, 255])

# 轮廓面积阈值：小于该面积的轮廓视为噪声
MIN_AREA = 300

# 去噪参数
BLUR_KERNEL_SIZE = 5                   # 中值滤波核大小
MORPH_KERNEL = np.ones((3, 3), np.uint8)  # 形态学操作核
DILATE_ITER = 1                        # 膨胀迭代次数
ERODE_ITER  = 1                        # 腐蚀迭代次数

# 运动控制阈值
X_ALIGN_THRESHOLD = 3      # 当 |cx - current_x| 超过该值（像素）时，机械臂需要水平移动校正
Y_PICK_THRESHOLD  = 70     # 当物体纵坐标 cy 小于该值（像素）时触发抓取动作
WAIT_BEFORE_LOWER = 0.4    # 触发抓取后等待的时间（秒），再执行下降动作

# 像素坐标 ↔ 机械臂坐标 校准参数
PIXEL_ORIGIN   = np.array([80, 0])  # 机械臂 (0,0) 对应的 ROI 像素坐标 (x, y)
PIXEL_TO_MM    = 1.0                # 像素到毫米的比例（根据标定调整，目前假设 1 像素 ≈ 1 毫米）
X_STEPS_PER_MM = -49.75             # X 轴每毫米对应的步进数（负号方向视机械臂安装方向而定）
Y_STEPS_PER_MM = 49.75              # Y 轴每毫米对应的步进数（预留，当前未使用）

# 串口配置（根据实际端口修改）
SERIAL_PORT = 'COM3'
BAUD_RATE   = 115200

# ===== 初始化摄像头和串口 =====
cap = cv2.VideoCapture(0)
time.sleep(1)  # 等摄像头曝光稳定
if not cap.isOpened():
    raise RuntimeError("无法打开摄像头")
arduino = serial.Serial(port=SERIAL_PORT, baudrate=BAUD_RATE, timeout=1)
time.sleep(2)  # 等待串口初始化完成

def send_xy(dx_mm: float, dy_mm: float):
    """发送 XY 平面相对位移指令到 Arduino"""
    dx_steps = int(round(dx_mm * X_STEPS_PER_MM))
    dy_steps = int(round(dy_mm * Y_STEPS_PER_MM))
    cmd = f"x{dx_steps}y{dy_steps}\n"
    arduino.write(cmd.encode())
    print(f"[Serial] 已发送: {cmd.strip()}")

# ──────────── Step 1: 单帧检测及移动方案规划 ────────────
ret, frame = cap.read()
if not ret:
    print("摄像头读取首帧失败，程序退出。")
    cap.release()
    arduino.close()
    exit(1)

# 提取 ROI 并转换为 HSV 颜色空间
roi = frame[Y_START:Y_END, X_START:X_END]
hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

# 颜色阈值分割初始掩膜
mask = cv2.inRange(hsv, LOWER_HSV, UPPER_HSV)
# 去除高亮反光区域影响（低饱和度 & 高亮度区域设为0）
reflection_mask = cv2.inRange(hsv, np.array([0, 0, 230]), np.array([180, 50, 255]))
mask[reflection_mask > 0] = 0

# 中值滤波去噪 + 膨胀 & 腐蚀
mask = cv2.medianBlur(mask, BLUR_KERNEL_SIZE)
mask = cv2.dilate(mask, MORPH_KERNEL, iterations=DILATE_ITER)
mask = cv2.erode(mask, MORPH_KERNEL, iterations=ERODE_ITER)

# 查找轮廓
contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# 按车道分类轮廓质心坐标
lane_objects = [[] for _ in range(TARGET_COUNT)]
if contours:
    # 按面积过滤并排序，取面积最大的 TARGET_COUNT 个轮廓
    contour_areas = [(cv2.contourArea(c), c) for c in contours if cv2.contourArea(c) >= MIN_AREA]
    contour_areas.sort(key=lambda x: x[0], reverse=True)
    top_contours = contour_areas[:TARGET_COUNT]

    roi_width = roi.shape[1]
    zone_width = roi_width / TARGET_COUNT
    for area, c in top_contours:
        M = cv2.moments(c)
        if M['m00'] == 0:
            continue
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        # 计算所在车道索引
        zone_id = int(cx // zone_width)
        if zone_id < 0:
            zone_id = 0
        elif zone_id >= TARGET_COUNT:
            zone_id = TARGET_COUNT - 1
        lane_objects[zone_id].append((cx, cy))

# 寻找空车道并确定要移动的物体及目标位置
shift_plan = None  # (donor_lane, (obj_x, obj_y), target_lane, (target_center_x, target_center_y))
if contours:
    for lane_idx in range(TARGET_COUNT):
        if len(lane_objects[lane_idx]) == 0:
            # 找到空车道，选取相邻有多余物体的车道作为捐赠源
            donor = None
            offset = 1
            while offset < TARGET_COUNT:
                left = lane_idx - offset
                right = lane_idx + offset
                if left >= 0 and len(lane_objects[left]) > 1:
                    donor = left
                    break
                if right < TARGET_COUNT and len(lane_objects[right]) > 1:
                    donor = right
                    break
                offset += 1
            if donor is not None:
                # 从捐赠车道弹出一个物体（要移动的物体）
                obj_to_move = lane_objects[donor].pop()  # (obj_x, obj_y)
                # 计算目标车道中心点像素坐标（用于参考）
                target_center_x = int((lane_idx + 0.5) * (roi.shape[1] / TARGET_COUNT))
                target_center_y = int(roi.shape[0] / 2)
                shift_plan = (donor, obj_to_move, lane_idx, (target_center_x, target_center_y))
                break

# 输出规划结果
if shift_plan:
    donor_lane, (obj_x, obj_y), target_lane, (tx, ty) = shift_plan
    print(f"从道 {donor_lane+1} 平移物体 ({obj_x},{obj_y}) 到道 {target_lane+1} 的中点 ({tx},{ty})")
else:
    print("所有车道均已满足或无可移动物体，无需平移。")
    cap.release()
    arduino.close()
    exit(0)

# ──────────── Step 2: 视频追踪抓取并搬运指定物体 ────────────
pick_lane_index = donor_lane      # 将从该车道取走物体
drop_lane_index = target_lane     # 物体将被放置到该空车道
roi_width = roi.shape[1]
zone_width = roi_width / TARGET_COUNT
drop_x_center = int((drop_lane_index + 0.5) * zone_width)  # 目标车道中心 X 像素位置

# 当前机械臂像素位置（初始化为校准原点像素坐标）
current_px = PIXEL_ORIGIN.copy()
# 初始对准：若要抓取物体不在当前机械臂正下方，则先水平移动对准该物体所在车道
initial_dx_mm = (obj_x - PIXEL_ORIGIN[0]) * PIXEL_TO_MM
if abs(obj_x - PIXEL_ORIGIN[0]) > X_ALIGN_THRESHOLD:
    send_xy(initial_dx_mm, 0)
    current_px[0] = obj_x  # 机械臂现已对准到物体所在车道的 X 位置

print("[INFO] 开始追踪目标物体，按 q 键退出。")
try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("摄像头画面获取失败，追踪停止。")
            break

        roi = frame[Y_START:Y_END, X_START:X_END]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, LOWER_HSV, UPPER_HSV)
        # 可选：去除高亮反光干扰
        # reflection_mask = cv2.inRange(hsv, np.array([0,0,230]), np.array([180,50,255]))
        # mask[reflection_mask > 0] = 0

        # 中值滤波和形态学去噪
        mask = cv2.medianBlur(mask, BLUR_KERNEL_SIZE)
        mask = cv2.dilate(mask, MORPH_KERNEL, iterations=DILATE_ITER)
        mask = cv2.erode(mask, MORPH_KERNEL, iterations=ERODE_ITER)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            # 没检测到任何物体，显示当前画面并继续
            cv2.imshow("Mask", mask)
            cv2.imshow("ROI", roi)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            continue

        # 在当前帧中寻找目标车道的物体轮廓
        target_contour = None
        target_cx = target_cy = None
        for c in contours:
            area = cv2.contourArea(c)
            if area < MIN_AREA:
                continue
            M = cv2.moments(c)
            if M["m00"] == 0:
                continue
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            # 判断该轮廓所在车道
            zone_id = int(cx // zone_width)
            if zone_id < 0:
                zone_id = 0
            elif zone_id >= TARGET_COUNT:
                zone_id = TARGET_COUNT - 1
            if zone_id == pick_lane_index:
                # 选取该车道中最接近取物线的物体作为目标（cy 越小离顶部越近）
                if target_contour is None or cy < target_cy:
                    target_contour = c
                    target_cx = cx
                    target_cy = cy

        if target_contour is None:
            print("[WARN] 未找到目标物体，终止追踪。")
            break

        # 调试显示：绘制目标轮廓和质心位置
        cv2.drawContours(roi, [target_contour], -1, (0, 255, 0), 2)
        cv2.circle(roi, (target_cx, target_cy), 5, (0, 0, 255), -1)
        cv2.putText(roi, f"Obj ({target_cx},{target_cy})", (target_cx + 10, target_cy),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

        # X 方向校正移动：若物体 X 与当前机械臂 X 对准像素有偏差，则水平移动机械臂跟踪
        diff_x = target_cx - current_px[0]
        if abs(diff_x) > X_ALIGN_THRESHOLD:
            dx_mm = diff_x * PIXEL_TO_MM
            send_xy(dx_mm, 0)
            current_px[0] = target_cx  # 更新当前机械臂所在 X 像素位置

        # 检查是否达到抓取触发线
        if target_cy is not None and target_cy < Y_PICK_THRESHOLD:
            print(f"[INFO] 物体接近取物线 (y={target_cy})，等待 {WAIT_BEFORE_LOWER}s 后执行抓取")
            time.sleep(WAIT_BEFORE_LOWER)
            # 吸真空开启（抓取）
            arduino.write(b"a\n")
            print("[ACTION] 真空阀已打开（开始吸取）")
            # Z 轴下降执行抓取
            arduino.write(b"l\n")
            print("[ACTION] Z 轴下降抓取...")
            time.sleep(1.0)  # 等待机械臂下降完成
            # Z 轴上升，提起物体
            arduino.write(b"r\n")
            print("[ACTION] Z 轴上升，物体已抓起")
            time.sleep(1.0)  # 等待机械臂上升完成

            # 水平移动机械臂到目标空车道的正上方
            dx_to_target_mm = (drop_x_center - current_px[0]) * PIXEL_TO_MM
            send_xy(dx_to_target_mm, 0)
            current_px[0] = drop_x_center
            print(f"[INFO] 正将物体移至第 {drop_lane_index+1} 道上方")

            # 下降放置物体
            arduino.write(b"l\n")
            print("[ACTION] Z 轴下降，放置物体...")
            time.sleep(1.0)  # 等待机械臂下降到位
            # 关闭真空（释放物体）
            arduino.write(b"b\n")
            print("[ACTION] 真空阀已关闭（物体释放）")
            time.sleep(0.5)  # 确保物体已掉落
            # 提升机械臂
            arduino.write(b"r\n")
            print("[ACTION] Z 轴上升，机械臂回到上方")
            time.sleep(1.0)

            print("[INFO] 物体已转移至空车道，操作完成。")
            break

        # 显示调试窗口
        cv2.imshow("Mask", mask)
        cv2.imshow("ROI", roi)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    # 释放资源
    cap.release()
    arduino.close()
    cv2.destroyAllWindows()
