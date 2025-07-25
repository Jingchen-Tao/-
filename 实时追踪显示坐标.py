#!/usr/bin/env python3
"""
detect_multi_green_tracked_speed_smoothed.py  —— 颜色 + 面积筛选 + 车道归类 + ID 追踪 + 平滑的 Y 方向速度
-----------------------------------------------------------------------------------
功能：
  1. ROI 裁剪后按 HSV 阈值分割；
  2. 中值滤波 + 膨胀；面积 ≥ MIN_AREA 的轮廓全部保留；
  3. 给每个目标分配唯一 ID，跨帧追踪；
  4. 判断所属车道（lane_id），在画面上显示：
        • 绿色轮廓
        • 中心绿点
        • 文本：ID | (cx,cy) | lane | avg_vY:px/s
  5. 按 q 退出；按 s 保存截图到当前目录。

说明：
  * 追踪逻辑采用简单质心（Centroid）匹配。
  * 速度计算：使用滑动窗口对最近 N 帧速度做平均，得到更平滑的 vY。
"""

import cv2
import numpy as np
import time
from pathlib import Path
from collections import OrderedDict, deque
from math import hypot

# ────────────── 可调参数 ──────────────
ROI = (170, 0, 510, 545)
LOWER_HSV = np.array([100, 100, 80])
UPPER_HSV = np.array([125, 255, 255])

MIN_AREA        = 300
BLUR_KSIZE      = 5
DILATE_ITER     = 1
KERNEL          = np.ones((3, 3), np.uint8)
DOT_RADIUS      = 5

NUM_LANES       = 3
DIST_THRESH     = 40
MAX_DISAPPEARED = 15
SPEED_WINDOW    = 5    # 速度平滑窗口大小

class CentroidTracker:
    def __init__(self, dist_threshold=DIST_THRESH, max_disappeared=MAX_DISAPPEARED, win=SPEED_WINDOW):
        self.next_id = 1
        self.objects = OrderedDict()       # id -> (cx, cy)
        self.disappeared = {}
        self.last_positions = {}           # id -> (cx, cy, ts)
        self.speed_hist = {}               # id -> deque of recent vy
        self.avg_speeds = {}               # id -> average vy
        self.dist_thresh = dist_threshold
        self.max_disappeared = max_disappeared
        self.win = win

    def register(self, centroid):
        oid = self.next_id
        now = time.time()
        self.objects[oid] = centroid
        self.disappeared[oid] = 0
        self.last_positions[oid] = (centroid[0], centroid[1], now)
        self.speed_hist[oid] = deque([0.0], maxlen=self.win)
        self.avg_speeds[oid] = 0.0
        self.next_id += 1

    def deregister(self, oid):
        for d in (self.objects, self.disappeared, self.last_positions, self.speed_hist, self.avg_speeds):
            d.pop(oid, None)

    def update(self, detections):
        now = time.time()
        # no detections
        if not detections:
            for oid in list(self.disappeared.keys()):
                self.disappeared[oid] += 1
                if self.disappeared[oid] > self.max_disappeared:
                    self.deregister(oid)
            return self.objects
        # first frame
        if not self.objects:
            for c in detections:
                self.register(c)
            return self.objects

        # distance matrix
        ids = list(self.objects.keys())
        centroids = list(self.objects.values())
        D = np.zeros((len(centroids), len(detections)), dtype=float)
        for i,(ox,oy) in enumerate(centroids):
            for j,(dx,dy) in enumerate(detections):
                D[i,j] = hypot(ox-dx, oy-dy)

        rows = D.min(axis=1).argsort()
        cols = D.argmin(axis=1)[rows]
        used_r, used_c = set(), set()
        for r,c in zip(rows, cols):
            if r in used_r or c in used_c or D[r,c] > self.dist_thresh:
                continue
            oid = ids[r]
            px, py, pts = self.last_positions[oid]
            nx, ny = detections[c]
            dt = now - pts if pts else 1e-6
            vy = (ny - py) / dt
            self.speed_hist[oid].append(vy)
            self.avg_speeds[oid] = sum(self.speed_hist[oid]) / len(self.speed_hist[oid])
            self.objects[oid] = (nx, ny)
            self.last_positions[oid] = (nx, ny, now)
            self.disappeared[oid] = 0
            used_r.add(r); used_c.add(c)

        # unmatched old
        for r in set(range(len(centroids))) - used_r:
            oid = ids[r]
            self.disappeared[oid] += 1
            if self.disappeared[oid] > self.max_disappeared:
                self.deregister(oid)
        # unmatched new
        for c in set(range(len(detections))) - used_c:
            self.register(detections[c])

        return self.objects

if __name__ == '__main__':
    cap = cv2.VideoCapture(0)
    time.sleep(1)
    if not cap.isOpened():
        raise RuntimeError("无法打开摄像头")
    tracker = CentroidTracker()
    print("[INFO] 按 q 退出，按 s 保存截图")
    while True:
        ret, frame = cap.read()
        if not ret: break
        x0,y0,x1,y1 = ROI
        roi = frame[y0:y1, x0:x1]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, LOWER_HSV, UPPER_HSV)
        mask = cv2.medianBlur(mask, BLUR_KSIZE)
        mask = cv2.dilate(mask, KERNEL, iterations=DILATE_ITER)
        cnts,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        dets,keep = [],[]
        for c in cnts:
            a = cv2.contourArea(c)
            if a < MIN_AREA: continue
            M = cv2.moments(c)
            if M['m00']==0: continue
            cx,cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
            dets.append((cx,cy)); keep.append(c)

        objs = tracker.update(dets)
        vis = roi.copy()
        lw = vis.shape[1]/NUM_LANES
        for i in range(1,NUM_LANES):
            x = int(i*lw)
            cv2.line(vis,(x,0),(x,vis.shape[0]),(128,128,128),1)
        for c in keep:
            cv2.drawContours(vis,[c],-1,(0,255,0),2)
        for oid,(cx,cy) in objs.items():
            lane = min(int(cx//lw)+1,NUM_LANES)
            vy = tracker.avg_speeds.get(oid,0.0)
            cv2.circle(vis,(cx,cy),DOT_RADIUS,(0,255,0),-1)
            txt = f"ID:{oid} ({cx},{cy}) L{lane} vY:{vy:.1f}px/s"
            cv2.putText(vis,txt,(cx+6,cy-6),cv2.FONT_HERSHEY_SIMPLEX,0.45,(0,255,255),1)
        cv2.imshow("ROI detection",vis)
        cv2.imshow("Mask",mask)
        print(f"\rTracking {len(objs)} objs",end="")
        k = cv2.waitKey(1)&0xFF
        if k==ord('q'): break
        if k==ord('s'):
            ts=time.strftime("%Y%m%d_%H%M%S")
            out=Path(f"detect_{ts}.png")
            cv2.imwrite(str(out),vis)
            print(f"\n[INFO] Saved {out}")
    cap.release()
    cv2.destroyAllWindows()
    print("\n[INFO] Done.")
