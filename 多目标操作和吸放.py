import cv2
import numpy as np
import serial
import time

"""
Multi-target pick-and-place demo (v3)
-------------------------------------
• Detect up to TARGET_COUNT blobs, sort by ascending y-coordinate.
• Move the arm sequentially; the current blob becomes the new pixel origin.
• Z-axis pick (l) + suction ON (a) → place (r) + suction OFF (b).
• Optional return-to-origin after the last target.
"""

# ────────────── USER-TUNABLE SETTINGS ──────────────
TARGET_COUNT = 4
AUTO_RETURN_HOME = True
FRAME_WAIT = 10  # seconds to wait after each XY move

# ROI
X_START, X_END = 170, 510
Y_START, Y_END = 0, 545

# HSV (green sample)
LOWER_HSV = np.array([100, 100, 80])
UPPER_HSV = np.array([125, 255, 255])

# Reflection mask
REFLECTION_LOWER = np.array([0, 0, 230])
REFLECTION_UPPER = np.array([180, 50, 255])

# Calibration
PIXEL_ORIGIN = np.array([195, 82])  # current arm tip in pixels
PIXEL_TO_MM = 1.1  # mm per pixel

# Steps ↔ mm
X_STEPS_PER_MM = 50
Y_STEPS_PER_MM = 50
Z_STEPS = 1000  # every Z move

# ────────────── SERIAL & CAMERA ──────────────
arduino = serial.Serial(port='COM3', baudrate=115200, timeout=1)
time.sleep(2)
cap = cv2.VideoCapture(0)
time.sleep(1)


# ────────────── HELPERS ──────────────
def send_xy(dx_mm: float, dy_mm: float):
    dx_steps = int(round(dx_mm * X_STEPS_PER_MM))
    dy_steps = int(round(dy_mm * Y_STEPS_PER_MM))
    arduino.write(f"x{dx_steps}y{dy_steps}\n".encode())
    print("→", f"x{dx_steps}y{dy_steps}")


def z_pick_place():
    """Z-down → suction ON → Z-up → suction OFF"""
    arduino.write(b"l\n")  # Z-down
    time.sleep(1)  # wait motor

    arduino.write(b"a\n")  # ### NEW 打开吸气阀
    time.sleep(0.2)  # 小延时，真空建立

    arduino.write(b"r\n")  # Z-up
    time.sleep(1)

    arduino.write(b"b\n")  # ### NEW 关闭吸气阀
    time.sleep(0.1)


# ────────────── MAIN ──────────────
try:
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


    def cy(c):
        M = cv2.moments(c)
        return M['m01'] / M['m00'] if M['m00'] else 1e9


    targets = sorted(contours, key=cy)[:TARGET_COUNT]

    current_px = PIXEL_ORIGIN.copy()
    current_mm = np.array([0.0, 0.0])

    for idx, cnt in enumerate(targets):
        M = cv2.moments(cnt)
        if M['m00'] == 0:
            print(f"目标 {idx} 质心失败，跳过。")
            continue
        cx, cy = int(M['m10'] / M['m00']), int(M['m01'] / M['m00'])

        delta_px = current_px - np.array([cx, cy])  # 起点-终点
        delta_mm = delta_px * PIXEL_TO_MM

        print(f"[目标{idx}] 像素差 {delta_px} → {delta_mm} mm")
        send_xy(delta_mm[0], delta_mm[1])
        time.sleep(FRAME_WAIT)

        z_pick_place()  # ### NEW

        current_px = np.array([cx, cy])
        current_mm += delta_mm

    if AUTO_RETURN_HOME and (abs(current_mm) > 0.1).any():
        print("返回原点…")
        send_xy(-current_mm[0], -current_mm[1])
        time.sleep(FRAME_WAIT)

finally:
    cap.release()
    arduino.close()
    print("资源已释放，程序结束。")


"""
#include <AccelStepper.h>
#include <Stepper.h>

// XY 步进电机
AccelStepper stepperX(AccelStepper::DRIVER, 30, 31);
AccelStepper stepperY(AccelStepper::DRIVER, 39, 41);

// Z 轴（普通 Stepper 控制）
const int Z_STEPS_PER_REV = 1000;
const int Z_PIN1 = 38;
const int Z_PIN2 = 40;
Stepper stepperZ(Z_STEPS_PER_REV, Z_PIN1, Z_PIN2);

// 真空阀（数字 28）
const int VAC_PIN = 28;          // ### NEW

void setup() {
  Serial.begin(115200);
  Serial.println("Ready for XY + Z + Vacuum");

  stepperX.setMaxSpeed(10000);
  stepperY.setMaxSpeed(10000);
  stepperX.setAcceleration(700);
  stepperY.setAcceleration(20000);

  stepperZ.setSpeed(800);

  pinMode(VAC_PIN, OUTPUT);      // ### NEW
  digitalWrite(VAC_PIN, LOW);    // 关阀
}

void loop() {
  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();

  if (cmd.startsWith("x")) {                       // XY 插补
    long xs = 0, ys = 0;
    if (sscanf(cmd.c_str(), "x%ldy%ld", &xs, &ys) == 2) {
      stepperX.move(xs);
      stepperY.move(ys);
      while (stepperX.distanceToGo() || stepperY.distanceToGo()) {
        stepperX.run();
        stepperY.run();
      }
    }
  }
  else if (cmd == "l") {                           // Z-down
    stepperZ.step(-2000);
  }
  else if (cmd == "r") {                           // Z-up
    stepperZ.step(2000);
  }
  else if (cmd == "a") {                           // ### NEW 吸气阀 ON
    digitalWrite(VAC_PIN, HIGH);
  }
  else if (cmd == "b") {                           // ### NEW 吸气阀 OFF
    digitalWrite(VAC_PIN, LOW);
  }
}

"""