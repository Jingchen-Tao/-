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

# HSV检测参数：绿色
# lower_hsv = np.array([17, 0, 0])
# upper_hsv = np.array([90, 255, 255])
lower_hsv = np.array([100, 100, 80])   # H, S, V
upper_hsv = np.array([125, 255, 255])

# 标定参数
pixel_origin_x = 235
pixel_origin_y = 62
pixel_to_mm_ratio = 1  # 暂时估算


# 步进换算系数
x_steps_per_mm = 50
y_steps_per_mm = 50

# 当前坐标记录
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

        delta_x_pixel = pixel_origin_x - cx_roi
        delta_y_pixel = pixel_origin_y - cy_roi

        delta_x_mm = delta_x_pixel * pixel_to_mm_ratio
        delta_y_mm = delta_y_pixel * pixel_to_mm_ratio

        print(f"X偏移: {delta_x_mm:.2f} mm")
        print(f"Y偏移: {delta_y_mm:.2f} mm")

        move_x_mm = int(round(delta_x_mm))
        move_y_mm = int(round(delta_y_mm))

        move_x_steps = move_x_mm * x_steps_per_mm
        move_y_steps = move_y_mm * y_steps_per_mm

        command = f"x{move_x_steps}y{move_y_steps}\n"
        arduino.write(command.encode())
        print("已发送指令:", command)

        current_x_mm += move_x_mm
        current_y_mm += move_y_mm

        time.sleep(10)
        print("目标移动完成")

        # ======= Z轴动作 =======
        print("Z轴下降中...")
        arduino.write(b"l\n")
        time.sleep(1)

        print("Z轴抬起中...")
        arduino.write(b"r\n")
        time.sleep(1)


    else:
        print("质心计算失败")
else:
    print("未检测到目标")

# ======== 回原点选择 ========
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