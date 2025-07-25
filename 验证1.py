import serial
import time

# 步长换算
x_steps_per_mm = -50
y_steps_per_mm = 0


# 目标位移 (100mm)
move_x_mm=0
move_y_mm =100

# 换算成步数
move_x_steps = move_x_mm * x_steps_per_mm
move_y_steps = move_y_mm * y_steps_per_mm

# 串口初始化
arduino = serial.Serial(port='COM3', baudrate=115200, timeout=2)
time.sleep(2)

# 格式化串口指令
command = f"x{move_x_steps}y{move_y_steps}\n"
arduino.write(command.encode())
print("已发送:", command)

# 等待机械执行完成
time.sleep(10)

arduino.close()

"""
#include <AccelStepper.h>

// 统一步长配置（工业版）
const float steps_per_mm = 49.75;


// 驱动引脚定义 (你实际用什么引脚可自行调整)
AccelStepper stepperX(AccelStepper::DRIVER, 30, 31);
AccelStepper stepperY(AccelStepper::DRIVER, 39, 41);

void setup() {
  Serial.begin(115200);
  Serial.println("Ready for Industrial Insert Control V1.0");

  // 工业参数：高速度、高加速度
  stepperX.setMaxSpeed(10000);  //
  stepperY.setMaxSpeed(10000);
  stepperX.setAcceleration(700);
  stepperY.setAcceleration(20000);
}

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    long x_steps = 0, y_steps = 0;
    int result = sscanf(command.c_str(), "x%ldy%ld", &x_steps, &y_steps);
    if (result == 2) {
      Serial.print("Moving X: "); Serial.print(x_steps);
      Serial.print(" steps, Y: "); Serial.println(y_steps);

      // 插补核心：目标位移
      stepperX.move(x_steps);
      stepperY.move(y_steps);

      // 硬实时插补调度
      while (stepperX.distanceToGo() != 0 || stepperY.distanceToGo() != 0) {
        stepperX.run();
        stepperY.run();
      }

      Serial.println("Move done.");
    } else {
      Serial.println("Invalid command format.");
    }
  }
}

"""