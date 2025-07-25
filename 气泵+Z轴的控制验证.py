"""
pump_control.py
----------------
在 PyCharm 终端一步完成气泵开关：
1. 询问是否打开气泵
2. 询问是否关闭气泵
"""

import serial
import time

PORT     = "COM3"        # 根据实际修改
BAUDRATE = 115200

def ask_yes_no(question: str) -> bool:
    """简易 y/n 提示"""
    while True:
        ans = input(f"{question} (y/n): ").strip().lower()
        if ans in ("y", "n"):
            return ans == "y"
        print("请输入 y 或 n。")

def main():
    # 打开串口并等待 Arduino 复位
    ser = serial.Serial(PORT, BAUDRATE, timeout=1)
    time.sleep(2)
    ser.reset_input_buffer()

    # 1) 是否打开气泵
    if ask_yes_no("是否打开气泵？"):
        ser.write(b'a\n')
        print(">> 已发送 'a'  (Pump ON)")
        # 打印来自 Arduino 的反馈
        print("[Arduino]", ser.readline().decode().strip())
    else:
        print(">> 跳过打开气泵")

    # 2) 是否关闭气泵
    if ask_yes_no("是否关闭气泵？"):
        ser.write(b'b\n')
        print(">> 已发送 'b'  (Pump OFF)")
        print("[Arduino]", ser.readline().decode().strip())
    else:
        print(">> 气泵保持当前状态")

    ser.close()
    print("脚本结束，串口已关闭。")

if __name__ == "__main__":
    main()


"""
#include <AccelStepper.h>
#include <Stepper.h>

const float STEPS_PER_MM_XY = 49.75;        // 仅供参考
const float STEPS_PER_MM_Z  = 49.75;        // 根据实测调整

// ---------- 硬件引脚 ----------
const int Z_PIN1   = 38;
const int Z_PIN2   = 40;
const int PUMP_PIN = 28;                    // 气泵（继电器 / MOSFET）

// ---------- 步进电机对象 ----------
AccelStepper stepperX(AccelStepper::DRIVER, 30, 31);
AccelStepper stepperY(AccelStepper::DRIVER, 39, 41);
Stepper       stepperZ(1000, Z_PIN1, Z_PIN2);   // 1000 → 每圈步数，自行调整

void setup() {
  Serial.begin(115200);
  Serial.println("Controller ready.");

  stepperX.setMaxSpeed(10000);
  stepperY.setMaxSpeed(10000);
  stepperX.setAcceleration(700);
  stepperY.setAcceleration(20000);
  stepperZ.setSpeed(100000);

  pinMode(PUMP_PIN, OUTPUT);
  digitalWrite(PUMP_PIN, LOW);              // 默认关闭气泵
}

void loop() {
  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();

  // ---------- XY 联动 ----------
  if (cmd.startsWith("x")) {
    long sx = 0, sy = 0;
    if (sscanf(cmd.c_str(), "x%ldy%ld", &sx, &sy) == 2) {
      stepperX.move(sx);
      stepperY.move(sy);
      while (stepperX.distanceToGo() || stepperY.distanceToGo()) {
        stepperX.run();
        stepperY.run();
      }
      Serial.println("XY done");
    } else {
      Serial.println("Bad XY format");
    }
  }
  // ---------- Z 位移（毫米） ----------
  else if (cmd.startsWith("move_z_mm")) {
    float dz = 0;
    if (sscanf(cmd.c_str(), "move_z_mm%f", &dz) == 1) {
      long steps = (long)(dz * STEPS_PER_MM_Z);
      stepperZ.step(steps);
      Serial.println("Z done");
    } else {
      Serial.println("Bad Z format");
    }
  }
  // ---------- 气泵手动开关 ----------
  else if (cmd == "a") {         // 开
    digitalWrite(PUMP_PIN, HIGH);
    Serial.println("Pump ON");
  }
  else if (cmd == "b") {         // 关
    digitalWrite(PUMP_PIN, LOW);
    Serial.println("Pump OFF");
  }
  else {
    Serial.println("Unknown cmd");
  }
}

"""