import serial
import time

# 串口初始化
arduino = serial.Serial(port='COM3', baudrate=9600, timeout=1)
time.sleep(2)

# 测试步进参数（你可以先试 1000步）
test_x_steps = 1000

# 向 X 轴正向移动
command_forward = f"x{test_x_steps}y0\n"
arduino.write(command_forward.encode())
print(f"发送前进: {command_forward.strip()}")
time.sleep(3)

# 向 X 轴返回
command_back = f"x{-test_x_steps}y0\n"
arduino.write(command_back.encode())
print(f"发送返回: {command_back.strip()}")
time.sleep(3)

arduino.close()
