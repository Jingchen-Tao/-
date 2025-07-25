import serial
import time

# 最新校准后的步长换算
# 最终标定系数（由实验反推）
steps_per_mm_x = 50.54
steps_per_mm_y = 51.50


# 目标位移
move_x_mm = 0
move_y_mm = 0

# 换算成步数
move_x_steps = int(move_x_mm * steps_per_mm_x)
move_y_steps = int(move_y_mm * steps_per_mm_y)

# 串口初始化
arduino = serial.Serial(port='COM4', baudrate=115200, timeout=1)
time.sleep(2)  # 给Arduino预留启动时间

# 格式化串口指令
command = f"x{move_x_steps}y{move_y_steps}\n"
arduino.write(command.encode())
print("已发送:", command.strip())

# 等待机械执行完成
time.sleep(10)

arduino.close()
print("运动完成，已关闭串口。")
