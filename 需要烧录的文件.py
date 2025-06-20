// 简化版
Arduino
Mega
控制程序
// 只控制
XY
步进电机移动

# include <Stepper.h>

// 你可以在这里调整机械参数方便调试：
int
stepsPerRevolution = 200; // 每圈步数(可调)

// X轴步进电机接线
const
int
XstepPin1 = 30;
const
int
XstepPin2 = 31;
Stepper
stepperX(stepsPerRevolution, XstepPin1, XstepPin2);

// Y轴步进电机接线
const
int
YstepPin1 = 39;
const
int
YstepPin2 = 41;
Stepper
stepperY(stepsPerRevolution, YstepPin1, YstepPin2);

void
setup()
{
    Serial.begin(9600);
stepperX.setSpeed(100); // 转速(RPM，可调)
stepperY.setSpeed(100);
Serial.println("Arduino XY 控制已就绪");
}

void
loop()
{
if (Serial.available())
{
    String
input = Serial.readStringUntil('\n');
input.trim();

// 解析
x与y指令
if (input.startsWith("x") & & input.indexOf("y") > 0)
{
    int
xpos = input.substring(1, input.indexOf("y")).toInt();
int
ypos = input.substring(input.indexOf("y") + 1).toInt();

Serial.print("接收到指令: X=");
Serial.print(xpos);
Serial.print(" Y=");
Serial.println(ypos);

// 移动XY两轴
stepperX.step(xpos);
stepperY.step(ypos);
}
else {
    Serial.print("无法识别指令: ");
Serial.println(input);
}
}
}
// 简化版
Arduino
Mega
控制程序
// 只控制
XY
步进电机移动

# include <Stepper.h>

// 你可以在这里调整机械参数方便调试：
int
stepsPerRevolution = 200; // 每圈步数(可调)

// X轴步进电机接线
const
int
XstepPin1 = 30;
const
int
XstepPin2 = 31;
Stepper
stepperX(stepsPerRevolution, XstepPin1, XstepPin2);

// Y轴步进电机接线
const
int
YstepPin1 = 39;
const
int
YstepPin2 = 41;
Stepper
stepperY(stepsPerRevolution, YstepPin1, YstepPin2);

void
setup()
{
    Serial.begin(9600);
stepperX.setSpeed(100); // 转速(RPM，可调)
stepperY.setSpeed(100);
Serial.println("Arduino XY 控制已就绪");
}

void
loop()
{
if (Serial.available())
{
    String
input = Serial.readStringUntil('\n');
input.trim();

// 解析
x与y指令
if (input.startsWith("x") & & input.indexOf("y") > 0)
{
    int
xpos = input.substring(1, input.indexOf("y")).toInt();
int
ypos = input.substring(input.indexOf("y") + 1).toInt();

Serial.print("接收到指令: X=");
Serial.print(xpos);
Serial.print(" Y=");
Serial.println(ypos);

// 移动XY两轴
stepperX.step(xpos);
stepperY.step(ypos);
}
else {
    Serial.print("无法识别指令: ");
Serial.println(input);
}
}
}
