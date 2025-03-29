// 定义红外循迹模块引脚
#define IR_1 A0  // 红外模块1
#define IR_2 A1  // 红外模块2
#define IR_3 A2  // 红外模块3
#define IR_4 A3  // 红外模块4
#define IR_5 A4  // 红外模块5
#define IR_6 A5  // 红外模块6

// 存储红外模块的检测值
int irValues[6];  // 用于存储6个红外模块的检测值

void setup() {
  // 设置引脚模式
  pinMode(IR_1, INPUT);
  pinMode(IR_2, INPUT);
  pinMode(IR_3, INPUT);
  pinMode(IR_4, INPUT);
  pinMode(IR_5, INPUT);
  pinMode(IR_6, INPUT);

  // 初始化串口通信
  Serial.begin(9600);
  Serial.println("Six-channel IR line follower testing started...");
}

void loop() {
  // 读取6个红外模块的值
  irValues[0] = digitalRead(IR_1);
  irValues[1] = digitalRead(IR_2);
  irValues[2] = digitalRead(IR_3);
  irValues[3] = digitalRead(IR_4);
  irValues[4] = digitalRead(IR_5);
  irValues[5] = digitalRead(IR_6);

  // 输出红外模块的检测值
  Serial.print("IR 1: ");
  Serial.print(irValues[0]);
  Serial.print(" | IR 2: ");
  Serial.print(irValues[1]);
  Serial.print(" | IR 3: ");
  Serial.print(irValues[2]);
  Serial.print(" | IR 4: ");
  Serial.print(irValues[3]);
  Serial.print(" | IR 5: ");
  Serial.print(irValues[4]);
  Serial.print(" | IR 6: ");
  Serial.println(irValues[5]);

  delay(200); // 每隔200毫秒检测一次
}