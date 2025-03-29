#include <Servo.h>  // 引入Servo库

// 定义舵机引脚
#define SERVO_PIN 11

// 创建舵机对象
Servo myServo;

// 定义舵机初始角度
int angle = 0;  // 初始角度为0度

void setup() {
  // 将舵机连接到指定引脚
  myServo.attach(SERVO_PIN);

  // 初始化串口通信
  Serial.begin(9600);
  Serial.println("Servo control started...");
}

void loop() {
  // 设置舵机角度
  myServo.write(angle);  // 将舵机转动到指定角度

  // 输出当前角度
  Serial.print("Servo angle: ");
  Serial.println(angle);

  // 更新角度
  angle += 30;  // 每次增加30度

  // 如果角度超过180度，重置为0度
  if (angle > 180) {
    angle = 0;
  }

  delay(1000);  // 每隔1秒转动一次
}