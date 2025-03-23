// 定义雷达引脚
#define TrigLAD 3  // 雷达发射引脚
#define leftEcho 7  // 左雷达回波引脚
#define rightEcho 8 // 右雷达回波引脚
#define frontEcho 4 // 前雷达回波引脚

// 存储雷达距离值
float leftDistance = 0;   // 左雷达距离
float rightDistance = 0;  // 右雷达距离
float frontDistance = 0;  // 前雷达距离

void setup() {
  // 设置引脚模式
  pinMode(TrigLAD, OUTPUT);   // 发射引脚为输出
  pinMode(leftEcho, INPUT);   // 左雷达回波引脚为输入
  pinMode(rightEcho, INPUT);  // 右雷达回波引脚为输入
  pinMode(frontEcho, INPUT);  // 前雷达回波引脚为输入

  // 初始化串口通信
  Serial.begin(9600);
  Serial.println("Radar distance testing started...");
}

void loop() {
  // 读取左雷达距离
  leftDistance = readDistance(TrigLAD, leftEcho);
  // 读取前雷达距离
  frontDistance = readDistance(TrigLAD, frontEcho);
  // 读取右雷达距离
  rightDistance = readDistance(TrigLAD, rightEcho);

  // 输出雷达检测结果
  Serial.print("Left Distance: ");
  Serial.print(leftDistance);
  Serial.print(" cm | Front Distance: ");
  Serial.print(frontDistance);
  Serial.print(" cm | Right Distance: ");
  Serial.print(rightDistance);
  Serial.println(" cm");

  delay(500); // 每隔500毫秒检测一次
}

// 读取超声波雷达距离
float readDistance(int trigPin, int echoPin) {
  // 发送10微秒的触发信号
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // 读取回波时间
  long duration = pulseIn(echoPin, HIGH);

  // 计算距离（单位：厘米）
  float distance = duration * 0.034 / 2;

  // 返回距离值
  return distance;
}