// 使用Arduino+sr04 进行雷达测距
// 一共有三组雷达，分别是左3前2右1；端口为2-TG1；3-TG2；4-TG3；8-EC1；12-EC2；13 - EC3 
// 定义超声波模块引脚

const int trigPinLeft = 4;
const int trigPinFront = 3;
const int trigPinRight = 2;

const int echoPinLeft = 13;
const int echoPinFront = 12;
const int echoPinRight = 8;

void setup() {
  Serial.begin(9600); // 初始化串口通信
  
  // 设置引脚模式
  pinMode(trigPinLeft, OUTPUT);
  pinMode(echoPinLeft, INPUT);
  pinMode(trigPinFront, OUTPUT);
  pinMode(echoPinFront, INPUT);
  pinMode(trigPinRight, OUTPUT);
  pinMode(echoPinRight, INPUT);
  
  Serial.println("三向超声波雷达测距系统已启动");
}

void loop() {
  // 测量三个方向的距离
  float distanceLeft = getDistance(trigPinLeft, echoPinLeft);
  float distanceFront = getDistance(trigPinFront, echoPinFront);
  float distanceRight = getDistance(trigPinRight, echoPinRight);
  
  // 打印测量结果
  Serial.print("左侧距离: ");
  Serial.print(distanceLeft);
  Serial.print(" cm | 前方距离: ");
  Serial.print(distanceFront);
  Serial.print(" cm | 右侧距离: ");
  Serial.print(distanceRight);
  Serial.println(" cm");
  
  delay(200); // 适当延时
}

// 超声波测距函数
float getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duration = pulseIn(echoPin, HIGH);
  float distance = duration * 0.034 / 2; // 计算距离(cm)
  
  // 过滤异常值
  if(distance > 400 || distance < 2) {
    distance = -1; // 表示超出测量范围
  }
  
  return distance;
}