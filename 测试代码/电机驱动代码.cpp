// 定义电机控制引脚
int leftMotor1 = 12;  // 左侧方向端口1
int leftMotor2 = 13;  // 左侧方向端口2
int rightMotor1 = 10; // 右侧方向端口1
int rightMotor2 = 11; // 右侧方向端口2
int leftPWM = 5;      // 左侧PWM端口
int rightPWM = 6;     // 右侧PWM端口

// 定义模式枚举
enum Mode {
  LEFT_FORWARD_FULL,   // 左轮全速前转
  LEFT_BACKWARD_FULL,  // 左轮全速后转
  LEFT_FORWARD_HALF,   // 左轮半速前转
  LEFT_BACKWARD_HALF,  // 左轮半速后转
  RIGHT_FORWARD_FULL,  // 右轮全速前转
  RIGHT_BACKWARD_FULL, // 右轮全速后转
  RIGHT_FORWARD_HALF,  // 右轮半速前转
  RIGHT_BACKWARD_HALF, // 右轮半速后转
  STOP_ALL             // 停止所有电机
};

Mode currentMode = LEFT_FORWARD_FULL; // 初始模式为左轮全速前转
unsigned long previousTime = 0; // 用于计时
const long interval = 1000; // 间隔时间为1秒

void setup() {
  // 设置引脚模式
  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);
  pinMode(leftPWM, OUTPUT);
  pinMode(rightPWM, OUTPUT);

  // 初始化串口通信
  Serial.begin(9600);
  Serial.println("Starting motor control loop...");
}

void loop() {
  unsigned long currentTime = millis(); // 获取当前时间

  // 每隔1秒切换模式
  if (currentTime - previousTime >= interval) {
    previousTime = currentTime; // 更新时间

    // 根据当前模式执行操作
    switch (currentMode) {
      case LEFT_FORWARD_FULL:
        digitalWrite(leftMotor1, HIGH);
        digitalWrite(leftMotor2, LOW);
        analogWrite(leftPWM, 255); // 左轮全速前转
        Serial.println("Mode: Left wheel forward (full speed)");
        break;

      case LEFT_BACKWARD_FULL:
        digitalWrite(leftMotor1, LOW);
        digitalWrite(leftMotor2, HIGH);
        analogWrite(leftPWM, 255); // 左轮全速后转
        Serial.println("Mode: Left wheel backward (full speed)");
        break;

      case LEFT_FORWARD_HALF:
        digitalWrite(leftMotor1, HIGH);
        digitalWrite(leftMotor2, LOW);
        analogWrite(leftPWM, 128); // 左轮半速前转
        Serial.println("Mode: Left wheel forward (half speed)");
        break;

      case LEFT_BACKWARD_HALF:
        digitalWrite(leftMotor1, LOW);
        digitalWrite(leftMotor2, HIGH);
        analogWrite(leftPWM, 128); // 左轮半速后转
        Serial.println("Mode: Left wheel backward (half speed)");
        break;

      case RIGHT_FORWARD_FULL:
        digitalWrite(rightMotor1, HIGH);
        digitalWrite(rightMotor2, LOW);
        analogWrite(rightPWM, 255); // 右轮全速前转
        Serial.println("Mode: Right wheel forward (full speed)");
        break;

      case RIGHT_BACKWARD_FULL:
        digitalWrite(rightMotor1, LOW);
        digitalWrite(rightMotor2, HIGH);
        analogWrite(rightPWM, 255); // 右轮全速后转
        Serial.println("Mode: Right wheel backward (full speed)");
        break;

      case RIGHT_FORWARD_HALF:
        digitalWrite(rightMotor1, HIGH);
        digitalWrite(rightMotor2, LOW);
        analogWrite(rightPWM, 128); // 右轮半速前转
        Serial.println("Mode: Right wheel forward (half speed)");
        break;

      case RIGHT_BACKWARD_HALF:
        digitalWrite(rightMotor1, LOW);
        digitalWrite(rightMotor2, HIGH);
        analogWrite(rightPWM, 128); // 右轮半速后转
        Serial.println("Mode: Right wheel backward (half speed)");
        break;

      case STOP_ALL:
        digitalWrite(leftMotor1, LOW);
        digitalWrite(leftMotor2, LOW);
        digitalWrite(rightMotor1, LOW);
        digitalWrite(rightMotor2, LOW);
        analogWrite(leftPWM, 0); // 停止左轮
        analogWrite(rightPWM, 0); // 停止右轮
        Serial.println("Mode: Stop all motors");
        break;
    }

    // 切换到下一个模式
    currentMode = static_cast<Mode>((currentMode + 1) % 9);
  }
}