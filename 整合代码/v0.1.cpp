#include <SoftwareSerial.h>
#include <Servo.h>

// 蓝牙模块引脚定义
#define BT_RX 0 // 板子上是0
#define BT_TX 1
SoftwareSerial BTSerial(BT_RX, BT_TX);

// 红外循迹模块引脚定义
#define IR_1 A0
#define IR_2 A1
#define IR_3 A2
#define IR_4 A3
#define IR_5 A4
#define IR_6 A5
int irValues[6];  // 存储6个红外模块的检测值

// 雷达引脚定义
#define TrigLAD 3
#define leftEcho 7
#define rightEcho 8
#define frontEcho 4
float leftDistance = 0; 
float rightDistance = 0;
float frontDistance = 0;

// 电机控制引脚定义
int leftMotor1 = 12;
int leftMotor2 = 13;
int rightMotor1 = 10;  
int rightMotor2 = 11;  
int leftPWM = 5;
int rightPWM = 6;

// 舵机引脚定义
#define SERVO_PIN 9
Servo myServo;
int servoAngle = 90;  // 默认中间位置

// 系统模式定义
enum SystemMode {
  MODE_INFRARED_TRACKING,  // 红外循迹模式
  MODE_RADAR_AVOIDANCE,    // 雷达避障模式
  MODE_MANUAL_CONTROL      // 手动控制模式
};
SystemMode currentMode = MODE_MANUAL_CONTROL;  // 默认手动控制

// 手动控制模式下的运动状态
enum ManualState {
  MANUAL_STOP,
  MANUAL_FORWARD,
  MANUAL_BACKWARD,
  MANUAL_LEFT,
  MANUAL_RIGHT
};
ManualState manualState = MANUAL_STOP;
bool manualFastSpeed = true;  // 默认快速模式
bool servoEnabled = false;    // 默认舵机关闭

void setup() {
  // 初始化串口通信
  Serial.begin(9600);
  BTSerial.begin(9600);
  
  // 初始化红外模块引脚
  pinMode(IR_1, INPUT);
  pinMode(IR_2, INPUT);
  pinMode(IR_3, INPUT);
  pinMode(IR_4, INPUT);
  pinMode(IR_5, INPUT);
  pinMode(IR_6, INPUT);
  
  // 初始化雷达模块引脚
  pinMode(TrigLAD, OUTPUT);
  pinMode(leftEcho, INPUT);
  pinMode(rightEcho, INPUT);
  pinMode(frontEcho, INPUT);
  
  // 初始化电机控制引脚
  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);
  pinMode(leftPWM, OUTPUT);
  pinMode(rightPWM, OUTPUT);
  
  // 初始化舵机
  myServo.attach(SERVO_PIN);
  myServo.write(servoAngle);
  
  Serial.println("Smart Car System Initialized");
  BTSerial.println("Smart Car System Initialized");
  printCurrentMode();
}

void loop() {
  // 处理蓝牙通信
  handleBluetooth();
  
  // 根据当前模式执行相应功能
  switch(currentMode) {
    case MODE_INFRARED_TRACKING:
      infraredTracking();
      break;
    case MODE_RADAR_AVOIDANCE:
      radarAvoidance();
      break;
    case MODE_MANUAL_CONTROL:
      manualControl();
      break;
  }
  
  delay(50);  // 适当延迟，减少处理负载
}

// 处理蓝牙通信
void handleBluetooth() {
  if (BTSerial.available()) {
    char command = BTSerial.read();
    Serial.print("Received command: ");
    Serial.println(command);
    
    switch(command) {
      case '1':  // 切换到红外循迹模式
        currentMode = MODE_INFRARED_TRACKING;
        stopMotors();
        printCurrentMode();
        break;
      case '2':  // 切换到雷达避障模式
        currentMode = MODE_RADAR_AVOIDANCE;
        stopMotors();
        printCurrentMode();
        break;
      case '3':  // 切换到手动控制模式
        currentMode = MODE_MANUAL_CONTROL;
        stopMotors();
        printCurrentMode();
        break;
        
      // 手动控制模式下的命令
      case 'F':  // 前进
        if (currentMode == MODE_MANUAL_CONTROL) manualState = MANUAL_FORWARD;
        break;
      case 'B':  // 后退
        if (currentMode == MODE_MANUAL_CONTROL) manualState = MANUAL_BACKWARD;
        break;
      case 'L':  // 左转
        if (currentMode == MODE_MANUAL_CONTROL) manualState = MANUAL_LEFT;
        break;
      case 'R':  // 右转
        if (currentMode == MODE_MANUAL_CONTROL) manualState = MANUAL_RIGHT;
        break;
      case 'S':  // 停止
        if (currentMode == MODE_MANUAL_CONTROL) manualState = MANUAL_STOP;
        break;
      case 'V':  // 速度切换(快/慢)
        if (currentMode == MODE_MANUAL_CONTROL) manualFastSpeed = !manualFastSpeed;
        break;
      case 'X':  // 舵机开关
        if (currentMode == MODE_MANUAL_CONTROL) {
          servoEnabled = !servoEnabled;
          if (!servoEnabled) myServo.write(90);  // 关闭时回到中间位置
        }
        break;
      case 'C':  // 舵机左转
        if (currentMode == MODE_MANUAL_CONTROL && servoEnabled) {
          servoAngle = max(0, servoAngle - 15);
          myServo.write(servoAngle);
        }
        break;
      case 'D':  // 舵机右转
        if (currentMode == MODE_MANUAL_CONTROL && servoEnabled) {
          servoAngle = min(180, servoAngle + 15);
          myServo.write(servoAngle);
        }
        break;
    }
  }
}

// 红外循迹功能
void infraredTracking() {
  // 读取红外传感器值
  irValues[0] = digitalRead(IR_1);
  irValues[1] = digitalRead(IR_2);
  irValues[2] = digitalRead(IR_3);
  irValues[3] = digitalRead(IR_4);
  irValues[4] = digitalRead(IR_5);
  irValues[5] = digitalRead(IR_6);
  
  // 根据传感器值判断路线
  if (irValues[2] == 0 && irValues[3] == 0) {
    // 中间两个传感器检测到黑线，直行
    moveForward(150);
  } 
  else if (irValues[0] == 0 || irValues[1] == 0) {
    // 左侧传感器检测到黑线，左转
    turnLeft(200);
  } 
  else if (irValues[4] == 0 || irValues[5] == 0) {
    // 右侧传感器检测到黑线，右转
    turnRight(200);
  } 
  else if (irValues[0] == 0 && irValues[1] == 0 && irValues[2] == 0) {
    // 检测到直角或锐角转弯(左侧)
    sharpTurnLeft();
  } 
  else if (irValues[3] == 0 && irValues[4] == 0 && irValues[5] == 0) {
    // 检测到直角或锐角转弯(右侧)
    sharpTurnRight();
  } 
  else {
    // 未检测到黑线，停止或缓慢前进
    stopMotors();
  }
}

// 雷达避障功能(默认靠右走)
void radarAvoidance() {
  // 读取雷达距离
  leftDistance = readDistance(TrigLAD, leftEcho);
  frontDistance = readDistance(TrigLAD, frontEcho);
  rightDistance = readDistance(TrigLAD, rightEcho);
  
  // 避障逻辑
  if (frontDistance < 20) {
    // 前方有障碍物
    if (rightDistance > leftDistance && rightDistance > 30) {
      // 右侧空间较大，右转
      turnRight(150);
      delay(300);
    } else if (leftDistance > 30) {
      // 左侧空间较大，左转
      turnLeft(150);
      delay(300);
    } else {
      // 两侧空间都不足，后退
      moveBackward(150);
      delay(500);
    }
  } else if (rightDistance < 15) {
    // 右侧距离太近，稍向左调整
    moveForward(150);
    analogWrite(leftPWM, 100);
    analogWrite(rightPWM, 200);
  } else if (rightDistance > 25) {
    // 右侧距离太远，稍向右调整
    moveForward(150);
    analogWrite(leftPWM, 200);
    analogWrite(rightPWM, 100);
  } else {
    // 保持直行，靠右走
    moveForward(150);
  }
}

// 手动控制功能
void manualControl() {
  int speed = manualFastSpeed ? 255 : 150;
  
  switch(manualState) {
    case MANUAL_STOP:
      stopMotors();
      break;
    case MANUAL_FORWARD:
      moveForward(speed);
      break;
    case MANUAL_BACKWARD:
      moveBackward(speed);
      break;
    case MANUAL_LEFT:
      turnLeft(speed);
      break;
    case MANUAL_RIGHT:
      turnRight(speed);
      break;
  }
}

// 读取超声波距离
float readDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duration = pulseIn(echoPin, HIGH);
  float distance = duration * 0.034 / 2;
  
  return distance;
}

// 电机控制函数
void moveForward(int speed) {
  digitalWrite(leftMotor1, HIGH);
  digitalWrite(leftMotor2, LOW);
  digitalWrite(rightMotor1, HIGH);
  digitalWrite(rightMotor2, LOW);
  analogWrite(leftPWM, speed);
  analogWrite(rightPWM, speed);
}

void moveBackward(int speed) {
  digitalWrite(leftMotor1, LOW);
  digitalWrite(leftMotor2, HIGH);
  digitalWrite(rightMotor1, LOW);
  digitalWrite(rightMotor2, HIGH);
  analogWrite(leftPWM, speed);
  analogWrite(rightPWM, speed);
}

void turnLeft(int speed) {
  digitalWrite(leftMotor1, LOW);
  digitalWrite(leftMotor2, HIGH);
  digitalWrite(rightMotor1, HIGH);
  digitalWrite(rightMotor2, LOW);
  analogWrite(leftPWM, speed);
  analogWrite(rightPWM, speed);
}

void turnRight(int speed) {
  digitalWrite(leftMotor1, HIGH);
  digitalWrite(leftMotor2, LOW);
  digitalWrite(rightMotor1, LOW);
  digitalWrite(rightMotor2, HIGH);
  analogWrite(leftPWM, speed);
  analogWrite(rightPWM, speed);
}

void sharpTurnLeft() {
  // 急左转
  turnLeft(255);
  delay(300);
  // 继续左转直到中间传感器检测到黑线
  while (digitalRead(IR_3) == 1) {
    turnLeft(200);
    delay(10);
  }
}

void sharpTurnRight() {
  // 急右转
  turnRight(255);
  delay(300);
  // 继续右转直到中间传感器检测到黑线
  while (digitalRead(IR_4) == 1) {
    turnRight(200);
    delay(10);
  }
}

void stopMotors() {
  digitalWrite(leftMotor1, LOW);
  digitalWrite(leftMotor2, LOW);
  digitalWrite(rightMotor1, LOW);
  digitalWrite(rightMotor2, LOW);
  analogWrite(leftPWM, 0);
  analogWrite(rightPWM, 0);
}

void printCurrentMode() {
  String modeStr;
  switch(currentMode) {
    case MODE_INFRARED_TRACKING: modeStr = "Infrared Tracking Mode"; break;
    case MODE_RADAR_AVOIDANCE: modeStr = "Radar Avoidance Mode"; break;
    case MODE_MANUAL_CONTROL: modeStr = "Manual Control Mode"; break;
  }
  
  Serial.println("Current Mode: " + modeStr);
  BTSerial.println("Current Mode: " + modeStr);
  
  if (currentMode == MODE_MANUAL_CONTROL) {
    BTSerial.println("Manual Control Commands:");
    BTSerial.println("F - Forward, B - Backward");
    BTSerial.println("L - Left, R - Right");
    BTSerial.println("S - Stop");
    BTSerial.println("V - Toggle Speed");
    BTSerial.println("X - Toggle Servo");
    BTSerial.println("C - Servo Left, D - Servo Right");
  }
}