/*
 * 智能小车主控程序
 * 功能：
 * 1. 红外循迹（直行、直角转弯、锐角转弯、钝角转弯）
 * 2. 雷达避障（默认靠右侧走）
 * 3. 蓝牙手动控制（前进/后退/转向/速度控制/舵机控制）
 * 
 * 修改说明：
 * 1. 增加详细注释
 * 2. 蓝牙通信迁移到默认串口0/1（移除了SoftwareSerial）
 * 3. 调整了电机控制引脚避免冲突
 */

 #include <Servo.h>  // 舵机控制库

 /* ========== 硬件引脚定义 ========== */
 // 红外循迹模块引脚（6路红外传感器）
 #define IR_1 A0  // 最左侧红外传感器
 #define IR_2 A1  // 左侧红外传感器
 #define IR_3 A2  // 左中红外传感器
 #define IR_4 A3  // 右中红外传感器
 #define IR_5 A4  // 右侧红外传感器
 #define IR_6 A5  // 最右侧红外传感器
 
 // 超声波雷达引脚（共用Trig，3个Echo）
 #define TrigLAD 3   // 超声波触发引脚（三个雷达共用）
 #define leftEcho 7  // 左侧雷达回波引脚
 #define rightEcho 8 // 右侧雷达回波引脚
 #define frontEcho 4 // 前方雷达回波引脚
 
 // 电机控制引脚（调整为不与串口冲突的引脚）
 #define leftMotor1 12  // 左侧电机方向控制1
 #define leftMotor2 13  // 左侧电机方向控制2
 #define rightMotor1 10 // 右侧电机方向控制1
 #define rightMotor2 11 // 右侧电机方向控制2
 #define leftPWM 5      // 左侧电机PWM速度控制
 #define rightPWM 6     // 右侧电机PWM速度控制
 
 // 舵机控制引脚
 #define SERVO_PIN 9  // 舵机信号引脚
 
 /* ========== 全局变量定义 ========== */
 // 红外传感器值存储数组
 int irValues[6];  // 索引0-5对应IR_1到IR_6
 
 // 雷达距离值（单位：厘米）
 float leftDistance = 0;   // 左侧障碍物距离
 float rightDistance = 0;  // 右侧障碍物距离
 float frontDistance = 0;  // 前方障碍物距离
 
 // 舵机控制变量
 Servo myServo;        // 舵机对象
 int servoAngle = 90;  // 舵机角度（范围0-180，90为中间位置）
 bool servoEnabled = false; // 舵机使能标志
 
 // 系统工作模式枚举
 enum SystemMode {
   MODE_INFRARED_TRACKING,  // 红外循迹模式
   MODE_RADAR_AVOIDANCE,    // 雷达避障模式
   MODE_MANUAL_CONTROL      // 手动控制模式
 };
 SystemMode currentMode = MODE_INFRARED_TRACKING;  // 当前系统模式，默认为红外循迹
 
 // 手动控制模式下的运动状态
 enum ManualState {
   MANUAL_STOP,      // 停止
   MANUAL_FORWARD,   // 前进
   MANUAL_BACKWARD,  // 后退
   MANUAL_LEFT,      // 左转
   MANUAL_RIGHT      // 右转
 };
 ManualState manualState = MANUAL_STOP;  // 手动控制状态
 bool manualFastSpeed = true;           // 手动控制速度标志（true为快速，false为慢速）
 
 /* ========== 初始化函数 ========== */
 void setup() {
   // 初始化串口通信（使用默认串口0/1，波特率9600）
   Serial.begin(9600);
   
   // 初始化红外传感器引脚
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
   myServo.write(servoAngle);  // 初始位置居中
   
   // 初始状态停止所有电机
   stopMotors();
   
   // 打印初始化信息
   Serial.println(F("Smart Car System Initialized"));
   printCurrentMode();  // 打印当前模式信息
 }
 
 /* ========== 主循环函数 ========== */
 void loop() {
   // 处理蓝牙串口命令（使用默认Serial）
   handleBluetooth();
   
   // 根据当前模式执行相应功能
   switch(currentMode) {
     case MODE_INFRARED_TRACKING:
       infraredTracking();  // 红外循迹模式
       break;
     case MODE_RADAR_AVOIDANCE:
       radarAvoidance();    // 雷达避障模式
       break;
     case MODE_MANUAL_CONTROL:
       manualControl();     // 手动控制模式
       break;
   }
   
   delay(50);  // 主循环延迟，降低CPU负载
 }
 
 /* ========== 蓝牙命令处理函数 ========== */
 /**
  * 处理蓝牙串口接收到的命令
  * 使用默认Serial（硬件串口）进行通信
  */
 void handleBluetooth() {
   if (Serial.available()) {
     char command = Serial.read();
     Serial.print(F("Received command: "));
     Serial.println(command);
     
     // 模式切换命令
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
         Serial.print(F("Speed changed to: "));
         Serial.println(manualFastSpeed ? "Fast" : "Slow");
         break;
       case 'X':  // 舵机开关
         if (currentMode == MODE_MANUAL_CONTROL) {
           servoEnabled = !servoEnabled;
           if (!servoEnabled) {
             myServo.write(90);  // 关闭时回到中间位置
             Serial.println(F("Servo disabled"));
           } else {
             Serial.println(F("Servo enabled"));
           }
         }
         break;
       case 'C':  // 舵机左转
         if (currentMode == MODE_MANUAL_CONTROL && servoEnabled) {
           servoAngle = max(0, servoAngle - 15);
           myServo.write(servoAngle);
           Serial.print(F("Servo angle: "));
           Serial.println(servoAngle);
         }
         break;
       case 'D':  // 舵机右转
         if (currentMode == MODE_MANUAL_CONTROL && servoEnabled) {
           servoAngle = min(180, servoAngle + 15);
           myServo.write(servoAngle);
           Serial.print(F("Servo angle: "));
           Serial.println(servoAngle);
         }
         break;
     }
   }
 }
 
 /* ========== 红外循迹功能函数 ========== */
 /**
  * 红外循迹模式主函数
  * 根据6路红外传感器值控制小车运动
  * 能够处理直线、直角转弯、锐角转弯等情况
  */
 void infraredTracking() {
   // 读取所有红外传感器值（0表示检测到黑线，1表示未检测到）
   irValues[0] = digitalRead(IR_1);
   irValues[1] = digitalRead(IR_2);
   irValues[2] = digitalRead(IR_3);
   irValues[3] = digitalRead(IR_4);
   irValues[4] = digitalRead(IR_5);
   irValues[5] = digitalRead(IR_6);
   
   // 判断路线情况并控制小车运动
   if (irValues[2] == 0 && irValues[3] == 0) {
     // 情况1：中间两个传感器检测到黑线 - 直行
     moveForward(150);  // 中等速度直行
   } 
   else if (irValues[0] == 0 || irValues[1] == 0) {
     // 情况2：左侧传感器检测到黑线 - 左转
     turnLeft(200);  // 较大速度左转
   } 
   else if (irValues[4] == 0 || irValues[5] == 0) {
     // 情况3：右侧传感器检测到黑线 - 右转
     turnRight(200);  // 较大速度右转
   } 
   else if (irValues[0] == 0 && irValues[1] == 0 && irValues[2] == 0) {
     // 情况4：左侧多个传感器检测到黑线 - 直角/锐角左转
     sharpTurnLeft();  // 执行急左转
   } 
   else if (irValues[3] == 0 && irValues[4] == 0 && irValues[5] == 0) {
     // 情况5：右侧多个传感器检测到黑线 - 直角/锐角右转
     sharpTurnRight();  // 执行急右转
   } 
   else {
     // 情况6：未检测到黑线 - 停止
     stopMotors();
   }
 }
 
 /* ========== 雷达避障功能函数 ========== */
 /**
  * 雷达避障模式主函数
  * 使用3个超声波传感器检测障碍物
  * 默认策略靠右侧行走
  */
 void radarAvoidance() {
   // 读取三个方向的障碍物距离
   leftDistance = readDistance(TrigLAD, leftEcho);
   frontDistance = readDistance(TrigLAD, frontEcho);
   rightDistance = readDistance(TrigLAD, rightEcho);
   
   // 避障决策逻辑
   if (frontDistance < 20) {
     // 情况1：前方有障碍物（距离<20cm）
     if (rightDistance > leftDistance && rightDistance > 30) {
       // 右侧空间较大 - 右转避开
       turnRight(150);
       delay(300);  // 持续转弯一段时间
     } else if (leftDistance > 30) {
       // 左侧空间较大 - 左转避开
       turnLeft(150);
       delay(300);  // 持续转弯一段时间
     } else {
       // 两侧空间都不足 - 后退
       moveBackward(150);
       delay(500);  // 持续后退一段时间
     }
   } else if (rightDistance < 15) {
     // 情况2：右侧距离太近（<15cm） - 稍向左调整
     moveForward(150);
     analogWrite(leftPWM, 100);  // 左侧轮子慢速
     analogWrite(rightPWM, 200); // 右侧轮子快速
   } else if (rightDistance > 25) {
     // 情况3：右侧距离太远（>25cm） - 稍向右调整
     moveForward(150);
     analogWrite(leftPWM, 200);  // 左侧轮子快速
     analogWrite(rightPWM, 100); // 右侧轮子慢速
   } else {
     // 情况4：正常情况 - 保持直行靠右走
     moveForward(150);
   }
 }
 
 /* ========== 手动控制功能函数 ========== */
 /**
  * 手动控制模式主函数
  * 根据蓝牙指令控制小车运动
  */
 void manualControl() {
   // 根据速度标志设置PWM值
   int speed = manualFastSpeed ? 255 : 150;
   
   // 根据当前手动状态控制电机
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
 
 /* ========== 超声波距离测量函数 ========== */
 /**
  * 读取超声波传感器距离
  * @param trigPin 触发引脚
  * @param echoPin 回波引脚
  * @return 测量到的距离（厘米）
  */
 float readDistance(int trigPin, int echoPin) {
   // 发送10微秒的触发脉冲
   digitalWrite(trigPin, LOW);
   delayMicroseconds(2);
   digitalWrite(trigPin, HIGH);
   delayMicroseconds(10);
   digitalWrite(trigPin, LOW);
   
   // 测量回波脉冲宽度
   long duration = pulseIn(echoPin, HIGH);
   
   // 计算距离（声速340m/s，除以2因为是往返距离）
   float distance = duration * 0.034 / 2;
   
   return distance;
 }
 
 /* ========== 电机控制函数 ========== */
 /**
  * 控制小车前进
  * @param speed 前进速度（0-255）
  */
 void moveForward(int speed) {
   digitalWrite(leftMotor1, HIGH);
   digitalWrite(leftMotor2, LOW);
   digitalWrite(rightMotor1, HIGH);
   digitalWrite(rightMotor2, LOW);
   analogWrite(leftPWM, speed);
   analogWrite(rightPWM, speed);
 }
 
 /**
  * 控制小车后退
  * @param speed 后退速度（0-255）
  */
 void moveBackward(int speed) {
   digitalWrite(leftMotor1, LOW);
   digitalWrite(leftMotor2, HIGH);
   digitalWrite(rightMotor1, LOW);
   digitalWrite(rightMotor2, HIGH);
   analogWrite(leftPWM, speed);
   analogWrite(rightPWM, speed);
 }
 
 /**
  * 控制小车左转
  * @param speed 转弯速度（0-255）
  */
 void turnLeft(int speed) {
   digitalWrite(leftMotor1, LOW);
   digitalWrite(leftMotor2, HIGH);
   digitalWrite(rightMotor1, HIGH);
   digitalWrite(rightMotor2, LOW);
   analogWrite(leftPWM, speed);
   analogWrite(rightPWM, speed);
 }
 
 /**
  * 控制小车右转
  * @param speed 转弯速度（0-255）
  */
 void turnRight(int speed) {
   digitalWrite(leftMotor1, HIGH);
   digitalWrite(leftMotor2, LOW);
   digitalWrite(rightMotor1, LOW);
   digitalWrite(rightMotor2, HIGH);
   analogWrite(leftPWM, speed);
   analogWrite(rightPWM, speed);
 }
 
 /**
  * 急左转（用于直角/锐角转弯）
  */
 void sharpTurnLeft() {
   // 第一阶段：快速左转
   turnLeft(255);
   delay(300);
   
   // 第二阶段：继续左转直到中间传感器检测到黑线
   while (digitalRead(IR_3) == 1) {
     turnLeft(200);
     delay(10);
   }
 }
 
 /**
  * 急右转（用于直角/锐角转弯）
  */
 void sharpTurnRight() {
   // 第一阶段：快速右转
   turnRight(255);
   delay(300);
   
   // 第二阶段：继续右转直到中间传感器检测到黑线
   while (digitalRead(IR_4) == 1) {
     turnRight(200);
     delay(10);
   }
 }
 
 /**
  * 停止所有电机
  */
 void stopMotors() {
   digitalWrite(leftMotor1, LOW);
   digitalWrite(leftMotor2, LOW);
   digitalWrite(rightMotor1, LOW);
   digitalWrite(rightMotor2, LOW);
   analogWrite(leftPWM, 0);
   analogWrite(rightPWM, 0);
 }
 
 /* ========== 辅助函数 ========== */
 /**
  * 打印当前系统模式信息到串口
  */
 void printCurrentMode() {
   String modeStr;
   switch(currentMode) {
     case MODE_INFRARED_TRACKING: 
       modeStr = F("Infrared Tracking Mode");
       break;
     case MODE_RADAR_AVOIDANCE: 
       modeStr = F("Radar Avoidance Mode");
       break;
     case MODE_MANUAL_CONTROL: 
       modeStr = F("Manual Control Mode");
       break;
   }
   
   Serial.print(F("Current Mode: "));
   Serial.println(modeStr);
   
   // 如果是手动模式，打印指令帮助信息
   if (currentMode == MODE_MANUAL_CONTROL) {
     Serial.println(F("Manual Control Commands:"));
     Serial.println(F("F - Forward, B - Backward"));
     Serial.println(F("L - Left, R - Right"));
     Serial.println(F("S - Stop"));
     Serial.println(F("V - Toggle Speed"));
     Serial.println(F("X - Toggle Servo"));
     Serial.println(F("C - Servo Left, D - Servo Right"));
   }
 }