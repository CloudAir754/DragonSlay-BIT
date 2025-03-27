/*
 * 智能小车主控程序 v0.4
 * 改进内容：
 * 1.删去错误的IR_THRESHOLD逻辑
 * 2.将Stop按钮，加入强制切换为任意情况停止（即按下后切入手动模式）
 * 3.梳理控制流
 * 4. 梳理手动的代码
 * 5. 增加不后退的转弯代码
 * 6. 删去乱的红外代码和雷达代码
 */

#include <Servo.h> // 舵机控制库

/* ========== 硬件引脚定义 ========== */
// 红外循迹模块引脚（6路红外传感器）
// 黑线（未收到足额反射光）==1；白=0
#define IR_1 A0 // 最左侧红外传感器
#define IR_2 A1 // 左侧红外传感器
#define IR_3 A2 // 左中红外传感器
#define IR_4 A3 // 右中红外传感器
#define IR_5 A4 // 右侧红外传感器
#define IR_6 A5 // 最右侧红外传感器

// 超声波雷达引脚（共用Trig，3个Echo）
#define TrigLAD 3   // 超声波触发引脚（三个雷达共用）
#define leftEcho 7  // 左侧雷达回波引脚
#define rightEcho 8 // 右侧雷达回波引脚
#define frontEcho 4 // 前方雷达回波引脚

// 电机控制引脚
#define leftMotor1 12  // 左侧电机方向控制1
#define leftMotor2 13  // 左侧电机方向控制2
#define rightMotor1 10 // 右侧电机方向控制1
#define rightMotor2 11 // 右侧电机方向控制2
#define leftPWM 5      // 左侧电机PWM速度控制
#define rightPWM 6     // 右侧电机PWM速度控制

// 舵机控制引脚
#define SERVO_PIN 9 // 舵机信号引脚

/* ========== 全局变量定义 ========== */

// 红外传感器值存储数组
int irValues[6]; // 索引0-5对应IR_1到IR_6

// 雷达距离值（单位：厘米）
float leftDistance = 0;  // 左侧障碍物距离
float rightDistance = 0; // 右侧障碍物距离
float frontDistance = 0; // 前方障碍物距离

// 舵机控制变量
Servo myServo;       // 舵机对象
int servoAngle = 90; // 舵机角度（范围0-180，90为中间位置）

// 系统工作模式枚举
enum SystemMode
{
    MODE_INFRARED_TRACKING, // 红外循迹模式
    MODE_RADAR_AVOIDANCE,   // 雷达避障模式
    MODE_MANUAL_CONTROL     // 手动控制模式
};
SystemMode currentMode = MODE_MANUAL_CONTROL; // 当前系统模式，默认为手动控制

// 手动控制模式下的运动状态
enum ManualState
{
    MANUAL_STOP,       // 停止
    MANUAL_FORWARD,    // 前进
    MANUAL_BACKWARD,   // 后退
    MANUAL_LEFT,       // 左转
    MANUAL_RIGHT,      // 右转
    MANUAL_LEFT_SMALL, // 小左转
    MANUAL_RIGHT_SMALL // 小右转
};
ManualState manualState = MANUAL_STOP; // 手动控制状态
bool manualFastSpeed = false;          // 手动控制速度标志（true为快速，false为慢速）

/* ========== 初始化函数 ========== */
/**
 * @brief 初始化硬件和系统设置
 * 配置所有输入输出引脚，初始化串口通信，
 * 设置舵机初始位置，停止所有电机
 */
void setup()
{
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
    myServo.write(servoAngle); // 初始位置居中

    // 初始状态停止所有电机
    stopMotors();

    // 打印初始化信息
    Serial.println(F("Smart Car System Initialized v0.3"));
    printCurrentMode(); // 打印当前模式信息
}

/* ========== 主循环函数 ========== */
/**
 * @brief 主程序循环
 * 处理蓝牙命令并根据当前模式执行相应功能
 * 包含50ms的延迟以降低CPU负载
 */
void loop()
{
    // 每次循环后，回到默认的STOP状态
    stopMotors();
    manualState = MANUAL_STOP;

    // 处理蓝牙串口命令（使用默认Serial）
    handleBluetooth();

    // 根据当前模式执行相应功能
    switch (currentMode)
    {
    case MODE_INFRARED_TRACKING:
        infraredTracking(); // 红外循迹模式
        break;
    case MODE_RADAR_AVOIDANCE:
        radarAvoidance(); // 雷达避障模式
        break;
    case MODE_MANUAL_CONTROL:
        manualControl(); // 手动控制模式
        break;
    }

    delay(50); // 主循环延迟，降低CPU负载
    // 单位毫秒
    // TODO A===【超参数】循环延迟，影响一次判断的周期长度
}

/* ========== 蓝牙命令处理函数 ========== */
/**
 * @brief 处理来自蓝牙串口的命令
 * 支持模式切换和手动控制命令
 *
 */
void handleBluetooth()
{
    // Serial.available() 是非阻塞的，返回缓冲区字节数
    //   若未输入，则整个函数直接跳过
    if (Serial.available())
    {
        char command = Serial.read();
        Serial.print(F("Received command: "));
        Serial.println(command);

        // 模式切换命令
        switch (command)
        {
        case '1': // 切换到红外循迹模式
            currentMode = MODE_INFRARED_TRACKING;
            stopMotors();
            myServo.write(90); // 重置舵机位置
            printCurrentMode();
            break;
        case '2': // 切换到雷达避障模式
            currentMode = MODE_RADAR_AVOIDANCE;
            stopMotors();
            myServo.write(90); // 重置舵机位置
            printCurrentMode();
            break;
        case '3': // 切换到手动控制模式
            currentMode = MODE_MANUAL_CONTROL;
            stopMotors();
            printCurrentMode();
            break;

        // 手动控制模式下的命令
        case 'F': // 前进
            if (currentMode == MODE_MANUAL_CONTROL)
                manualState = MANUAL_FORWARD;
            break;
        case 'B': // 后退
            if (currentMode == MODE_MANUAL_CONTROL)
                manualState = MANUAL_BACKWARD;
            break;
        case 'L': // 左转
            if (currentMode == MODE_MANUAL_CONTROL)
                manualState = MANUAL_LEFT;
            break;
        case 'R': // 右转
            if (currentMode == MODE_MANUAL_CONTROL)
                manualState = MANUAL_RIGHT;
            break;

        case 'l': // 小左转
            if (currentMode == MODE_MANUAL_CONTROL)
                manualState = MANUAL_LEFT_SMALL;
            break;
        case 'r': // 小右转
            if (currentMode == MODE_MANUAL_CONTROL)
                manualState = MANUAL_RIGHT_SMALL;
            break;

        case 'S': // 停止
            if (currentMode == MODE_MANUAL_CONTROL)
            {
                manualState = MANUAL_STOP;
            }
            else
            {
                currentMode = MODE_MANUAL_CONTROL;
                manualState = MANUAL_STOP;
            }
            break;
        case 'V': // 速度切换(快/慢)    //改成在任意情况下都可以调速
            manualFastSpeed = !manualFastSpeed;
            Serial.print(F("Speed changed to: "));
            Serial.println(manualFastSpeed ? "Fast" : "Slow");
            break;
        case 'X': // 舵机控制（0°-90°）
            if (currentMode == MODE_MANUAL_CONTROL)
            {
                myServo.write(0); // 转到0°
                delay(500);       // 停顿500ms
                // TODO A===【超参数】舵机开合时间+角度
                myServo.write(90); // 回到90°
                Serial.println(F("Servo moved to 0 and back to 90"));
            }
            break;
        }
    }
}

/* ========== 红外循迹功能函数 ========== */
/**
 * @brief 红外循迹模式主函数
 * 读取6路红外传感器值，根据检测到的黑线位置
 * 控制小车沿黑线行驶
 * 包含调试信息输出
 *
 */
void infraredTracking()
{

    // 读取所有红外传感器模拟值
    irValues[0] = digitalRead(IR_1);
    irValues[1] = digitalRead(IR_2);
    irValues[2] = digitalRead(IR_3);
    irValues[3] = digitalRead(IR_4);
    irValues[4] = digitalRead(IR_5);
    irValues[5] = digitalRead(IR_6);

    // 调试输出传感器值
    Serial.print("IR Values: ");
    for (int i = 0; i < 3; i++)
    {
        Serial.print(irValues[i]);
        Serial.print(" ^ ");
    }
    Serial.print(" === "); // 正中央
    for (int i = 3; i < 6; i++)
    {
        Serial.print(irValues[i]);
        Serial.print(" ^ ");
    }
    Serial.println();

    // TODO【函数】红外要单独写
}

/* ========== 雷达避障功能函数 ========== */
/**
 * @brief 雷达避障模式主函数
 * 读取三个方向(左、前、右)的障碍物距离
 * 根据障碍物距离做出避障决策
 * 包含调试信息输出
 */
void radarAvoidance()
{

    // 读取三个方向的障碍物距离
    leftDistance = readDistance(TrigLAD, leftEcho);
    frontDistance = readDistance(TrigLAD, frontEcho);
    rightDistance = readDistance(TrigLAD, rightEcho);

    // 调试输出距离信息
    Serial.print("Distances - L:");
    Serial.print(leftDistance);
    Serial.print("cm F:");
    Serial.print(frontDistance);
    Serial.print("cm R:");
    Serial.print(rightDistance);
    Serial.println("cm");

    // TODO 【函数】雷达要改
}

/* ========== 手动控制功能函数 ========== */
/**
 * @brief 手动控制模式主函数
 * 根据manualState和manualFastSpeed变量
 * 控制小车的运动和速度
 */
void manualControl()
{
    // 根据速度标志设置PWM值
    // 增加低速的速度
    // TODO  A===【超参数】 手动的高低速PWM
    int speed = manualFastSpeed ? 255 : 180;

    // 根据当前手动状态控制电机
    switch (manualState)
    {
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

    case MANUAL_LEFT_SMALL:
        turnLeftSmall(speed);
        break;
    case MANUAL_RIGHT_SMALL:
        turnRightSmall(speed);
        break;
    }
}

/* ========== 超声波距离测量函数 ========== */
/**
 * @brief 使用超声波传感器测量距离
 * @param trigPin 触发引脚
 * @param echoPin 回波引脚
 * @return float 测量到的距离（厘米），超距返回150.0
 */
float readDistance(int trigPin, int echoPin)
{
    // TODO 【函数】 返回的内容有可能因为上一次返回结果超时
    // 发送10微秒的触发脉冲
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // 测量回波脉冲宽度（添加超时时间6ms，约100cm）
    long duration = pulseIn(echoPin, HIGH, 6000);

    if (duration == 0)
    {
        return 150.0; // 测量范围是0~110+；若为150则认为无穷大
    }

    // 计算距离（声速340m/s，除以2因为是往返距离）
    float distance = duration * 0.01724; //

    return distance;
}

/* ========== 电机控制函数 ========== */
/**
 * @brief 控制小车前进
 * @param speed PWM速度值（0-255）
 */
void moveForward(int speed)
{
    digitalWrite(leftMotor1, HIGH);
    digitalWrite(leftMotor2, LOW);
    digitalWrite(rightMotor1, HIGH);
    digitalWrite(rightMotor2, LOW);
    analogWrite(leftPWM, speed);
    analogWrite(rightPWM, speed);
}

/**
 * @brief 控制小车后退
 * @param speed PWM速度值（0-255）
 */
void moveBackward(int speed)
{
    digitalWrite(leftMotor1, LOW);
    digitalWrite(leftMotor2, HIGH);
    digitalWrite(rightMotor1, LOW);
    digitalWrite(rightMotor2, HIGH);
    analogWrite(leftPWM, speed);
    analogWrite(rightPWM, speed);
}

/**
 * @brief 控制小车左转
 * @param speed PWM速度值（0-255）
 * 左侧电机反转，右侧电机正转
 */
void turnLeft(int speed)
{
    digitalWrite(leftMotor1, LOW);
    digitalWrite(leftMotor2, HIGH);
    digitalWrite(rightMotor1, HIGH);
    digitalWrite(rightMotor2, LOW);
    analogWrite(leftPWM, speed);
    analogWrite(rightPWM, speed);
}

/**
 * @brief 控制小车右转
 * @param speed PWM速度值（0-255）
 * 左侧电机正转，右侧电机反转
 */
void turnRight(int speed)
{
    digitalWrite(leftMotor1, HIGH);
    digitalWrite(leftMotor2, LOW);
    digitalWrite(rightMotor1, LOW);
    digitalWrite(rightMotor2, HIGH);
    analogWrite(leftPWM, speed);
    analogWrite(rightPWM, speed);
}

/**
 * @brief 控制小车右转但不后退
 * @param speed PWM速度值（0-255）
 * 左侧电机正转，右侧电机停
 */
void turnRightSmall(int speed)
{
    digitalWrite(leftMotor1, HIGH);
    digitalWrite(leftMotor2, LOW);
    digitalWrite(rightMotor1, LOW);
    digitalWrite(rightMotor2, LOW);
    analogWrite(leftPWM, speed);
    analogWrite(rightPWM, 0);
}

/**
 * @brief 控制小车左转但不后退
 * @param speed PWM速度值（0-255）
 * 左侧电机停，右侧电机正转
 */
void turnLeftSmall(int speed)
{
    digitalWrite(leftMotor1, LOW);
    digitalWrite(leftMotor2, LOW);
    digitalWrite(rightMotor1, HIGH);
    digitalWrite(rightMotor2, LOW);
    analogWrite(leftPWM, 0);
    analogWrite(rightPWM, speed);
}

/**
 * @brief 停止所有电机
 */
void stopMotors()
{
    digitalWrite(leftMotor1, LOW);
    digitalWrite(leftMotor2, LOW);
    digitalWrite(rightMotor1, LOW);
    digitalWrite(rightMotor2, LOW);
    analogWrite(leftPWM, 0);
    analogWrite(rightPWM, 0);
}

/* ========== 辅助函数 ========== */
/**
 * @brief 打印当前系统模式信息
 * 如果是手动模式，还会打印指令帮助信息
 */
void printCurrentMode()
{
    String modeStr;
    switch (currentMode)
    {
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
}