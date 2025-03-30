/*
 * 智能小车主控程序 v0.3.2
 *
 * 改进内容：
 * 1. 雷达循迹
 * 
 *
 */

// TODO 【版本信息】
#define _VERSION_ "v0.3.1"

#include <Servo.h> // 舵机控制库

/* ========== 硬件引脚定义 ========== */
// （6路红外传感器）
// 黑=1；白=0
#define IR_1 A0 // 最左侧红外传感器
#define IR_2 A1 // 左侧红外传感器
#define IR_3 A2 // 左中红外传感器
#define IR_4 A3 // 右中红外传感器
#define IR_5 A4 // 右侧红外传感器
#define IR_6 A5 // 最右侧红外传感器

// 超声波雷达引脚（3个Trig，3个Echo）
#define rightTrig 2 // 右雷达激发
#define frontTrig 7 // 前雷达激发【改接线】
#define leftTrig 4	// 左雷达激发

#define rightEcho 8	 // 右侧雷达回波引脚 1
#define frontEcho 12 // 前方雷达回波引脚 2
#define leftEcho 13	 // 左侧雷达回波引脚 3

// 定义电机控制引脚
#define IN1 5
#define IN2 6
#define IN3 3
#define IN4 11

// 舵机控制引脚
#define SERVO_PIN 10 // 舵机信号引脚

/* ========== 全局变量定义 ========== */

// 红外传感器值存储数组
int irValues[6]; // 索引0-5对应IR_1到IR_6

// 雷达距离值（单位：厘米）
float leftDistance = 0;	 // 左侧障碍物距离
float rightDistance = 0; // 右侧障碍物距离
float frontDistance = 0; // 前方障碍物距离

// 舵机控制变量
Servo myServo;		 // 舵机对象
int servoAngle = 90; // 舵机角度（范围0-180，90为中间位置）

// 系统工作模式枚举
enum SystemMode
{
	MODE_INFRARED_TRACKING, // 红外循迹模式
	MODE_RADAR_AVOIDANCE,	// 雷达避障模式
	MODE_MANUAL_CONTROL		// 手动控制模式
};
SystemMode currentMode = MODE_MANUAL_CONTROL; // 当前系统模式，默认为手动控制

// 手动控制模式下的运动状态
enum ManualState
{
	MANUAL_STOP,	   // 停止
	MANUAL_FORWARD,	   // 前进
	MANUAL_BACKWARD,   // 后退
	MANUAL_LEFT,	   // 左转
	MANUAL_RIGHT,	   // 右转
	MANUAL_LEFT_SMALL, // 小左转
	MANUAL_RIGHT_SMALL // 小右转
};
ManualState manualState = MANUAL_STOP; // 手动控制状态
bool manualSpeed = false;			   // 手动控制速度标志（f-低速短时；t-高速长时）

// TODO 【调参】手动加速部分
// 高速长时=10cm~
// 低速短时=4cm~
#define StandardLowSpeed 100  // 低速pwm
#define StandardHighSpeed 170 // 高速pwm
#define LongTerm 350		  // 长时间行走delay
#define ShortTerm 200		  // 短时间行走delay

#define DebugTime 500 // 用于调试时，每个循环进行等待

// 初始化函数
void setup()
{

	Serial.begin(9600);

	// 初始化红外传感器引脚
	pinMode(IR_1, INPUT);
	pinMode(IR_2, INPUT);
	pinMode(IR_3, INPUT);
	pinMode(IR_4, INPUT);
	pinMode(IR_5, INPUT);
	pinMode(IR_6, INPUT);

	// 初始化雷达模块引脚
	pinMode(leftTrig, OUTPUT);
	pinMode(frontTrig, OUTPUT);
	pinMode(rightTrig, OUTPUT);

	pinMode(leftEcho, INPUT);
	pinMode(rightEcho, INPUT);
	pinMode(frontEcho, INPUT);

	// 设置电机控制引脚为输出
	pinMode(IN1, OUTPUT);
	pinMode(IN2, OUTPUT);
	pinMode(IN3, OUTPUT);
	pinMode(IN4, OUTPUT);

	// 初始化舵机
	myServo.attach(SERVO_PIN);
	myServo.write(servoAngle); // 初始位置居中

	// 初始状态停止所有电机
	stopMotors(50);

	// 打印初始化信息
	Serial.print("Smart Car System Initialized ");
	Serial.println(_VERSION_);
	printCurrentMode(); // 打印当前模式信息
}

// 主循环函数
void loop()
{
	// 每次循环后，回到默认的STOP状态
	stopMotors(5);
	manualState = MANUAL_STOP;

	// 处理蓝牙命令
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
	// DEBUG 延时
	stopState();
	delay(DebugTime);
}

// 蓝牙处理
void handleBluetooth()
{
	// Serial.available() 是非阻塞的，返回缓冲区字节数
	//   若未输入，则整个函数直接跳过
	if (Serial.available())
	{
		char command = Serial.read();

		// 模式切换命令
		switch (command)
		{
		case '1': // 切换到红外循迹模式
			currentMode = MODE_INFRARED_TRACKING;
			stopMotors(100);
			printCurrentMode();
			break;
		case '2': // 切换到雷达避障模式
			currentMode = MODE_RADAR_AVOIDANCE;
			stopMotors(100);
			printCurrentMode();
			break;
		case '3': // 切换到手动控制模式
			currentMode = MODE_MANUAL_CONTROL;
			stopMotors(100);
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
		case 'V':
			manualSpeed = !manualSpeed;

			Serial.print(F("Speed changed to: "));
			if (!manualSpeed)
				Serial.println(F("False-低速短时"));
			else
				Serial.println(F("True-高速长时"));

			break;
		case 'X': // 舵机控制（0°-90°）
			if (currentMode == MODE_MANUAL_CONTROL)
			{
				myServo.write(20); // 转到0°
				delay(300);		  // 停顿500ms

				myServo.write(servoAngle); // 回到90°
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
	delay(100); // 临时小工具

	// TODO【函数=红外】
}

/* ========== 雷达避障功能函数（靠右侧行驶） ========== */
/**
 * @brief 雷达避障模式主函数
 * 读取三个方向(左、前、右)的障碍物距离
 * 根据障碍物距离做出避障决策
 * 包含调试信息输出和当前决策信息显示
 */
void radarAvoidance()
{
    // 1. 读取传感器数据
    // TODO 需要注释掉
    // float leftDistance = 1;  // readDistance(leftTrig, leftEcho);
    // float frontDistance = 1; // readDistance(frontTrig, frontEcho);
    // float rightDistance = 1; // readDistance(rightTrig, rightEcho);
	leftDistance = readDistance(leftTrig, leftEcho);
    frontDistance = readDistance(frontTrig, frontEcho);
    rightDistance =  readDistance(rightTrig, rightEcho);

    // 2. 输出传感器数据和当前状态
    Serial.print("[传感器数据] 左:");
    Serial.print(leftDistance);
    Serial.print("cm 前:");
    Serial.print(frontDistance);
    Serial.print("cm 右:");
    Serial.print(rightDistance);
    Serial.println("cm");

    //int StandardHighSpeed, StandardLowSpeed, LongTerm, ShortTerm = 1; // TODO 需要注释掉

    int Radarspeed[2] = {StandardHighSpeed, StandardLowSpeed}; // 0高速，1低速
    int Radartime_use[2] = {LongTerm, ShortTerm};              // 0高速，1低速

    const int A_RADAR_LENGH = 30; // A探测限值；大于此值则认为有通道（转弯）
    const int B_FRONT_HOPE = 12;  // B前进期望；大于此值则可向前走
    const int C_RIGHT_MIN = 8;    // C贴右最小值；小于此值则认为太靠右

    if (rightDistance > A_RADAR_LENGH)
    {
        /*
        1. 【右转】当右边大于 {A探测限值} 。
        则右边有通道：先向前（可以撞墙），再右转，再向前走。
        低速前进 * 3，高速右转 * 1，高速前进 * 2；
        */
        moveForward(Radarspeed[1], Radartime_use[1]);
        moveForward(Radarspeed[1], Radartime_use[1]);
        moveForward(Radarspeed[1], Radartime_use[1]);
        turnRight(Radarspeed[0], Radartime_use[0]);
        moveForward(Radarspeed[0], Radartime_use[0]);
        moveForward(Radarspeed[0], Radartime_use[0]);
        Serial.println("[调试]1. 【右转】当右边大于 {A探测限值} ");
    }
    else if (leftDistance > A_RADAR_LENGH)
    {
        /*
        2. 【左转】当左边大于 {A探测限值} （此时右侧条件即运行1，已经不满足）。
        则左边有通道：先向前（可以撞墙），再左转，再向前走。
        低速前进 * 3，慢速左转 * 2，高速前进 * 2；
        */
        moveForward(Radarspeed[1], Radartime_use[1]);
        moveForward(Radarspeed[1], Radartime_use[1]);
        moveForward(Radarspeed[1], Radartime_use[1]);
        turnLeft(Radarspeed[0], Radartime_use[0]);
        moveForward(Radarspeed[0], Radartime_use[0]);
        moveForward(Radarspeed[0], Radartime_use[0]);
        Serial.println("[调试]2. 【左转】当左边大于 {A探测限值} ");
    }
    else if (frontDistance <= B_FRONT_HOPE)
    {
        /*
        3. 【倒退】陷入困境，当前方距离小于等于 {B前进期望} 。
        则前方撞墙：倒退一定距离。
        慢速后退 * 2 大概6cm：*/
        moveBackward(Radarspeed[1], Radartime_use[1]);
        moveBackward(Radarspeed[1], Radartime_use[1]);
        Serial.println("[调试]3. 【倒退】陷入困境，当前方距离小于等于 {B前进期望} ");
    }
    else if (rightDistance > leftDistance)
    {
        /*
        4. 【直行前进不够右】当 右侧距离大于 左侧距离。
        则右侧不够：先右一点，再少往前一点。
        慢小右 * 1，高速前进 * 1
        */
        turnRightSmall(Radarspeed[1], Radartime_use[1]);
        moveForward(Radarspeed[0], Radartime_use[0]);
        Serial.println("[调试]4. 【直行前进不够右】当 右侧距离大于 左侧距离 ");
    }
    else if (rightDistance < C_RIGHT_MIN)
    {
        /*
        5. 【直行前进太靠右】当 右侧距离小于 {C贴右最小值}。
        则右侧贴的太近：先左一点，再少往前一点，再回方向。
        慢小左 * 1，高速前进 * 1 ，慢小右*1
        */
        turnLeftSmall(Radarspeed[1], Radartime_use[1]);
        moveForward(Radarspeed[0], Radartime_use[0]);
        turnRightSmall(Radarspeed[1], Radartime_use[1]);
        Serial.println("[调试]5. 【直行前进太靠右】当 右侧距离小于 {C贴右最小值} ");
    }
    else
    {
        /*&
        6. 【直行前进】 可以前进。
        高速前进 * 2
        */
       moveForward(Radarspeed[0], Radartime_use[0]);
       moveForward(Radarspeed[0], Radartime_use[0]);
       Serial.println("[调试]6. 【直行前进】 可以前进");
    }
}

// 手动控制功能函数
void manualControl()
{

	int speed = manualSpeed ? StandardHighSpeed : StandardLowSpeed;
	int time_use = manualSpeed ? LongTerm : ShortTerm;

	// 根据当前手动状态控制电机
	switch (manualState)
	{
	case MANUAL_STOP:
		stopMotors(time_use);
		break;
	case MANUAL_FORWARD:
		moveForward(speed, time_use);
		break;
	case MANUAL_BACKWARD:
		moveBackward(speed, time_use);
		break;
	case MANUAL_LEFT:
		turnLeft(speed, time_use);
		break;
	case MANUAL_RIGHT:
		turnRight(speed, time_use);
		break;

	case MANUAL_LEFT_SMALL:
		turnLeftSmall(speed, time_use);
		break;
	case MANUAL_RIGHT_SMALL:
		turnRightSmall(speed, time_use);
		break;
	}
}

// 超声波距离测量函数
float readDistance(int trigPin, int echoPin)
{

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
		return 150.0; // 测量范围是0~110+；若为150.0则认为无穷大
	}

	// 计算距离（声速340m/s，除以2因为是往返距离）
	float distance = duration * 0.01724;

	return distance;
}

// 前进
void moveForward(int speed, int dalay_time)
{
	motorControlState(speed, speed);
	delay(dalay_time);
	stopState();
}

// 后退
void moveBackward(int speed, int dalay_time)
{

	motorControlState(-speed, -speed);
	delay(dalay_time);
	stopState();
}

// 左转
void turnLeft(int speed, int dalay_time)
{

	motorControlState(-speed, speed);
	delay(dalay_time);
	stopState();
}

// 右转
void turnRight(int speed, int dalay_time)
{

	motorControlState(speed, -speed);
	delay(dalay_time);
	stopState();
}

// 小右转
void turnRightSmall(int speed, int dalay_time)
{
	speed = speed * 0.8;
	motorControlState(speed, 0);
	delay(dalay_time);
	stopState();
}

// 小左转
void turnLeftSmall(int speed, int dalay_time)
{
	speed = speed * 0.8;
	motorControlState(0, speed);
	delay(dalay_time);
	stopState();
}

// 停止电机并延时
void stopMotors(int dalay_time)
{
	stopState();
	delay(dalay_time);
	stopState();
}

// 停止电机【状态】
void stopState()
{
	motorControlState(0, 0);
}

// 电机控制状态函数
void motorControlState(int leftSpeed, int rightSpeed)
{
	// 限制速度范围
	leftSpeed = constrain(leftSpeed, -255, 255);
	rightSpeed = constrain(rightSpeed, -255, 255);

	// 控制左轮（IN1/IN2）
	if (leftSpeed > 0)
	{
		analogWrite(IN1, leftSpeed);
		analogWrite(IN2, 0);
	}
	else if (leftSpeed < 0)
	{
		analogWrite(IN1, 0);
		analogWrite(IN2, -leftSpeed);
	}
	else
	{
		digitalWrite(IN1, LOW);
		digitalWrite(IN2, LOW);
	}

	// 控制右轮（IN3/IN4）
	if (rightSpeed > 0)
	{
		analogWrite(IN3, rightSpeed);
		analogWrite(IN4, 0);
	}
	else if (rightSpeed < 0)
	{
		analogWrite(IN3, 0);
		analogWrite(IN4, -rightSpeed);
	}
	else
	{
		digitalWrite(IN3, LOW);
		digitalWrite(IN4, LOW);
	}
}

// 打印模式
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