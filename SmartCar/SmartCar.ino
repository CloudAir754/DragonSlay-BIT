/*
 * 智能小车主控程序 v1.0.0
 * 该程序用于北理工屠龙大赛
 * 可做参考，建议大改。以避免赛方查重（拱手.jpg）
 * 🚗💨【祝你成功！】
 */

// 【版本信息】（由于在终端输出信息，便于确认自己是否烧对了版本）
#define _VERSION_ "v1.0.0 "

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
#define frontTrig 7 // 前雷达激发
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
int servoAngle = 97; // 舵机角度（范围0-180，90为中间位置）
int open_servo = 76; // 舵机开启角度
int open_time = 70 ;// 舵机开启时间

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

// 手动加速部分
//【该计数仅供参考，实际工作中，受电池电压、轮胎磨损程度、车重、重心分布影响】
// 高速长时=10cm~
// 低速短时=4cm~
#define StandardLowSpeed 100  // 低速pwm
#define StandardHighSpeed 170 // 高速pwm
#define LongTerm 350		  // 长时间行走delay
#define ShortTerm 200		  // 短时间行走delay

#define DebugTime 20 // 用于调试时，每个循环进行等待 20 700
//（700ms延时，用于看终端汇报的情况，给人脑以debug方向）

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
	myServo.write(servoAngle); // 初始位置关

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
		 // 这个可以去掉，主要用于debug用【你无法现象，蓝牙在连接后会发送一串确认消息，参赛前我的蓝牙在我发送一个信息后就卡死闪退……后面好不容易弄好……】
		Serial.print("你输入了：");
		Serial.println(command);
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
		case 'X': // 舵机控制
				  
			if (currentMode == MODE_MANUAL_CONTROL)
			{
				myServo.write(open_servo); // 转到开
				delay(open_time);		   // 停顿
				myServo.write(servoAngle); // 回到关
				Serial.println(F("biu~"));
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

	// 定义速度参数
	//  速度参数得调小；也要考虑过坡的动力要求
	// 回复上一行，本队在实际比赛中，战略性放弃了爬坡需求；红外太低了会卡坡，改结构会导致很多额外的问题
	//  较大的基础和较小的pid持续时间，会让你的新能源车有一种单杠内燃机车的感觉（人话：像拖拉机，突突突——突突；不过很稳）
	const int baseSpeed = 200; // 基础速度
	const int maxSpeed = 250;  // 最大速度
	const int pidTime = 20;	   // PID一次进行的时间

	const int turnSpeed = 150; // 转弯速度(直角弯）
	const int turnTime = 150;

	// 定义传感器权重
	const double weights[6] = {-1.2, -0.8, -0.4, 0.4, 0.8, 1.2}; // 各传感器的权重值

	// 计算偏差值
	double error = 0;

	static double lastError = 0;
	static double integral = 0;

	for (int i = 0; i < 6; i++)
	{
		if (irValues[i]) // 检测到黑线
		{
			error += weights[i];
		}
	}

	if (irValues[0] && irValues[1])
	{
		// 检测到最左侧两个传感器，可能是直角或锐角左转
		Serial.println("Sharp left turn detected");
		turnLeft(turnSpeed, turnTime);

		lastError = 0;
		integral = 0;
	}
	else if (irValues[5] && irValues[4])
	{
		// 检测到右侧传感器，可能是直角或锐角右转
		Serial.println("Sharp right turn detected");
		turnRight(turnSpeed, turnTime);

		lastError = 0;
		integral = 0;
	}
	else
	{
		// 正常循迹情况，使用PID控制
		float kp = 0.85; // 比例系数；响应当前误差，过高导致振荡
		float ki = 0.1;	 // 积分系数；累计历史误差，调高可以避免偏向一侧
		float kd = 0.05; // 微分系数；误差变化率，增大会减小超调变笨拙

		// 计算PID值
		integral += error;
		double derivative = error - lastError;
		double correction = kp * error + ki * integral + kd * derivative;
		lastError = error;

		// 应用修正
		int leftSpeed = baseSpeed * (1 + correction);
		int rightSpeed = baseSpeed * (1 - correction);

		// 限制速度范围
		leftSpeed = constrain(leftSpeed, 0, maxSpeed);
		rightSpeed = constrain(rightSpeed, 0, maxSpeed);

		// 控制电机
		Serial.print("PID control - L: ");
		Serial.print(leftSpeed);
		Serial.print(" R: ");
		Serial.println(rightSpeed);

		stopState();
		motorControlState(leftSpeed, rightSpeed);
		delay(pidTime);
		stopState();
	}

	delay(10); // 控制循环频率
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

	leftDistance = readDistance(leftTrig, leftEcho);
	frontDistance = readDistance(frontTrig, frontEcho);
	rightDistance = readDistance(rightTrig, rightEcho);

	// 2. 输出传感器数据和当前状态
	Serial.print("[雷达] 左:");
	Serial.print(leftDistance);
	Serial.print("cm 前:");
	Serial.print(frontDistance);
	Serial.print("cm 右:");
	Serial.print(rightDistance);
	Serial.println("cm");

	int Radarspeed[2] = {StandardHighSpeed, StandardLowSpeed}; // 0高速，1低速
	int Radartime_use[2] = {LongTerm, ShortTerm};			   // 0高速，1低速

	// 【调参】雷达调优看这里
	// 这些超参数，能保证测试赛道正常走就行，不过可以考虑一些特殊情况
	//		毕竟，比赛赛道往往和你的想象由很大差距(科协的小伙伴的任何一个灵机一动，都会让你调的无比牛的“先验”参数，变成过拟合~)
	const int A_RADAR_LENGH = 50; // A探测限值；大于此值则认为有通道（转弯）
	const int B_FRONT_HOPE = 12;  // B前进期望；大于此值则可向前走
	const int C_Not_Center = 23;  // C；居中时，左右大概时17

	// 建议在车身上增加防撞设施，避免雷达在车子紧贴墙壁时，雷达传回数据为0或极大；进而导致以下的决策过程出现意外
	if ((frontDistance <= B_FRONT_HOPE) )
	{
		/*
		3. 【倒退】陷入困境，当前方距离小于等于 {B前进期望} 。
		则前方撞墙：倒退一定距离。
		慢速后退 * 2 大概6cm：*/
		stopMotors(100);
		moveBackward(Radarspeed[0], Radartime_use[0]);
		moveBackward(Radarspeed[1], Radartime_use[1]);
		//moveBackward(Radarspeed[1], Radartime_use[1]);
		Serial.println("[调试]3. 【倒退】陷入困境，当前方距离小于等于 {B前进期望} ");
	}
	else if (rightDistance > A_RADAR_LENGH)
	{
		/*
		1. 【右转】当右边大于 {A探测限值} 。
		则右边有通道：先向前（可以撞墙），再右转，再向前走。

		*/

		moveForward(Radarspeed[0], Radartime_use[0]);
		turnRightSmall(Radarspeed[0], Radartime_use[0]);
		moveForward(Radarspeed[0], Radartime_use[0]);
		turnRightSmall(Radarspeed[1], Radartime_use[1]);
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

		moveForward(Radarspeed[0], Radartime_use[0]);
		turnLeftSmall(Radarspeed[0], Radartime_use[0]);
		moveForward(Radarspeed[0], Radartime_use[0]);
		turnLeftSmall(Radarspeed[1], Radartime_use[1]);
		moveForward(Radarspeed[0], Radartime_use[0]);

		Serial.println("[调试]2. 【左转】当左边大于 {A探测限值} ");
	}
	else
	{
		if (leftDistance > C_Not_Center)
		{
			// 过于偏右
			moveBackward(Radarspeed[1], Radartime_use[1]);
			stopState();
			delay(100);
			turnLeftSmall(Radarspeed[1], Radartime_use[1]);
			turnLeftSmall(Radarspeed[1], Radartime_use[1]);
			moveForward(Radarspeed[1], Radartime_use[1]);
			turnRightSmall(Radarspeed[1], Radartime_use[1]);
			moveForward(Radarspeed[1], Radartime_use[1]);

			Serial.println("过于右偏");
		}
		else if (rightDistance > C_Not_Center)
		{
			// 过于偏左
			moveBackward(Radarspeed[1], Radartime_use[1]);

			stopState();
			delay(100);
			turnRightSmall(Radarspeed[1], Radartime_use[1]);

			turnRightSmall(Radarspeed[1], Radartime_use[1]);

			moveForward(Radarspeed[1], Radartime_use[1]);

			turnLeftSmall(Radarspeed[1], Radartime_use[1]);

			moveForward(Radarspeed[1], Radartime_use[1]);

			Serial.println("过于左偏");
		}

		if (leftDistance < rightDistance)
		{
			// 已经靠左偏

			turnRightSmall(Radarspeed[1], Radartime_use[1]);

			moveForward(Radarspeed[1], Radartime_use[1]);

			turnLeftSmall(Radarspeed[1], Radartime_use[1]);

			moveForward(Radarspeed[1], Radartime_use[1]);

			Serial.println("已经靠左偏");
		}
		else
		{
			// 已经靠右偏

			turnLeftSmall(Radarspeed[1], Radartime_use[1]);

			moveForward(Radarspeed[1], Radartime_use[1]);

			turnRightSmall(Radarspeed[1], Radartime_use[1]);

			moveForward(Radarspeed[1], Radartime_use[1]);

			Serial.println("已经靠右偏");
		}

		delay(100);
	}
	delay(20);
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

	// 测量回波脉冲宽度（添加超时时间10ms，约178cm）
	long duration = pulseIn(echoPin, HIGH, 10000);

	if (duration == 0)
	{
		return 200.0; // 测量范围是0~110+；若为200.0则认为无穷大
	}

	// 计算距离（声速340m/s，除以2因为是往返距离）
	float distance = duration * 0.01724;

	return distance;
}

// 前进
void moveForward(int speed, int delay_time)
{
	motorControlState(speed, speed);
	delay(delay_time);
	stopState();
}

// 后退
void moveBackward(int speed, int delay_time)
{

	motorControlState(-speed, -speed);
	delay(delay_time);
	stopState();
}

// 左转
void turnLeft(int speed, int delay_time)
{

	motorControlState(-speed, speed);
	delay(delay_time);
	stopState();
}

// 右转
void turnRight(int speed, int delay_time)
{

	motorControlState(speed, -speed);
	delay(delay_time);
	stopState();
}

// 小右转
void turnRightSmall(int speed, int delay_time)
{
	speed = speed * 0.8;
	motorControlState(speed, 0);
	delay(delay_time);
	stopState();
}

// 小左转
void turnLeftSmall(int speed, int delay_time)
{
	speed = speed * 0.8;
	motorControlState(0, speed);
	delay(delay_time);
	stopState();
}

// 停止电机并延时
void stopMotors(int delay_time)
{
	stopState();
	delay(delay_time);
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