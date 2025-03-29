// Ardunio + L298n；使用analogWrite以控制输出电平
// in1 - in4 分别接到 5、6、9、10
// 完成高速、低速；前后左右转；每一轮3s；将当轮情况输出到终端
// 定义电机控制引脚
#define IN1 5
#define IN2 6
#define IN3 3
#define IN4 11

#include <Servo.h>  // 引入Servo库

// 定义舵机引脚
#define SERVO_PIN 10
// 创建舵机对象
Servo myServo;

// 定义舵机初始角度
int angle = 0;  // 初始角度为0度

// 定义速度等级
#define HIGH_SPEED 200 // 高速 (PWM值 0-255)
#define LOW_SPEED 80  // 低速

void setup()
{
	// 设置电机控制引脚为输出
	pinMode(IN1, OUTPUT);
	pinMode(IN2, OUTPUT);
	pinMode(IN3, OUTPUT);
	pinMode(IN4, OUTPUT);

	myServo.attach(SERVO_PIN);

	// 初始化串口通信
	Serial.begin(9600);
	Serial.println("L298N Motor Control Demo");
}

void loop()
{
	// 高速前进（两个电机正转）
	motorControl(HIGH_SPEED, HIGH_SPEED);
	Serial.println("高速前进");
	delay(3000);

	// 低速前进
	motorControl(LOW_SPEED, LOW_SPEED);
	Serial.println("低速前进");
	delay(3000);

	// 高速后退（两个电机反转）
	motorControl(-HIGH_SPEED, -HIGH_SPEED);
	Serial.println("高速后退");
	delay(3000);

	// 低速后退
	motorControl(-LOW_SPEED, -LOW_SPEED);
	Serial.println("低速后退");
	delay(3000);

	// 左转（左轮后退，右轮前进）
	motorControl(-HIGH_SPEED, HIGH_SPEED);
	Serial.println("高速左转");
	delay(3000);

	// 低速左转
	motorControl(-LOW_SPEED, LOW_SPEED);
	Serial.println("低速左转");
	delay(3000);

	// 右转（左轮前进，右轮后退）
	motorControl(HIGH_SPEED, -HIGH_SPEED);
	Serial.println("高速右转");
	delay(3000);

	// 低速右转
	motorControl(LOW_SPEED, -LOW_SPEED);
	Serial.println("低速右转");
	delay(3000);

	// 停止
	motorControl(0, 0);
	Serial.println("停止");
	delay(2000);
	delay(20000);
}

// 电机控制函数（修正版）
// 参数：左轮速度，右轮速度（范围-255~255，负值表示反转）
void motorControl(int leftSpeed, int rightSpeed)
{
	// 限制速度范围
	leftSpeed = constrain(leftSpeed, -255, 255);
	rightSpeed = constrain(rightSpeed, -255, 255);
	// constrain(x, a, b)
	/*
	x：要限制的值（可以是整数或浮点数）
	a：范围下限（最小值）
	b：范围上限（最大值）	
	*/

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