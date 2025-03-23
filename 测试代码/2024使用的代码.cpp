// 所有方向描述以车内为准

int ProgramUse = 0; // 模式
/* 模式切换；0停止，1循迹，2避障，3屠龙*/
int InDe[6]; // 红外返回值
/*单个红外+红外整体，返回值到全局数组*/
#define TragLAD 3  // 雷达发射引脚
#define leftLAD 7  // 信号返回引脚
#define rightLAD 8 // 信号返回引脚
#define frontLAD 4 // 信号返回引脚
int RadDe[3];	   // 雷达返回值，左中右
/*雷达，发射一次，接收一次信号；
	过短的时间间隔会导致返回的时间值受感染*/
#include <Servo.h> //舵机模块
Servo myservo;	   // 舵机模块
int pos = 0;	   // 舵机初始值
/*舵机部分，直接设置指定值*/
#define STOP 0
#define FORWARD 1
#define BACKWARD 2
#define TURNLEFT 3
#define TURNRIGHT 4
#define CHANGESPEED 5

int leftMotor1 = 12;  // 左侧方向端口1===|
int leftMotor2 = 13;  // 左侧方向端口2
int rightMotor1 = 10; // 右侧方向端口1
int rightMotor2 = 11; // 右侧方向端口2
int leftPWM = 5;	  // 左侧pwm端口
int rightPWM = 6;	  // 左侧pwm端口=====|

int SpeedMode = 0;			   // 速度档位设置,对应pwm和delay
int OPERTATE_WAIT = 140;	   // 设置一个操作指令输入后的生效时间
int WAIT_TIMES[3] = {2, 3, 2}; // 手动操作的直行倍数为3，巡线模式应当为1
int TURNOP_WAIT = 60;		   // 设置转弯操作生效时间

double PWM[3] = {75, 85, 90};		  // 基本上都是【0】
double leftGain[3] = {0.8, 0.8, 0.8}; //>1,右转;;;只测试了档位0的增益
/*马达部分*/

void RadRun()
{
	RadarDetection();
	// RadDe[0] 左中右
	int leftDistance = RadDe[0];
	int frontDistance = RadDe[1];

	// 避障策略
	if (leftDistance > 13)
	{
		// 左侧距离足够远，可以安全前进并稍微左转

		TURNLEFTSTATE();
		for (int i = 0; i < 2; i++)
		{
			FORWARDSTATE();
		}
	}
	else if (leftDistance < 13 && frontDistance > 8)
	{
		// 左侧较近但前方足够远，可以安全前进
		for (int i = 0; i < 2; i++)
		{
			FORWARDSTATE();
		}
	}
	else

	{
		TURNRIGHTSTATE();
		// 左侧较近且前方也较近，应该右转以避免障碍物
		for (int i = 0; i < 3; i++)
		{

			FORWARDSTATE();
		}
	}
	STOPSTATE();
}

void FollowLine()
{
	delay(100);
	InfraredDetection();
	// Serial.println("红外数据输出如下，自车辆左侧到右侧");
	// InfraredDetection();
	// for (int i = 0; i < 6; i++)
	// {
	// 	Serial.print(InDe[i]);
	// 	Serial.print(" ");
	// }
	// Serial.println("");

	// 0是最右边
	if ((InDe[0] && InDe[1] && InDe[4]) || (InDe[0] && InDe[1] && InDe[5]) ||
		(InDe[0] && InDe[4] && InDe[5]) || (InDe[1] && InDe[4] && InDe[5])) // 2nd
	{

		Serial.println("停止");
		STOPSTATE();
	}
	else if ((InDe[2] && InDe[3])) // 即将t字
	{
		Serial.println("慢前进");
		FORWARDSTATE();
	}
	else if (InDe[3])//左转微调
	{
		Serial.println("左微调");
		//for (int i = 0; i < 2; i++)
		//{
			TURNLEFTSTATE_BACK();
			return;
	}
	else if (InDe[2])//右转微调
	{
		Serial.println("右微调");
		//for (int i = 0; i < 2; i++)
		//{
			TURNRIGHTSTATE_BACK();
			return;
	}

	else if (((InDe[1]) || (InDe[2])) && (!(InDe[0] || InDe[4] || InDe[5])))
	{
		Serial.println("小右 转");
		for (int i = 0; i < 3; i++)
		{
			TURNRIGHTSTATE_BACK();
		}
		//FORWARDSTATE();
		
	}
	else if (InDe[0])
	{
		Serial.println("右急转弯");
		for (int i = 0; i < 6; i++)
		{
			TURNRIGHTSTATE_BACK();
		}
		// BACKWARDSTATE();
	}

	else if (((InDe[4]) || (InDe[3])) && (!(InDe[0] || InDe[1] || InDe[2])))
	{
		Serial.println("小左转");
		for (int i = 0; i < 2; i++)
		{
			TURNLEFTSTATE_BACK();
		}
		//FORWARDSTATE();
	}
	else if (InDe[5])
	{
		Serial.println("左急转弯");
		for (int i = 0; i < 5; i++)
		{
			TURNLEFTSTATE_BACK();
		}
		// BACKWARDSTATE();
	}

	else if (!(InDe[0] || InDe[1] || InDe[4] || InDe[5])) // firstStatus
	{
		Serial.println("直走");

		FORWARDSTATE();
		//delay(100);
	}
	STOPSTATE();
	delay(100);
	// 做一个延时
	STOPSTATE();
}

void STOPSTATE()
{
	// Serial.println("STOP"); // 输出状态
	analogWrite(leftPWM, 0);
	analogWrite(rightPWM, 0); // pwm==0
	digitalWrite(leftMotor1, LOW);
	digitalWrite(leftMotor2, LOW);
	digitalWrite(rightMotor1, LOW);
	digitalWrite(rightMotor2, LOW);
}

void TURNRIGHTSTATE_BACK()
{
	// Serial.println("TURN  RIGHT"); // 输出状态
	analogWrite(leftPWM, PWM[SpeedMode]);
	analogWrite(rightPWM, PWM[SpeedMode]); // pwm_set
	digitalWrite(leftMotor1, LOW);
	digitalWrite(leftMotor2, HIGH);
	digitalWrite(rightMotor1, HIGH);
	digitalWrite(rightMotor2, LOW);
	delay(TURNOP_WAIT);
	STOPSTATE();
}

void TURNLEFTSTATE_BACK()
{
	// Serial.println("TURN  LEFT"); // 输出状态
	analogWrite(leftPWM, PWM[SpeedMode]);
	analogWrite(rightPWM, PWM[SpeedMode]); // pwm_set
	digitalWrite(leftMotor1, HIGH);
	digitalWrite(leftMotor2, LOW);
	digitalWrite(rightMotor1, LOW);
	digitalWrite(rightMotor2, HIGH);
	delay(TURNOP_WAIT);
	STOPSTATE();
}

void TURNRIGHTSTATE()
{
	// Serial.println("TURN  RIGHT"); // 输出状态
	analogWrite(leftPWM, PWM[SpeedMode]);
	analogWrite(rightPWM, PWM[SpeedMode]); // pwm_set
	digitalWrite(leftMotor1, LOW);
	digitalWrite(leftMotor2, HIGH);
	digitalWrite(rightMotor1, LOW);
	digitalWrite(rightMotor2, LOW);
	delay(TURNOP_WAIT);
	STOPSTATE();
}

void TURNLEFTSTATE()
{
	// Serial.println("TURN  LEFT"); // 输出状态
	analogWrite(leftPWM, PWM[SpeedMode]);
	analogWrite(rightPWM, PWM[SpeedMode]); // pwm_set
	digitalWrite(leftMotor1, LOW);
	digitalWrite(leftMotor2, LOW);
	digitalWrite(rightMotor1, LOW);
	digitalWrite(rightMotor2, HIGH);
	delay(TURNOP_WAIT);
	STOPSTATE();
}

void BACKWARDSTATE()
{

	// Serial.println("BACKWARD"); // 输出状态
	analogWrite(leftPWM, PWM[SpeedMode]);
	analogWrite(rightPWM, PWM[SpeedMode]); // pwm_set
	digitalWrite(leftMotor1, LOW);
	digitalWrite(leftMotor2, HIGH);
	digitalWrite(rightMotor1, LOW);
	digitalWrite(rightMotor2, HIGH);

	delay(OPERTATE_WAIT * WAIT_TIMES[SpeedMode]);
	STOPSTATE();
}

void FORWARDSTATE()
{
	// Serial.println("FORWARD"); // 输出状态
	analogWrite(leftPWM, PWM[SpeedMode]);
	analogWrite(rightPWM, PWM[SpeedMode] * leftGain[SpeedMode]); // pwm_set;;只有直行需要增益
	digitalWrite(leftMotor1, HIGH);
	digitalWrite(leftMotor2, LOW);
	digitalWrite(rightMotor1, HIGH);
	digitalWrite(rightMotor2, LOW);
	delay(OPERTATE_WAIT * WAIT_TIMES[SpeedMode]);
	STOPSTATE();
}

// 自主更改速度模式
void SpeedChange()
{
	if (SpeedMode != 2)
	{
		SpeedMode++;
	}
	else
	{
		SpeedMode = 0;
	}
	Serial.print("SpeedMode has been set: ");
	Serial.println(SpeedMode);
}

void ProgramChange()
{
	ProgramUse++;
	switch (ProgramUse)
	{
	case 6:
	{
		ProgramUse = 0;
		// 回归停止
		Serial.println("进入模式 0 ，全局暂停");
		STOPSTATE();
	}
	break;
	case 1:
	{
		// 巡线模式
		Serial.println("进入模式 1 ，巡线模式");
		SpeedMode = 0;
	}
	break;
	case 3:
	{
		// 雷达模式
		Serial.println("进入模式 2 ，雷达模式");
		SpeedMode = 0;
	}
	break;
	case 5:
	{
		// 屠龙模式
		Serial.println("进入模式 3 ，屠龙模式");
		SpeedMode = 1;
	}
	break;
	case 2:
	{
		Serial.println("中间暂停模式");
	}
	case 4:
	{
		Serial.println("中间暂停模式");
	}
	}
}

// 发射函数
void BulletPart()
{
	Serial.println("发射！");
	myservo.write(90);
	delay(150); // 这个延时也许刚刚好
	myservo.write(0);
}

// 雷达发射信号模块,时长10ms
void trag()
{
	digitalWrite(TragLAD, LOW);
	delay(10); // 这个值可以依据实际情况做改良
	delayMicroseconds(2);
	digitalWrite(TragLAD, HIGH);
	delayMicroseconds(10);
	digitalWrite(TragLAD, LOW);
	// 发送10us的高电平
}

// 雷达监测，一次大概30ms
void RadarDetection()
{
	// 左中右
	trag();
	RadDe[0] = pulseIn(rightLAD, HIGH) / 58;
	trag();
	RadDe[1] = pulseIn(frontLAD, HIGH) / 58;
	trag();
	RadDe[2] = pulseIn(leftLAD, HIGH) / 58;
	return;
	// 到这里大概30ms
}

/*
红外模块，返回6个端口的数据到数模转换接口
从左至右线的蓝色为棕色到蓝色（绿色到黑色）==>A5到A0
*/
// 单个的红外探测
int InDetecEvery(int a)
{
	int val = analogRead(a);
	if (val > 512)
	// 1 代表高电平，灯灭，在黑线上，无回光
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

// 红外组合
void InfraredDetection()
{

	for (int i = 0; i < 6; i++)
	{
		InDe[i] = InDetecEvery(i);
	}
}

void showAllData()
{
	// 雷达数据+红外数据==1秒一次
	// 红外输出
	Serial.println("红外数据输出如下，自车辆左侧到右侧");
	InfraredDetection();
	for (int i = 0; i < 6; i++)
	{
		Serial.print(InDe[i]);
		Serial.print(" ");
	}
	Serial.println("");
	Serial.println("雷达数据如下，自车辆左中右");
	RadarDetection(); // 约30ms
	for (int i = 0; i < 3; i++)
	{
		Serial.print(RadDe[i]);
		Serial.print(" ");
	}
	Serial.println("");
	// 算上输出的各种损耗，按照40ms算，
	// 每1000ms刷新一次，啊，其实直接延时1000比较合适
	delay(1000);
}

void setup()
{
	ProgramUse = 0;				  // 默认停止，但监听
	Serial.begin(9600);			  // 串口波特率
	pinMode(TragLAD, OUTPUT);	  // 雷达模块定义====|
	pinMode(leftLAD, INPUT);	  // 雷达模块定义	==|
	pinMode(rightLAD, INPUT);	  // 雷达模块定义	==|
	pinMode(frontLAD, INPUT);	  // 雷达模块定义====|
	myservo.attach(9);			  // 舵机口设定
	myservo.write(0);			  // 舵机，初始化到0
	pinMode(leftMotor1, OUTPUT);  // 初始化引脚====|
	pinMode(leftMotor2, OUTPUT);  // 初始化引脚
	pinMode(rightMotor1, OUTPUT); // 初始化引脚
	pinMode(rightMotor2, OUTPUT); // 初始化引脚
	pinMode(leftPWM, OUTPUT);	  // 初始化引脚
	pinMode(rightPWM, OUTPUT);	  // 初始化引脚====|
	SpeedMode = 0;				  // 初始化速度档位为最慢
}

void loop()
{

	if (Serial.available() > 0)
	{
		int cmd = Serial.read();
		// 读取蓝牙模块发送到串口的数据
		if (cmd != 10)
		// 10代表换行符
		{
			Serial.print(cmd - 48);
			int tem = cmd - 48; // 这是输入的数字值
			switch (tem)
			{
			case STOP:
			{
				STOPSTATE();
				break;
			}

			case FORWARD:
			{
				FORWARDSTATE();
				break;
			}

			case BACKWARD:
			{
				BACKWARDSTATE();
				break;
			}
			case TURNLEFT:
			{
				TURNLEFTSTATE();
				break;
			}
			case TURNRIGHT:
			{
				TURNRIGHTSTATE();
				break;
			}

			case CHANGESPEED:
			{
				SpeedChange();
				break;
			}

			case 6:
			{
				// 模式调节
				ProgramChange();
				break;
			}
			// case 7:
			// 	// 测试口
			// 	{
			// 		Serial.println("循迹模块测试口，测试100次");
			// 		for (int i = 0; i < 100; i++)
			// 		{
			// 			FollowLine();
			// 			delay(300);
			// 		}
			// 		break;
			// 	}
			// case 8:
			// 	// 输出测试数据
			// 	{
			// 		for (int i = 0; i < 100; i++)
			// 		{
			// 			showAllData();
			// 		}

			// 		break;
			// 	}
			case 9:
				// 发射
				{

					BulletPart();

					break;
				}
			}
		}
	}
	// 以下为零输入状态
	else
	{

		switch (ProgramUse)
		{
		case 0:
		{
			// 静止等待状态
			Serial.println("未输入指令，等待状态");
			STOPSTATE();
			delay(1000); // 1s后再读取
			break;
		}
		case 2:
		{
			// 静止等待状态
			Serial.println("未输入指令，等待状态");
			STOPSTATE();
			delay(1000); // 1s后再读取
			break;
		}
		case 4:
		{
			// 静止等待状态
			Serial.println("未输入指令，等待状态");
			STOPSTATE();
			delay(1000); // 1s后再读取
			break;
		}

		case 1:
		{
			// 巡线模式
			// Serial.println("未输入指令，巡线状态");
			FollowLine();
			break;
		}
		case 3:
		{
			// Serial.println("未输入指令，雷达状态");
			RadRun();
			break;
		}
		case 5:
		{
			// 手动操作
			break;
		}
		}
	}
}