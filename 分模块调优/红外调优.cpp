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
    const int baseSpeed = 150;       // 基础速度
    const int maxSpeed = 200;        // 最大速度
    const int correctionSpeed = 120; // 修正速度
    const int turnSpeed = 180;       // 转弯速度

    // 定义传感器权重
    const int weights[6] = {-30, -15, -5, 5, 15, 30}; // 各传感器的权重值

    // 计算偏差值
    int error = 0;
    int activeSensors = 0;
    
    for (int i = 0; i < 6; i++)
    {
        if (irValues[i] == 1) // 检测到黑线
        {
            error += weights[i];
            activeSensors++;
        }
    }

    // 处理不同情况
    if (activeSensors == 0)
    {
        // 没有检测到任何黑线，保持上次动作或停止
        Serial.println("No line detected - stopping");
        stopMotors();
        delay(50);
        return;
    }
    else if (irValues[0] == 1 && irValues[5] == 1)
    {
        // 同时检测到最左和最右传感器，可能是十字路口
        Serial.println("Crossroad detected - going straight");
        moveForward(baseSpeed);
        delay(100);
    }
    else if (irValues[0] == 1 || (irValues[1] == 1 && irValues[2] == 1))
    {
        // 检测到左侧传感器，可能是直角或锐角左转
        Serial.println("Sharp left turn detected");
        turnLeft(turnSpeed);
        delay(100);
    }
    else if (irValues[5] == 1 || (irValues[4] == 1 && irValues[3] == 1))
    {
        // 检测到右侧传感器，可能是直角或锐角右转
        Serial.println("Sharp right turn detected");
        turnRight(turnSpeed);
        delay(100);
    }
    else
    {
        // 正常循迹情况，使用PID控制
        float kp = 0.8; // 比例系数
        float ki = 0.0; // 积分系数
        float kd = 0.2; // 微分系数
        
        static int lastError = 0;
        static int integral = 0;
        
        // 计算PID值
        integral += error;
        int derivative = error - lastError;
        int correction = kp * error + ki * integral + kd * derivative;
        lastError = error;
        
        // 应用修正
        int leftSpeed = baseSpeed - correction;
        int rightSpeed = baseSpeed + correction;
        
        // 限制速度范围
        leftSpeed = constrain(leftSpeed, 0, maxSpeed);
        rightSpeed = constrain(rightSpeed, 0, maxSpeed);
        
        // 控制电机
        Serial.print("PID control - L: ");
        Serial.print(leftSpeed);
        Serial.print(" R: ");
        Serial.println(rightSpeed);
        
        motorControlState(leftSpeed, rightSpeed);
    }
    
    delay(30); // 控制循环频率
}