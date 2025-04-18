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
    // TODO 速度参数得调小；也要考虑过坡的动力要求
    const int baseSpeed = 150; // 基础速度
    const int baseTime = 150;  // 基础时间
    const int maxSpeed = 200;  // 最大速度
    const int pidTime = 120;   // PID一次进行的时间

    const int correctionSpeed = 120; // 修正速度(20~30°)
    const int correctionTime = 120;  // 修正时长

    const int turnSpeed = 180; // 转弯速度(直角弯）
    const int turnTime = 180;

    // 定义传感器权重
    const int weights[6] = {-20, -10, -5, 5, 10, 20}; // 各传感器的权重值

    // 计算偏差值
    int error = 0;
    int activeSensors = 0;

    static int lastError = 0;
    static int integral = 0;

    for (int i = 0; i < 6; i++)
    {
        if (irValues[i] == 1) // 检测到黑线
        {
            error += weights[i];
            activeSensors++;
        }
    }

    // 状态机控制变量（新增部分）
    static int searchState = 0; // 0: 未搜索 1: 左转 2: 右转 3: 后退

    // 检测到黑线时重置搜索状态（新增关键逻辑）
    if (activeSensors > 0)
    {
        searchState = 0;
    }

    // 处理不同情况
    if (activeSensors == 0)
    {
        //  没有检测到任何黑线，左右摆，一波测试效果之后，再后退
        if (searchState == 0)
        {
            // 开始搜索序列
            searchState = 1;
            Serial.println("No line - Searching left");
            turnLeft(correctionSpeed, correctionTime);
        }
        else
        {

            switch (searchState)
            {
            case 1:

                searchState = 2;

                Serial.println("No line - Searching right");
                turnRight(correctionSpeed, correctionTime); // 回正
                turnRight(correctionSpeed, correctionTime); // 右转

                break;

            case 2: // 后退阶段
                searchState = 3;
                Serial.println("No line - Moving backward");
                turnLeft(correctionSpeed, correctionTime);     // 回正
                moveBackward(correctionSpeed, correctionTime); // 后退第一次
                break;

            case 3: // 再后退阶段
                searchState = 0;
                Serial.println("No line - Moving backward");
                moveBackward(correctionSpeed, correctionTime); // 后退第二次
                break;
            }
        }
        lastError = 0;
        integral = 0;
        dalay(100);
        Serial.println("完成一次意外调整");
        return; // 退出函数，不执行后续循迹逻辑
    }

    if (irValues[0] == 1 || (irValues[1] == 1 && irValues[2] == 1))
    {
        // 检测到左侧传感器，可能是直角或锐角左转
        Serial.println("Sharp left turn detected");
        turnLeft(turnSpeed, turnTime);
        lastError = 0;
        integral = 0;
    }
    else if (irValues[5] == 1 || (irValues[4] == 1 && irValues[3] == 1))
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
        float kp = 0.6; // 比例系数；响应当前误差，过高导致振荡
        float ki = 0.1; // 积分系数；累计历史误差，调高可以避免偏向一侧
        float kd = 0.3; // 微分系数；误差变化率，增大会减小超调变笨拙



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

        stopState();
        motorControlState(leftSpeed, rightSpeed);
        delay(pidTime);
        stopState();
    }

    delay(30); // 控制循环频率
}