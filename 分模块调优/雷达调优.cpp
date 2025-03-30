void radarAvoidance()
{
    // 1. 读取传感器数据
    // TODO 需要注释掉
    float leftDistance = 1;  // readDistance(leftTrig, leftEcho);
    float frontDistance = 1; // readDistance(frontTrig, frontEcho);
    float rightDistance = 1; // readDistance(rightTrig, rightEcho);

    // 2. 输出传感器数据和当前状态
    Serial.print("[传感器数据] 左:");
    Serial.print(leftDistance);
    Serial.print("cm 前:");
    Serial.print(frontDistance);
    Serial.print("cm 右:");
    Serial.print(rightDistance);
    Serial.println("cm");

    int StandardHighSpeed, StandardLowSpeed, LongTerm, ShortTerm = 1; // TODO 需要注释掉

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
        // moveForward(Radarspeed[1], Radartime_use[1]);
        // moveForward(Radarspeed[1], Radartime_use[1]);
        // moveForward(Radarspeed[1], Radartime_use[1]);
        // turnRight(Radarspeed[0], Radartime_use[0]);
        // moveForward(Radarspeed[0], Radartime_use[0]);
        // moveForward(Radarspeed[0], Radartime_use[0]);
        Serial.println("[调试]1. 【右转】当右边大于 {A探测限值} ");
    }
    else if (leftDistance > A_RADAR_LENGH)
    {
        /*
        2. 【左转】当左边大于 {A探测限值} （此时右侧条件即运行1，已经不满足）。
        则左边有通道：先向前（可以撞墙），再左转，再向前走。
        低速前进 * 3，慢速左转 * 2，高速前进 * 2；
        */
        // moveForward(Radarspeed[1], Radartime_use[1]);
        // moveForward(Radarspeed[1], Radartime_use[1]);
        // moveForward(Radarspeed[1], Radartime_use[1]);
        // turnLeft(Radarspeed[0], Radartime_use[0]);
        // moveForward(Radarspeed[0], Radartime_use[0]);
        // moveForward(Radarspeed[0], Radartime_use[0]);
        Serial.println("[调试]2. 【左转】当左边大于 {A探测限值} ");
    }
    else if (frontDistance <= B_FRONT_HOPE)
    {
        /*
        3. 【倒退】陷入困境，当前方距离小于等于 {B前进期望} 。
        则前方撞墙：倒退一定距离。
        慢速后退 * 2 大概6cm：*/
        // moveBackward(Radarspeed[1], Radartime_use[1]);
        // moveBackward(Radarspeed[1], Radartime_use[1]);
        Serial.println("[调试]3. 【倒退】陷入困境，当前方距离小于等于 {B前进期望} ");
    }
    else if (rightDistance > leftDistance)
    {
        /*
        4. 【直行前进不够右】当 右侧距离大于 左侧距离。
        则右侧不够：先右一点，再少往前一点。
        慢小右 * 1，高速前进 * 1
        */
        // turnRightSmall(Radarspeed[1], Radartime_use[1]);
        // moveForward(Radarspeed[0], Radartime_use[0]);
        Serial.println("[调试]4. 【直行前进不够右】当 右侧距离大于 左侧距离 ");
    }
    else if (rightDistance < C_RIGHT_MIN)
    {
        /*
        5. 【直行前进太靠右】当 右侧距离小于 {C贴右最小值}。
        则右侧贴的太近：先左一点，再少往前一点，再回方向。
        慢小左 * 1，高速前进 * 1 ，慢小右*1
        */
        // turnLeftSmall(Radarspeed[1], Radartime_use[1]);
        // moveForward(Radarspeed[0], Radartime_use[0]);
        // turnRightSmall(Radarspeed[1], Radartime_use[1]);
        Serial.println("[调试]5. 【直行前进太靠右】当 右侧距离小于 {C贴右最小值} ");
    }
    else
    {
        /*&
        6. 【直行前进】 可以前进。
        高速前进 * 2
        */
    //    moveForward(Radarspeed[0], Radartime_use[0]);
    //    moveForward(Radarspeed[0], Radartime_use[0]);
       Serial.println("[调试]6. 【直行前进】 可以前进");
    }
}