
void setup() {
  // 初始化硬件串口（用于与PC通信）
  Serial.begin(9600);
  
  // 初始化软件串口（用于与蓝牙模块通信）
  BTSerial.begin(9600);
  
  Serial.println("Bluetooth Test Started");
}

void loop() {
  // 如果从蓝牙模块接收到数据，则将其打印到串口监视器
  if (BTSerial.available()) {
    char receivedChar = BTSerial.read();
    Serial.print("Received: ");
    Serial.println(receivedChar);
  }

  // 如果从串口监视器接收到数据，则将其发送到蓝牙模块
  if (Serial.available()) {
    char sendChar = Serial.read();
    BTSerial.write(sendChar);
    Serial.print("Sent: ");
    Serial.println(sendChar);
  }
}