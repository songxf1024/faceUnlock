// https://github.com/Robotechnic/DigiKeyboardFr
#include "DigiKeyboard.h"

// https://github.com/SpenceKonde/ATTinyCore
#include "TinySoftwareSerial.h"

// 板载LED 通常在 Digispark 上是引脚 1
const int LED_PIN = 1;
// 手动创建一个 ring buffer 实例
soft_ring_buffer my_rx_buffer = { { 0 }, 0, 0 };
// 初始化串口对象：TX=PB0, RX=PB2
TinySoftwareSerial mySerial(&my_rx_buffer, 0, 2);

void setup() {
  // 串口和键盘初始化设置
  mySerial.begin(9600);
  DigiKeyboard.sendKeyStroke(0);
  
  // 设置 LED 引脚为输出
  pinMode(LED_PIN, OUTPUT);     
  // 启动提示：LED 闪烁两下
  for (int i = 0; i < 2; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }
  // 启动后 LED 保持关闭状态
  digitalWrite(LED_PIN, LOW);
  mySerial.println("Hello!");
}

void loop() {
  // 检查串口是否有输入数据
  if (mySerial.available() > 0) {
    // 读取串口的输入数据
    char inChar = mySerial.read();
    mySerial.write(inChar);
    // 用于上位机探测串口号
    if(inChar == 't') {
      digitalWrite(LED_PIN, HIGH);
      DigiKeyboard.delay(200);
      digitalWrite(LED_PIN, LOW);
    } else if(inChar == 'u') {
      digitalWrite(LED_PIN, HIGH);    // 解锁开始，点亮 LED
      DigiKeyboard.delay(200);
      DigiKeyboard.sendKeyStroke(KEY_ESC);
      DigiKeyboard.delay(1000);
      DigiKeyboard.print("administrator");
      DigiKeyboard.delay(200);
      DigiKeyboard.sendKeyStroke(KEY_TAB);
      DigiKeyboard.delay(200);
      DigiKeyboard.print("1061700625");
      DigiKeyboard.delay(200);
      DigiKeyboard.sendKeyStroke(KEY_ENTER);
      digitalWrite(LED_PIN, LOW);     // 解锁完成，关闭 LED
    }
  }
}
