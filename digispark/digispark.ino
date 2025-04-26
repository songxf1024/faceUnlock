// https://github.com/Robotechnic/DigiKeyboardFr
#include "DigiKeyboard.h"
// https://github.com/SpenceKonde/ATTinyCore
#include "TinySoftwareSerial.h"

#define KEY_ESC 41
#define KEY_TAB 43

// 板载LED 通常在 Digispark 上是引脚 1
const int LED_PIN = 1;

void LED_wink(int num) {
  // 临时改成输出模式
  pinMode(LED_PIN, OUTPUT);
  for (int i = 0; i < 2; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }
  // 改回输入模式（拉上拉）
  pinMode(LED_PIN, INPUT_PULLUP);
}

void setup() {
  // 串口和键盘初始化设置
  Serial.begin(9600);
  DigiKeyboard.sendKeyStroke(0);
  // 设置 LED 引脚为输出
  pinMode(LED_PIN, OUTPUT);  
  Serial.println("Hello");   
  // 启动提示：LED 闪烁两下
  LED_wink(2);
  // 启动后 LED 保持关闭状态
  digitalWrite(LED_PIN, LOW);
  Serial.println("World!");
  delay(500);
}

void loop() {
  // 检查串口是否有输入数据
  if (Serial.available() > 0) {
    // 读取串口的输入数据
    char inChar = Serial.read();
    Serial.write(inChar);
    // 用于上位机探测串口号
    if(inChar == 't') {
      LED_wink(1);
    } else if(inChar == 'u') {
      LED_wink(2);    // 解锁开始
      delay(200);
      DigiKeyboard.sendKeyStroke(KEY_ESC);
      DigiKeyboard.delay(1000);
      DigiKeyboard.print("administrator");
      DigiKeyboard.delay(200);
      DigiKeyboard.sendKeyStroke(KEY_TAB);
      DigiKeyboard.delay(200);
      DigiKeyboard.print("1061700625");
      DigiKeyboard.delay(200);
      DigiKeyboard.sendKeyStroke(KEY_ENTER);
      LED_wink(2);     // 解锁完成
      delay(200);
    }
  }
}
