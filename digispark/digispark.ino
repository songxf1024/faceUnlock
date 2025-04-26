// https://github.com/Robotechnic/DigiKeyboardFr
#include "DigiKeyboard.h"
// https://github.com/SpenceKonde/ATTinyCore
#include "TinySoftwareSerial.h"

#define KEY_ESC 41
#define KEY_TAB 43

// 板载LED 通常在 Digispark 上是引脚 1
const int LED_PIN = 1;
char inputBuffer[32];  // 最多缓存32字节指令
uint8_t inputPos = 0;  // 当前缓存写入位置

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
  Serial.println("Hello faceUnlock!");   
  // 启动提示：LED 闪烁两下
  LED_wink(2);
}

void processCommand(const char* cmd) {
  Serial.println(cmd);
    // 用于上位机探测串口号
    if (strcmp(cmd, "test") == 0) {
      LED_wink(1);
    } else if (strcmp(cmd, "unlock") == 0) {
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

void loop() {
  // 检查串口是否有输入数据
  if (Serial.available() > 0) {
    char inChar = Serial.read();
    // 过滤非ASCII可打印字符，允许 '\n' '\r'
    if (!((inChar >= 0x20 && inChar <= 0x7E) || (inChar == '\n') || (inChar == '\r'))) {
      return;
    }
    // 如果是回车换行，就认为一条指令接收完了
    else if (inChar == '\n' || inChar == '\r') {
      // 结尾补零，形成C风格字符串
      inputBuffer[inputPos] = '\0'; 
      // 处理接收到的完整指令
      processCommand(inputBuffer);
      // 清空缓冲区准备下一条指令
      inputPos = 0;
    } 
    else {
      // 普通字符，放到缓冲区。预留1字节给'\0'
      if (inputPos < sizeof(inputBuffer) - 1) {
        inputBuffer[inputPos++] = inChar;
      }
    }
  }
}


// void loop() {
//   // 检查串口是否有输入数据
//   if (Serial.available() > 0) {
//     // 读取串口的输入数据
//     char inChar = Serial.read();
//     Serial.write(inChar);
//     // 用于上位机探测串口号
//     if(inChar == 't') {
//       LED_wink(1);
//     } else if(inChar == 'u') {
//       LED_wink(2);    // 解锁开始
//       delay(200);
//       DigiKeyboard.sendKeyStroke(KEY_ESC);
//       DigiKeyboard.delay(1000);
//       DigiKeyboard.print("administrator");
//       DigiKeyboard.delay(200);
//       DigiKeyboard.sendKeyStroke(KEY_TAB);
//       DigiKeyboard.delay(200);
//       DigiKeyboard.print("1061700625");
//       DigiKeyboard.delay(200);
//       DigiKeyboard.sendKeyStroke(KEY_ENTER);
//       LED_wink(2);     // 解锁完成
//       delay(200);
//     }
//   }
// }
