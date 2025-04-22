// https://github.com/Robotechnic/DigiKeyboardFr
#include "DigiKeyboard.h"

// https://github.com/SpenceKonde/ATTinyCore
#include "TinySoftwareSerial.h"

void setup() {
  // open the serial port:
  Serial.begin(9600);
  Serial.println("Hello!");
  DigiKeyboard.sendKeyStroke(0);
  // DigiKeyboard.delay(5000);
}

void loop() {
  // check for incoming serial data:
  if (Serial.available() > 0) {
    // read incoming serial data:
    char inChar = Serial.read();
    Serial.write(inChar);
    if(inChar == 'u') {
      DigiKeyboard.delay(1000);
      DigiKeyboard.sendKeyStroke(KEY_ESC);
      DigiKeyboard.delay(500);
      DigiKeyboard.print("administrator");
      DigiKeyboard.delay(500);
      DigiKeyboard.sendKeyStroke(KEY_TAB);
      DigiKeyboard.delay(500);
      DigiKeyboard.print("1061700625");
      DigiKeyboard.delay(500);
      DigiKeyboard.sendKeyStroke(KEY_ENTER);
    }
  }
}
