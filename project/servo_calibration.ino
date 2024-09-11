#include <ESP32_Servo.h>
Servo window;
Servo gates;
int incomingByte = 0;
bool mode = 0;

void setup() {
  window.attach(4);
  gates.attach(13);
  Serial.begin(115200);
}

void loop() {
  if (Serial.available() > 0) {  //если есть доступные данные
    // считываем байт
    incomingByte = Serial.parseInt();
    if (incomingByte == 888) {
      mode = 1; // смена режима
      Serial.println("окно");
    } else if (incomingByte == 777) {
      mode = 0;
      Serial.println("ворота");
    }
    // отсылаем то, что получили
    Serial.print("I received: ");
    Serial.println(incomingByte);
    if (incomingByte > 0 and incomingByte < 181 and mode == 0) {
      gates.write(incomingByte);
    }
    if (incomingByte > 0 and incomingByte < 181 and mode == 1) {
      window.write(incomingByte);
    }
  }
}
