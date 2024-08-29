// адресная лента
#include <FastLED.h>
#define NUM_LEDS 10
CRGB leds[NUM_LEDS];
#define LED_PIN 18
#define COLOR_ORDER GRB
#define CHIPSET WS2812

// окно и ворота
#include <ESP32_Servo.h>
Servo window;
Servo gates;
#define OPENED_GATES 70 // угол для открытых ворот
#define CLOSED_GATES 30 // угол для закрытых ворот
#define OPENED_WINDOW 30 // угол для открытого окна
#define CLOSED_WINDOW 70 // угол для закрытого окна

#define WHITE_LED 17 // светодиодная лента белая
#define WIND 16 /// вентилятор
#define BUTTON 19  // кнопка
int buttonState;   // переменная для хранения состояния кнопки

#include <Wire.h> // I2C

// MGB-BUZ1
#include <Adafruit_MCP4725.h>
Adafruit_MCP4725 buzzer;
int vol1 = 1000;
int vol2 = 100;  // разница значений = громкость
int ton;

// MGB-LCD12864
#include <I2C_graphical_LCD_display.h>
I2C_graphical_LCD_display lcd;

// MGS-L75
#include <BH1750.h>
BH1750 lightMeter;

// MGS-THP80
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
Adafruit_BME280 bme280;

// MGS-D20
#include <VL53L0X.h>
VL53L0X lox;

// MGS-FR403
#include <MGS_FR403.h>
MGS_FR403 Fire;

//MGS-A6
#include <Adafruit_MPU6050.h>
Adafruit_MPU6050 mpu;

//MGS-SND504
#include "MCP3221.h"
MCP3221 mcp3221(0x4E);

// MGS-WT1
#include "mcp3021.h"
byte ADDR = 0b011;  // 0x4B
MCP3021 mcp3021;
const float air_value = 561.0;
const float water_value = 293.0;
const float moisture_0 = 0.0;
const float moisture_100 = 100.0;

// MGS-CO30
#include "SparkFun_SGP30_Arduino_Library.h"
SGP30 mySensor;

void setup() {
  Serial.begin(115200);
  window.attach(4); // пин окна
  gates.attach(13); // пин ворот

  // адресная лента
  FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, NUM_LEDS);
  FastLED.setBrightness(90);

  pinMode(WHITE_LED, OUTPUT);
  pinMode(WIND, OUTPUT);
  pinMode(BUTTON, INPUT);

  Wire.begin(); // I2C

  // MGS-CO30
  if (mySensor.begin() == false) {
    Serial.println("No SGP30 Detected. Check connections.");
  }
  mySensor.initAirQuality();

  // MGB-BUZ1
  buzzer.begin(0x60);           // С перемычкой адрес будет 0x60
  buzzer.setVoltage(0, false);  // выключение звука

  // MGB-LCD12864
  setBusChannel(0x04);
  lcd.begin();
  lcd.gotoxy(10, 50);  // координата курсора
  lcd.string("Privet, MGBOT!", false);

  // MGS-L75
  setBusChannel(0x04);
  lightMeter.begin();

  // MGS-THP80
  bool bme_status = bme280.begin();
  if (!bme_status) {
    Serial.println("Не найден по адресу 0х77, пробую другой...");
    bme_status = bme280.begin(0x76);
    if (!bme_status)
      Serial.println("Датчик не найден, проверьте соединение");
  }

  // MGS-D20
  lox.init();
  lox.setTimeout(500);
  // параметры для режима высокой точности
  lox.setMeasurementTimingBudget(200000);

  // MGS-FR403
  Fire.begin();

  // MGS-A6
  if (!mpu.begin(0x69)) {
    Serial.println("Failed to find MPU6050 chip");
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // MGS-SND504
  setBusChannel(0x05);
  mcp3021.begin(ADDR);
}

void loop() {
  digitalWrite(WIND, LOW); // выкл вентилятор
  digitalWrite(WHITE_LED, LOW); // выкл светодиодную ленту
  // бегущий огонь 3 раза
  for (int i = 0; i < 3; i++) {
    for (int i = 0; i < NUM_LEDS; i++) {
      FastLED.clear();
      leds[i].setHue(i * 255 / NUM_LEDS);
      FastLED.show();
      delay(100);
    }
  }
  /// вкл вентилятор и ленту
  digitalWrite(WIND, HIGH);
  digitalWrite(WHITE_LED, HIGH);
  window.write(OPENED_WINDOW); // открыть окно
  delay(1000);
  window.write(CLOSED_WINDOW); // закрыть окно
  gates.write(OPENED_GATES); // открыть ворота
  delay(1000);
  gates.write(CLOSED_GATES); // закрыть ворота
  buzzer.setVoltage(0, false);  // выключение звука
  note(3, 450);
  note(5, 150);
  note(6, 450);  // пример нескольких нот
  delay(300);
  buttonState = digitalRead(BUTTON);  // проверяем состояние кнопки
  if (buttonState == LOW) {
    Serial.println("ВКЛ");
  } else {
    Serial.println("ВЫКЛ");
  }

  // считывание датчиков
  
  setBusChannel(0x04);
  float lux = lightMeter.readLightLevel();
  Serial.println("Освещенность: " + String(lux, 0) + " Люкс");

  float t = bme280.readTemperature();
  float h = bme280.readHumidity();
  float p = bme280.readPressure() / 133.3F;
  Serial.println("Температура = " + String(t, 1) + "°C");
  Serial.println("Влажность = " + String(h, 0) + "%");
  Serial.println("Атм. давление = " + String(p, 0) + " мм рт.ст.");

  float dist = lox.readRangeSingleMillimeters();
  Serial.println("Расстояние  = " + String(dist, 0) + " мм  ");

  Fire.get_ir_and_vis();
  Serial.println("ИК: " + String(Fire.ir_data, 0) + " мкВт/см2");
  Serial.println("Видимый: " + String(Fire.vis_data, 0) + " мкВт/см2");

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  Serial.print("Акселерометр ось X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", ось Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", ось Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  setBusChannel(0x05);
  float adc0 = mcp3221.getVoltage();
  Serial.println("Уровень звука = " + String(adc0, 1));

  float adc1 = mcp3021.readADC();
  float wl = map(adc1, air_value, water_value, moisture_0, moisture_100);
  Serial.println("Уровень протечки = " + String(wl, 1) + " %");

  mySensor.measureAirQuality();
  Serial.println("CO2: " + String(mySensor.CO2) + " ppm");
  Serial.println("TVOC: " + String(mySensor.TVOC) + " ppb");
}

// функция создания ноты
void note(int type, int duration) {  // нота (нота, длительность)
  switch (type) {
    case 1: ton = 1000; break;
    case 2: ton = 860; break;
    case 3: ton = 800; break;
    case 4: ton = 700; break;
    case 5: ton = 600; break;
    case 6: ton = 525; break;
    case 7: ton = 450; break;
    case 8: ton = 380; break;
    case 9: ton = 315; break;
    case 10: ton = 250; break;
    case 11: ton = 190; break;
    case 12: ton = 130; break;
    case 13: ton = 80; break;
    case 14: ton = 30; break;
    case 15: ton = 1; break;
  }
  delay(10);  // воспроизведение звука с определенной тональностью и длительностью
  for (int i = 0; i < duration; i++) {
    buzzer.setVoltage(vol1, false);
    buzzer.setVoltage(vol2, false);
    delayMicroseconds(ton);
  }
}

// функция смены I2C-канала
bool setBusChannel(uint8_t i2c_channel) {
  if (i2c_channel >= 0x08) {
    return false;
  } else {
    Wire.beginTransmission(0x70);
    //Wire.write(i2c_channel | 0x08);  // для микросхемы PCA9547
    Wire.write(0x01 << i2c_channel);  // Для микросхемы PW548A*/
    Wire.endTransmission();
    return true;
  }
}
