#include <iocontrol.h>
#include <WiFi.h>

// Wi-Fi
const char* ssid = "XXXXXXXXX";
const char* password = "XXXXXXXX";

// Название панели на сайте iocontrol.ru
const char* myPanelName = "XXXXXXXXX";
int status;

// Название переменных как на сайте iocontrol.ru
const char* VarName_sensors = "sensors";
const char* VarName_Wind = "Wind";
const char* VarName_rgb = "RGB";
const char* VarName_LED = "LED";
const char* VarName_window = "window";
const char* VarName_gates = "gates";
const char* VarName_button = "button";
const char* VarName_buzzer = "buzzer";
const char* VarName_lcd = "lcd";

// Создаём объект клиента
WiFiClient client;
// Создаём объект iocontrol, передавая название панели и клиента
iocontrol mypanel(myPanelName, client);

#include <FastLED.h>
#define NUM_LEDS 10
CRGB leds[NUM_LEDS];
#define LED_PIN 18
#define COLOR_ORDER GRB
#define CHIPSET WS2812

#include <ESP32_Servo.h>
Servo window;
Servo gates;
#define OPENED_GATES 70
#define CLOSED_GATES 30
#define OPENED_WINDOW 30
#define CLOSED_WINDOW 70

#define WHITE_LED 17
#define WIND 16
#define BUTTON 19  
int buttonState;   

#include <Wire.h>

#include <Adafruit_MCP4725.h>
Adafruit_MCP4725 buzzer;
int vol1 = 1000;
int vol2 = 100; 
int ton;

#include <I2C_graphical_LCD_display.h>
I2C_graphical_LCD_display lcd;

#include <BH1750.h>
BH1750 lightMeter;

#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
Adafruit_BME280 bme280;

#include <VL53L0X.h>
VL53L0X lox;

#include <MGS_FR403.h>
MGS_FR403 Fire;

#include <Adafruit_MPU6050.h>
Adafruit_MPU6050 mpu;

#include "MCP3221.h"
MCP3221 mcp3221(0x4E);

#include "mcp3021.h"
byte ADDR = 0b011;  // 0x4B
MCP3021 mcp3021;
const float air_value = 561.0;
const float water_value = 293.0;
const float moisture_0 = 0.0;
const float moisture_100 = 100.0;

#include "SparkFun_SGP30_Arduino_Library.h"
SGP30 mySensor;

int count = 0;

void setup() {
  Serial.begin(115200);
  window.attach(4);
  gates.attach(13);
  FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, NUM_LEDS);
  FastLED.setBrightness(90);

  pinMode(WHITE_LED, OUTPUT);
  pinMode(WIND, OUTPUT);
  pinMode(BUTTON, INPUT);

  Wire.begin();

  if (mySensor.begin() == false) {
    Serial.println("No SGP30 Detected. Check connections.");
  }
  mySensor.initAirQuality();

  buzzer.begin(0x60);          
  buzzer.setVoltage(0, false); 

  setBusChannel(0x04);
  lcd.begin();
  lcd.gotoxy(10, 50); 
  lcd.string("Privet, MGBOT!", false);

  setBusChannel(0x04);
  lightMeter.begin();

  bool bme_status = bme280.begin();
  if (!bme_status) {
    Serial.println("Не найден по адресу 0х77, пробую другой...");
    bme_status = bme280.begin(0x76);
    if (!bme_status)
      Serial.println("Датчик не найден, проверьте соединение");
  }

  lox.init();
  lox.setTimeout(500);
  lox.setMeasurementTimingBudget(200000);

  Fire.begin();

  if (!mpu.begin(0x69)) {
    Serial.println("Failed to find MPU6050 chip");
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  setBusChannel(0x05);
  mcp3021.begin(ADDR);

  WiFi.begin(ssid, password);
  // Ждём подключения
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  // Вызываем функцию первого запроса к сервису
  mypanel.begin();
}

void loop()  // вызываем функцию обработки сообщений через определенный период
{
  // ************************ ЧТЕНИЕ ************************
  // Чтение значений переменных из сервиса
  status = mypanel.readUpdate();
  // Если статус равен константе OK...
  if (status == OK) {
    // Выводим текст в последовательный порт
    Serial.println("------- Read OK -------");
    // Записываем считанные из сервиса значения в переменные
    digitalWrite(WIND, mypanel.readInt(VarName_Wind)); // состояние вентилятора
    digitalWrite(WHITE_LED, mypanel.readInt(VarName_LED)); // состояние светодиодной ленты
    int io_buzzer = mypanel.readInt(VarName_buzzer); // состояние генератора звука
    if (io_buzzer == 1) {
      buzzer.setVoltage(0, false);  // выключение звука
      note(3, 450);
      note(5, 150);
      note(6, 450);  // пример нескольких нот
    }
    int io_rgb = mypanel.readInt(VarName_rgb); // состояние адресной ленты
    if (io_rgb == 1) {
      for (int i = 0; i < NUM_LEDS; i++) {
        leds[i].setHue(i * 255 / NUM_LEDS);
      }
      FastLED.show();
    } else {
      FastLED.clear();
      FastLED.show();
    }
    int io_window = mypanel.readInt(VarName_window); // состояние окна
    if (io_window == 1) {
      window.write(OPENED_WINDOW);
    } else {
      window.write(CLOSED_WINDOW);
    }
    int io_gates = mypanel.readInt(VarName_gates); // состояние ворот
    if (io_gates == 1) {
      gates.write(OPENED_GATES);
    } else {
      gates.write(CLOSED_GATES);
    }

    String myString = mypanel.readString(VarName_lcd); // дублирование фразы на дисплей
    setBusChannel(0x04);
    lcd.clear(); // очистить экран
    char charBuf[20]; // создать буффер
    myString.toCharArray(charBuf, 20); // поместить в буффер переменную
    lcd.string(charBuf, false); // отправить на дисплей
  }
  
  // ************************ ЗАПИСЬ ************************
  // Отправляем переменные из контроллера в сервис
  buttonState = digitalRead(BUTTON);  // проверяем состояние кнопки
  mypanel.write(VarName_button, !buttonState); // дублируем состояние кнопки на панель
  // считывание датчиков
  setBusChannel(0x04);
  float lux = lightMeter.readLightLevel();
  float t = bme280.readTemperature();
  float h = bme280.readHumidity();
  float p = bme280.readPressure() / 133.3F;
  float dist = lox.readRangeSingleMillimeters();
  Fire.get_ir_and_vis();
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  setBusChannel(0x05);
  float adc0 = mcp3221.getVoltage();
  float adc1 = mcp3021.readADC();
  float wl = map(adc1, air_value, water_value, moisture_0, moisture_100);
  mySensor.measureAirQuality();
  // поочередно сменяем надпись на панели, выводя каждый датчик
  String sensorValue;
  switch (count) {
    case 1:
      sensorValue = "temp_" + String(t, 0);
      break;
    case 2:
      sensorValue = "hum_" + String(h, 0);
      break;
    case 3:
      sensorValue = "press_" + String(p, 0);
      break;
    case 4:
      sensorValue = "light_" + String(lux, 0);
      break;
    case 5:
      sensorValue = "dist_" + String(dist, 0);
      break;
    case 6:
      sensorValue = "IR_" + String(Fire.ir_data, 0);
      break;
    case 7:
      sensorValue = String(a.acceleration.x, 0) + "_" + String(a.acceleration.y, 0) + "_" + String(a.acceleration.z, 0) + "_";
      break;
    case 8:
      sensorValue = "sound_" + String(adc0, 0);
      break;
    case 9:
      sensorValue = "water_" + String(wl, 0);
      break;
    case 10:
      sensorValue = "CO2_" + String(mySensor.CO2);
      break;
    case 11:
      sensorValue = "TVOC_" + String(mySensor.TVOC);
      break;
    case 12: // сброс заново
      count = 0;
      break;
  }
  count++; // счетчик номера датчика для показа
  mypanel.write(VarName_sensors, sensorValue); // отправка значения на панель
  status = mypanel.writeUpdate();
  // Если статус равен константе OK...
  if (status == OK) {
    // Выводим текст в последовательный порт
    Serial.println("------- Write OK -------");
  }
  delay(1000);
}

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
