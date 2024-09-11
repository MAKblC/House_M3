#include <WiFi.h>
#include <PubSubClient.h>

// окно и ворота
#include <ESP32_Servo.h>
Servo window;
Servo gates;
// углы предварительно калибруются
#define OPENED_GATES 130
#define CLOSED_GATES 40
#define OPENED_WINDOW 20
#define CLOSED_WINDOW 120

// Адресная лента
#include <FastLED.h>
#define NUM_LEDS 10      // количество светодиодов в матрице
CRGB leds[NUM_LEDS];     // определяем матрицу (FastLED библиотека)
#define LED_PIN 18       // пин к которому подключена матрица
#define COLOR_ORDER GRB  // порядок цветов матрицы
#define CHIPSET WS2812   // тип светодиодов

// кнопка антивандальная
#define BUTTON 19  // пин для подключения
int buttonState;   // переменная для хранения состояния кнопки

// переменные состояния ЧС
#define THIEF 1
#define GAS 2
#define EARTH 3
#define WATER 4
#define FIRE 5
// критические показатели датчиков
int dist_max = 50; // максимальное расстояние до проникновения
int fire_max = 500; // порог срабатывания датчика пожара
int gas_max = 100; // порог срабатвания датчика газа
int sound_max = 1000; // порог срабатывания шума
int accel_max = 1; // порог отклонения акселерометра от ровного положения
int water_max = 20; // порог срабатывания датчика протечки

#define WHITE_LED 17 // светодиодная лента белая
#define WIND 16 // вентилятор

////////////////////// Настройки //////////////////////
// Wi-Fi
const char* ssid = "MGBot";
const char* password = "Terminator812";

const char* mqtt_server = "m8.wqtt.ru";  // адрес
int mqtt_port = 16976;                   // порт
const char* mqtt_login = "u_FP4I6G";     // пользователь
const char* mqtt_pass = "jp7VZyhz";      // пароль

WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE (50)
char msg[MSG_BUFFER_SIZE];
int value = 0;
const int sending_period = 5;
const bool retain_flag = false;

// топики в WQTT
const char* temp_topic = "Temp"; // температура       
const char* hum_topic = "Hum"; // влажность
const char* press_topic = "Press"; // давление    
const char* light_topic = "Light"; // освещенность    
const char* ErrorThief = "ErrorThief"; // проникновение 
const char* ErrorWater = "ErrorWater"; // протечка 
const char* ErrorGas = "ErrorGas"; // утечка газа     
const char* ErrorFire = "ErrorFire"; // пожар   
const char* ErrorEarth = "ErrorEarth"; // землетрясение
const char* Guest = "Guest"; // гости          

uint32_t tmr1; // для отслеживания датчиков

#include <Wire.h>

// MGS-THP80
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
Adafruit_BME280 bme280;

// MGS-L75
#include <BH1750.h>
BH1750 lightMeter;

// MGS-FR403
#include <MGS_FR403.h>
MGS_FR403 Fire;

// MGS-D20
#include <VL53L0X.h>
VL53L0X lox;

// MGB-LCD12864
#include <I2C_graphical_LCD_display.h>
I2C_graphical_LCD_display lcd;

// MGB-BUZ1
#include <Adafruit_MCP4725.h>
Adafruit_MCP4725 buzzer;
int vol1 = 1000;
int vol2 = 100;  // разница значений = громкость
int ton;

// MGS-CO30
#include "SparkFun_SGP30_Arduino_Library.h"
SGP30 mySensor;

// MGS-SND504
#include "MCP3221.h"
MCP3221 mcp3221(0x4E);
#include "mcp3021.h"

// MGS-WT1
byte ADDR = 0b011;  // 0x4B
MCP3021 mcp3021;
const float air_value = 561.0;
const float water_value = 293.0;
const float moisture_0 = 0.0;
const float moisture_100 = 100.0;

// MGS-A6
#include <Adafruit_MPU6050.h>
Adafruit_MPU6050 mpu;

// подключение к сети
void setup_wifi() {

  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

// функция приёма и обработки пакета данных
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  // обрабаотка пришедшего сообщения
  
  // отслеживание белой ленты
  if (String(topic) == "TopicLight" and (char) payload[0] == '1') {
    digitalWrite(WHITE_LED, HIGH); 
  } else if (String(topic) == "TopicLight" and (char) payload[0] == '0') {
    digitalWrite(WHITE_LED, LOW);
    // отслеживание вентилятора
  } else if (String(topic) == "TopicWind" and (char) payload[0] == '1') {
    digitalWrite(WIND, HIGH);
  } else if (String(topic) == "TopicWind" and (char) payload[0] == '0') {
    digitalWrite(WIND, LOW);
    // отслеживание открытия/закрытия окна и ворот
  } else if ((char)payload[0] == '2') {
    window_turn(OPENED_WINDOW);
  } else if ((char)payload[0] == '3') {
    window_turn(CLOSED_WINDOW);
  } else if ((char)payload[0] == '4') {
    gates_turn(OPENED_GATES);
  } else if ((char)payload[0] == '5') {
    gates_turn(CLOSED_GATES);
  }
}

// Подключение к серверу
void reconnect() {
  // Ждем, пока не подсоединимся
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Создается случайный номер клиента
    String clientId = "IoTik32Client-";
    clientId += String(random(0xffff), HEX);
    // Попытка подключения
    if (client.connect(clientId.c_str(), mqtt_login, mqtt_pass)) {
      Serial.println("connected");
      // ... и подписки на топики
      client.subscribe("TopicWindow");
      client.subscribe("TopicRGB");
      client.subscribe("TopicLight");
      client.subscribe("TopicWind");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // 5 секунд для повторного подключения
      delay(5000);
    }
  }
}

void setup() {
  pinMode(WHITE_LED, OUTPUT);
  pinMode(WIND, OUTPUT);
  pinMode(BUTTON, INPUT);

  // окно и ворота
  window.attach(4);
  gates.attach(13);
  window.write(120);
  gates.write(40);

  // адресная лента
  FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, NUM_LEDS);
  
  Serial.begin(115200);
  
  Wire.begin();

  // MGS-THP80
  bool bme_status = bme280.begin();
  if (!bme_status) {
    Serial.println("Не найден по адресу 0х77, пробую другой...");
    bme_status = bme280.begin(0x76);
    if (!bme_status)
      Serial.println("Датчик не найден, проверьте соединение");
  }

  // MGS-L75
  setBusChannel(0x04);
  lightMeter.begin();

  // MGS-D20
  lox.init();
  lox.setTimeout(500);
  // параметры для режима высокой точности
  lox.setMeasurementTimingBudget(200000);

  // MGB-LCD12864
  setBusChannel(0x04);
  lcd.begin();
  lcd.gotoxy(10, 50);  // координата курсора
  lcd.string("Privet, MGBOT!", false);

  // MGB-BUZ1
  buzzer.begin(0x60);           // 0x61
  buzzer.setVoltage(0, false);  // выключение звука

  // MGS-CO30
  if (mySensor.begin() == false) {
    Serial.println("No SGP30 Detected. Check connections.");
  }
  mySensor.initAirQuality();

  // MGS-FR403
  Fire.begin();

  // MGS-A6
  if (!mpu.begin(0x69)) {
    Serial.println("Failed to find MPU6050 chip");
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // MGS-WT1
  setBusChannel(0x05);
  mcp3021.begin(ADDR);

  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}

void loop() {
  // подключение к брокеру
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // отслеживание датчиков по таймеру
  if (millis() - tmr1 >= (sending_period * 1000)) {
    tmr1 = millis();
    sendData();
  }

  // измерение
  setBusChannel(0x05);
  float adc0 = mcp3221.getVoltage();
  Serial.println("Уровень звука = " + String(adc0, 1));
  float dist = lox.readRangeSingleMillimeters();
  Serial.println("Расстояние  = " + String(dist, 0) + " мм  ");
  // отправить уведомление в случае проникновения
  if (dist < dist_max or adc0 < sound_max) {
    danger(THIEF);
  }

  mySensor.measureAirQuality();
  Serial.println("TVOC: " + String(mySensor.TVOC) + " ppb");
  // отправить уведомление в случае утечки газа
  if (mySensor.TVOC > gas_max) {
    danger(GAS);
  }

  float adc1 = mcp3021.readADC();
  float wl = map(adc1, air_value, water_value, moisture_0, moisture_100);
  Serial.println("Уровень протечки = " + String(wl, 1) + " %");
  // отправить уведомление в случае протечки воды
  if (wl > water_max) {
    danger(WATER);
  }

  Fire.get_ir_and_vis();
  Serial.println("ИК: " + String(Fire.ir_data, 0) + " мкВт/см2");
  // отправить уведомление в случае пожара
  if (Fire.ir_data > fire_max) {
    danger(FIRE);
  }

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  Serial.print("Акселерометр ось X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", ось Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", ось Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");
  // отправить уведомление в случае землетрясения
  if (a.acceleration.x > accel_max) {
    danger(EARTH);
  }

  buttonState = digitalRead(BUTTON);  // проверяем состояние кнопки
  if (buttonState == LOW) {
    Serial.println("ВКЛ");
    // отправить уведомление в прихода гостей
    client.publish(Guest, "1", retain_flag);
  }
}

// отправка данных с датчиков
void sendData() {
  float t = bme280.readTemperature();
  float h = bme280.readHumidity();
  float p = bme280.readPressure() / 133.3F;
  setBusChannel(0x04);
  float lux = lightMeter.readLightLevel();
  client.publish(temp_topic, String(t).c_str(), retain_flag);
  client.publish(hum_topic, String(h).c_str(), retain_flag);
  client.publish(press_topic, String(p).c_str(), retain_flag);
  client.publish(light_topic, String(lux).c_str(), retain_flag);
}

// обраотка ЧС
void danger(int type) {
  switch (type) {
    case 1:  // проникновение
      leds[0].setRGB(255, 255, 0);
      leds[1].setRGB(255, 255, 0);
      client.publish(ErrorThief, "1", retain_flag);
      break;
    case 2:  // утечка газа
      leds[2].setRGB(0, 255, 255);
      leds[3].setRGB(0, 255, 255);
      client.publish(ErrorGas, "1", retain_flag);
      break;
    case 3:  // землетрясение
      leds[4].setRGB(0, 255, 0);
      leds[5].setRGB(0, 255, 0);
      client.publish(ErrorEarth, "1", retain_flag);
      break;
    case 4:  // протечка труб
      leds[6].setRGB(0, 0, 255);
      leds[7].setRGB(0, 0, 255);
      client.publish(ErrorWater, "1", retain_flag);
      break;
    case 5:  // пожар
      leds[8].setRGB(255, 0, 0);
      leds[9].setRGB(255, 0, 0);
      client.publish(ErrorFire, "1", retain_flag);
      break;
  }
  FastLED.show();
  note(3, 450);
  note(5, 150);
  setBusChannel(0x04);
  lcd.clear();
  lcd.gotoxy(10, 50);  // координата курсора
  lcd.string("CAUTION!", false);
  FastLED.clear();
  FastLED.show();
}

// функция смены I2C-шины
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

// функция генерирования звука в виде нот
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

// функция плавного движения ворот
void gates_turn(int angle) {
  int currentAngle = gates.read();
  if (currentAngle < angle) {
    for (int i = currentAngle; i < angle; i++) {
      gates.write(i);
      delay(15);
    }
  } else {
    for (int i = currentAngle; i > angle; i--) {
      gates.write(i);
      delay(15);
    }
  }
}

// функция плавного движения окна
void window_turn(int angle) {
  int currentAngle = window.read();
  if (currentAngle < angle) {
    for (int i = currentAngle; i < angle; i++) {
      window.write(i);
      delay(25);
    }
  } else {
    for (int i = currentAngle; i > angle; i--) {
      window.write(i);
      delay(25);
    }
  }
}
