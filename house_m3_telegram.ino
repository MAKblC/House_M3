/*
Не забудьте поправить настройки и адреса устройств в зависмости от комплектации!  
*/
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>

#define WIFI_SSID "MGBot"
#define WIFI_PASSWORD "Terminator812"
// токен вашего бота
#define BOT_TOKEN "6373693891:AAG41EIiBxLpRTPJLOcrWxQIAV-6oe0qjNI"

// ссылка для поста фотографии
String test_photo_url = "https://mgbot.ru/upload/logo-r.png";

// отобразить кнопки перехода на сайт с помощью InlineKeyboard
String keyboardJson1 = "[[{ \"text\" : \"Ваш сайт\", \"url\" : \"https://mgbot.ru\" }],[{ \"text\" : \"Перейти на сайт IoTik.ru\", \"url\" : \"https://www.iotik.ru\" }]]";

WiFiClientSecure secured_client;
UniversalTelegramBot bot(BOT_TOKEN, secured_client);
unsigned long bot_lasttime;
const unsigned long BOT_MTBS = 1000; // период обновления сканирования новых сообщений

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
#define BUTTON 19  // пин для подключения
int buttonState;   // переменная для хранения состояния кнопки

#include <Wire.h>

#include <Adafruit_MCP4725.h>
Adafruit_MCP4725 buzzer;
int vol1 = 1000;
int vol2 = 100;  // разница значений = громкость
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

void setup() {
  Serial.begin(115200);
  window.attach(4);
  gates.attach(13);
  FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, NUM_LEDS);
  FastLED.setBrightness(90);

  pinMode(WHITE_LED, OUTPUT);
  pinMode(WIND, OUTPUT);
  pinMode(BUTTON, INPUT);

  Serial.print("Connecting to Wifi SSID ");
  Serial.print(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  secured_client.setCACert(TELEGRAM_CERTIFICATE_ROOT);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.print("\nWiFi connected. IP address: ");
  Serial.println(WiFi.localIP());

  Wire.begin();

  if (mySensor.begin() == false) {
    Serial.println("No SGP30 Detected. Check connections.");
  }
  mySensor.initAirQuality();

  buzzer.begin(0x60);           // С перемычкой адрес будет 0x60
  buzzer.setVoltage(0, false);  // выключение звука

  setBusChannel(0x04);
  lcd.begin();
  lcd.gotoxy(10, 50);  // координата курсора
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
  // параметры для режима высокой точности
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
}

void handleNewMessages(int numNewMessages) {
  Serial.print("handleNewMessages ");
  Serial.println(numNewMessages);

  for (int i = 0; i < numNewMessages; i++) {
    String chat_id = bot.messages[i].chat_id;
    String text = bot.messages[i].text;
    text.toLowerCase();
    String from_name = bot.messages[i].from_name;
    if (from_name == "")
      from_name = "Guest";

    // выполняем действия в зависимости от пришедшей команды
    if (text == "/sensors")  // измеряем данные
    {
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
      String welcome = "Показания датчиков:\n-------------------------------------------\n";
      welcome += "🌡 Температура воздуха: " + String(t, 1) + " °C\n";
      welcome += "💧 Влажность воздуха: " + String(h, 0) + " %\n";
      welcome += "☁ Атмосферное давление: " + String(p, 0) + " гПа\n";
      welcome += "☀ Освещенность: " + String(lux) + " Лк\n";
      welcome += "🚰 Уровень воды: " + String(wl, 0) + " %\n";
      welcome += "☢ Концентрация ЛОС: " + String(mySensor.TVOC) + " ppb\n";
      welcome += "🌬 Концентрация СО2: " + String(mySensor.CO2) + " ppm\n";
      welcome += "📏 Расстояние: " + String(dist, 0) + " мм\n";
      welcome += "accel x = " + String(a.acceleration.x) + "\n";
      welcome += "accel y = " + String(a.acceleration.y) + "\n";
      welcome += "accel z = " + String(a.acceleration.z) + "\n";
      welcome += "🔥 ИК-канал = " + String(Fire.ir_data, 0) + " мкВт/см2\n";
      welcome += "🔥 Видимый канал = " + String(Fire.vis_data, 0) + " мкВт/см2\n";
      welcome += "🎧 Уровень звука = " + String(adc0, 0) + "\n";
      bot.sendMessage(chat_id, welcome, "Markdown");
    }
    
    if (text == "/photo") {  // пост фотографии
      bot.sendPhoto(chat_id, test_photo_url, "а вот и фотка!");
    }
    if (text == "/windon") {
      digitalWrite(WIND, HIGH);
      bot.sendMessage(chat_id, "Вентилятор включен", "");
    }
    if (text == "/windoff") {
      digitalWrite(WIND, LOW);
      bot.sendMessage(chat_id, "Вентилятор выключен", "");
    }
    if (text == "/lighton") {
      digitalWrite(WHITE_LED, HIGH);
      bot.sendMessage(chat_id, "Освещение включено", "");
    }
    if (text == "/lightoff") {
      digitalWrite(WHITE_LED, LOW);
      FastLED.clear();
      FastLED.show();
      bot.sendMessage(chat_id, "Освещение выключено", "");
    }
    if (text == "/gateon") {
      gates.write(OPENED_GATES);
      bot.sendMessage(chat_id, "Ворота открыты", "");
    }
    if (text == "/gateoff") {
      gates.write(CLOSED_GATES);
      bot.sendMessage(chat_id, "Ворота закрыты", "");
    }
    if (text == "/windowon") {
      window.write(OPENED_WINDOW);
      bot.sendMessage(chat_id, "Окно открыто", "");
    }
    if (text == "/windowoff") {
      window.write(CLOSED_WINDOW);
      bot.sendMessage(chat_id, "Окно закрыто", "");
    }
    if (text == "/sound") {
      note(14, 400);
      note(2, 100);
      buzzer.setVoltage(0, false);  // выключение звука
    }
    if (text == "/randomrgb") {
      fill_solid(leds, NUM_LEDS, CRGB(random(0, 255), random(0, 255), random(0, 255)));
      FastLED.show();
      bot.sendMessage(chat_id, "Случайный цвет включен", "");
    }
    if ((text == "/start") || (text == "start") || (text == "/help") || (text == "help"))  // команда для вызова помощи
    {
      bot.sendMessage(chat_id, "Привет, " + from_name + "!", "");
      bot.sendMessage(chat_id, "Я контроллер Йотик 32. Команды смотрите в меню слева от строки ввода", "");
      String sms = "Команды:\n";
      sms += "/options - пульт управления\n";
      sms += "/site - перейти на сайт\n";
      sms += "/photo - запостить фото\n";
      sms += "/help - вызвать помощь\n";
      bot.sendMessage(chat_id, sms, "Markdown");
    }
    if (text == "/site")  // отобразить кнопки в диалоге для перехода на сайт
    {
      bot.sendMessageWithInlineKeyboard(chat_id, "Выберите действие", "", keyboardJson1);
    }

    if (text == "/options")  // клавиатура для управления теплицей
    {
      String keyboardJson = "[[\"/lighton\", \"/lightoff\"],[\"/randomrgb\",\"/sensors\"],[\"/sound\",\"/windon\", \"/windoff\"],[\"/gateon\",\"/gateoff\",\"/windowon\",\"/windowoff\"]]";
      bot.sendMessageWithReplyKeyboard(chat_id, "Выберите команду", "", keyboardJson, true);
    }
  }
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

void loop()  // вызываем функцию обработки сообщений через определенный период
{
  if (millis() - bot_lasttime > BOT_MTBS) {
    int numNewMessages = bot.getUpdates(bot.last_message_received + 1);

    while (numNewMessages) {
      Serial.println("got response");
      handleNewMessages(numNewMessages);
      numNewMessages = bot.getUpdates(bot.last_message_received + 1);
    }

    bot_lasttime = millis();
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