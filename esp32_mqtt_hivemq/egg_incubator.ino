#include <WiFi.h>
#include <DHT.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>

#define DHTPIN 4
#define DHTTYPE DHT11

SoftwareSerial SIM(5, 6);
const char *ssid = "VIET NAM VO DICH";
const char *password = "12345678";

const char *mqtt_server = "mqtt.coder96.com";
const char *mqtt_user = "your_mqtt_username";
const char *mqtt_password = "your_mqtt_password";
const char *mqtt_topic_pub = "incubator/datapub";

DHT dht(DHTPIN, DHTTYPE);
LiquidCrystal_I2C lcd(0x27, 16, 2);

WiFiClient espClient;
PubSubClient client(espClient);

#define RELAY_PIN 5
#define FAN_PIN 6
#define BUZZER_PIN 7
#define BUTTON_START_PIN 12
#define BUTTON_1_PIN 13
#define BUTTON_2_PIN 14
#define BUTTON_STOP_BUZZER_PIN 15

volatile bool flagPause = false;
volatile bool flagButton1 = false;
volatile bool flagButton2 = false;
volatile bool buttonStartPressed = false;
volatile bool button1Pressed = false;
volatile bool button2Pressed = false;
volatile bool buttonStopBuzzerPressed = false;

unsigned long lastDebounceTimeStart = 0;
unsigned long lastDebounceTime1 = 0;
unsigned long lastDebounceTime2 = 0;
unsigned long lastDebounceTimeStopBuzzer = 0;
unsigned long hatchStartTime;
unsigned long estimatedHatchTime;
float setTemperature = 37.5; // Default temperature for incubating eggs

float count = 0.0;

void IRAM_ATTR handleButtonStartPress()
{
    if ((millis() - lastDebounceTimeStart) > 50)
    {
        buttonStartPressed = true;
        lastDebounceTimeStart = millis();
    }
}

void IRAM_ATTR handleButton1Press()
{
    if ((millis() - lastDebounceTime1) > 50)
    {
        button1Pressed = true;
        lastDebounceTime1 = millis();
    }
}

void IRAM_ATTR handleButton2Press()
{
    if ((millis() - lastDebounceTime2) > 50)
    {
        button2Pressed = true;
        lastDebounceTime2 = millis();
    }
}

void IRAM_ATTR handleButtonStopBuzzerPress()
{
    if ((millis() - lastDebounceTimeStopBuzzer) > 50)
    {
        buttonStopBuzzerPressed = true;
        lastDebounceTimeStopBuzzer = millis();
    }
}

void setup()
{
    Serial.begin(115200);

    // Kết nối Wi-Fi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }
    Serial.println("Connected to WiFi");

    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);

    // Khởi động module SIM 900A
    SIM.begin(115200);

    // Khởi động cảm biến DHT
    dht.begin();

    // Khởi động LCD
    lcd.init();
    lcd.backlight();

    // Cấu hình các chân GPIO
    pinMode(RELAY_PIN, OUTPUT);
    pinMode(FAN_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(BUTTON_START_PIN, INPUT_PULLUP);
    pinMode(BUTTON_1_PIN, INPUT_PULLUP);
    pinMode(BUTTON_2_PIN, INPUT_PULLUP);
    pinMode(BUTTON_STOP_BUZZER_PIN, INPUT_PULLUP);

    // Cấu hình ngắt ngoài cho các nút nhấn
    attachInterrupt(digitalPinToInterrupt(BUTTON_START_PIN), handleButtonStartPress, FALLING);
    attachInterrupt(digitalPinToInterrupt(BUTTON_1_PIN), handleButton1Press, FALLING);
    attachInterrupt(digitalPinToInterrupt(BUTTON_2_PIN), handleButton2Press, FALLING);
    attachInterrupt(digitalPinToInterrupt(BUTTON_STOP_BUZZER_PIN), handleButtonStopBuzzerPress, FALLING);

    // Hiển thị màn hình khởi động
    lcd.setCursor(0, 0);
    lcd.print("System Initializing");
    delay(2000); // Giả lập thời gian khởi tạo hệ thống

    // Hiển thị màn hình chờ
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Set Start Time");

    // Đọc thông số từ EEPROM nếu có
    EEPROM.begin(512);
    if (EEPROM.read(0) == 1)
    {
        EEPROM.get(1, hatchStartTime);
        EEPROM.get(9, estimatedHatchTime);
        EEPROM.get(17, setTemperature);
    }
}

void loop()
{
    if (!client.connected())
    {
        reconnect();
    }
    client.loop();

    // Xử lý các nút nhấn và cờ
    if (buttonStartPressed)
    {
        buttonStartPressed = false;
        if (millis() - lastDebounceTimeStart < 3000)
        {
            lcd.setCursor(0, 0);
            lcd.print("Humidity: ");
            lcd.print(dht.readHumidity());
            delay(2000);
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Set Start Time");
        }
        else
        {
            flagPause = true;
            lcd.setCursor(0, 0);
            lcd.print("Pause");
            lcd.setCursor(0, 1);
            lcd.print("ERROR");
            sendAlertToUser();
        }
    }

    if (button1Pressed)
    {
        button1Pressed = false;
        if (millis() - lastDebounceTime1 < 2000)
        {
            count += 0.1;
        }
        else
        {
            flagButton1 = true;
        }
    }

    if (button2Pressed)
    {
        button2Pressed = false;
        if (millis() - lastDebounceTime2 < 2000)
        {
            count -= 0.1;
        }
        else
        {
            flagButton2 = true;
        }
    }

    if (buttonStopBuzzerPressed)
    {
        buttonStopBuzzerPressed = false;
        digitalWrite(BUZZER_PIN, LOW); // Tắt chuông báo
    }

    // Đọc dữ liệu từ cảm biến DHT11
    float humidity = dht.readHumidity();
    float temperature = dht.readTemperature();

    if (isnan(humidity) || isnan(temperature))
    {
        Serial.println("Failed to read from DHT sensor!");
        lcd.setCursor(0, 0);
        lcd.print("Sensor Error");
        delay(2000);
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Set Start Time");
        return;
    }

    // Hiển thị dữ liệu lên LCD
    lcd.setCursor(0, 0);
    lcd.print("Temp: ");
    lcd.print(temperature);
    lcd.print(" C");
    lcd.setCursor(0, 1);
    lcd.print("Hum: ");
    lcd.print(humidity);
    lcd.print(" %");

    // Điều khiển sợi đốt và quạt qua relay
    controlTemperature(temperature);

    // Gửi dữ liệu lên server bằng MQTT
    sendDataToMQTT(temperature, humidity);

    // Kiểm tra thời gian dự kiến trứng nở
    unsigned long currentTime = millis();
    if (currentTime >= estimatedHatchTime)
    {
        digitalWrite(BUZZER_PIN, HIGH); // Kích hoạt chuông báo
        if ((currentTime / 300000) % 2 == 0)
        {                                  // 5 phút một lần
            digitalWrite(BUZZER_PIN, LOW); // Tắt chuông sau 10 giây
            delay(10000);
            digitalWrite(BUZZER_PIN, HIGH);
        }
    }

    delay(60000); // Gửi dữ liệu mỗi phút
    // Gưi tin nhắn và gọi nếu quá ngưỡng đã quy định
    if (temperature < 35 && temperature > 38)
    {
        GUI();
    }
}

void controlTemperature(float temperature)
{
    if (temperature < setTemperature)
    {
        digitalWrite(RELAY_PIN, HIGH); // Bật sợi đốt
        digitalWrite(FAN_PIN, LOW);    // Tắt quạt
    }
    else
    {
        digitalWrite(RELAY_PIN, LOW); // Tắt sợi đốt
        digitalWrite(FAN_PIN, HIGH);  // Bật quạt
    }
}

void sendDataToMQTT(float temperature, float humidity)
{
    StaticJsonDocument<200> doc;
    doc["temperature"] = temperature;
    doc["humidity"] = humidity;
    doc["hatch_start_time"] = hatchStartTime;
    doc["estimated_hatch_time"] = estimatedHatchTime;

    char payload[256];
    serializeJson(doc, payload);

    client.publish(mqtt_topic_pub, payload);
}

void GUI()
{
    SIM.println("AT+CMGF=1");
    delay(1000);
    SIM.println("AT+CMGS=\"+84981234602\"");
    delay(1000);
    SIM.print("WARNING! WARNING Incubator");
    delay(1000);
    SIM.write(26); // Ký tự kết thúc tin nhắn
    delay(1000);
}

void reconnect()
{
    while (!client.connected())
    {
        Serial.print("Attempting MQTT connection...");
        if (client.connect("ESP8266Client", mqtt_user, mqtt_password))
        {
            Serial.println("connected");
            client.subscribe(mqtt_topic_pub);
        }
        else
        {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 5 seconds");
            delay(5000);
        }
    }
}
// Nhận dữ liệu về
void callback(char *topic, byte *payload, unsigned int length)
{
    // Handle messages received from MQTT broker if needed
}