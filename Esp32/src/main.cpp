#include <WiFi.h>
#include <HardwareSerial.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <PubSubClient.h>
#include "sys_capserver.hpp"
#include "sys_wifi.hpp"
#include <TinyGPS++.h>
#include <Arduino.h>
#include <HardwareSerial.h>
#define DE_PIN 4  // Chân điều khiển DE
#define RE_PIN 2  // Chân điều khiển RE

const char *mqtt_ssid = "emqx";
const char *mqtt_password = "public";
const char *mqtt_server = "broker.emqx.io";
const int mqtt_port = 1883; // MQTT default port
const char *mqtt_topic = "esp32/test11";

#define BOOT_PIN 0

// Khai báo UART2 cho GPS
HardwareSerial rs485(1);
HardwareSerial gpsSerial(1);
TinyGPSPlus gps;

// Cấu trúc dữ liệu GPS
struct GPSData
{
    float latitude;
    float longitude;
    bool locationValid;
    int satellites;
    float altitude;
};

// Queue để truyền dữ liệu GPS
QueueHandle_t gpsQueue;

WiFiClient espClient;
PubSubClient client(espClient);
volatile uint8_t Interupt_Flag = 0;

// This function will be called when the BOOT pin transitions from LOW to HIGH (rising edge)
void bootInterruptHandler()
{
    Serial.println("Interupt ocur");
    Interupt_Flag = 1;
}

// Task đọc GPS
void gpsReadTask(void *parameter)
{
    while (true)
    {
        // Đọc dữ liệu GPS
        while (gpsSerial.available() > 0)
        {
            if (gps.encode(gpsSerial.read()))
            {
                if (gps.location.isValid())
                {
                    GPSData gpsData;
                    gpsData.latitude = gps.location.lat();
                    gpsData.longitude = gps.location.lng();
                    gpsData.locationValid = true;
                    gpsData.satellites = gps.satellites.value();
                    gpsData.altitude = gps.altitude.meters();

                    // Gửi dữ liệu GPS vào queue
                    if (xQueueSend(gpsQueue, &gpsData, pdMS_TO_TICKS(100)) == pdTRUE)
                    {
                        Serial.print("GPS - Lat: ");
                        Serial.print(gpsData.latitude, 6);
                        Serial.print(", Lng: ");
                        Serial.print(gpsData.longitude, 6);
                        Serial.print(", Satellites: ");
                        Serial.println(gpsData.satellites);
                    }
                }
            }
        }

        // Kiểm tra lỗi GPS
        if (millis() > 5000 && gps.charsProcessed() < 10)
        {
            Serial.println("No GPS signal!");
        }

        vTaskDelay(pdMS_TO_TICKS(1000)); // Kiểm tra GPS mỗi 1 giây
    }
}

void task3Function(void *parameter)
{
    while (true)
    {
        sys_capserver_proc();
        if (Interupt_Flag)
        {
            clearWiFiCredentials();
            Interupt_Flag = 0;
        }
        if (wifiState == WIFI_CONNECTED)
        {
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void task4Function(void *parameter)
{
    while (true)
    {
        if (wifiState == WIFI_CONFIGURED_NOT_CONNECTED)
        {
            pinMode(2, OUTPUT);
            digitalWrite(2, !digitalRead(2)); // Toggle the LED pin state
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void task5Function(void *parameter)
{
    while (true)
    {
        GPSData gpsData;
        bool hasGPSData = false;

        // Đọc dữ liệu GPS từ queue
        if (xQueueReceive(gpsQueue, &gpsData, pdMS_TO_TICKS(50)) == pdTRUE)
        {
            hasGPSData = true;
        }

        // Gửi dữ liệu lên MQTT nếu có WiFi
        if (hasGPSData && wifiState == WIFI_CONNECTED)
        {
            // Create a character array to hold the JSON string
            char jsonBuffer[256];
            for (int i = 0; i < 256; i++)
            {
                jsonBuffer[i] = 0;
            }

            // Tạo JSON với GPS data
            String jsonString = "{\"device\":\"ESP32_GPS\"";

            if (hasGPSData)
            {
                jsonString += ",\"latitude\":" + String(gpsData.latitude, 6);
                jsonString += ",\"longitude\":" + String(gpsData.longitude, 6);
                jsonString += ",\"satellites\":" + String(gpsData.satellites);
                jsonString += ",\"altitude\":" + String(gpsData.altitude, 2);
            }

            jsonString += "}";
            jsonString.toCharArray(jsonBuffer, sizeof(jsonBuffer));

            if (!client.connected())
            {
                client.connect("ESP32Client");
            }
            client.loop();

            if (client.publish(mqtt_topic, jsonBuffer))
            {
                Serial.println("Du lieu da gui len MQTT:");
                Serial.println(jsonBuffer);
            }
            else
            {
                Serial.println("Loi gui du lieu len MQTT!");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(2000)); // Gửi dữ liệu mỗi 2 giây
    }
}

void setup()
{
    pinMode(DE_PIN, OUTPUT);
    pinMode(RE_PIN, OUTPUT);
  
    Serial.begin(115200);  // Serial Monitor
    rs485.begin(9600, SERIAL_8N1, 16, 17);  // Baudrate 9600, RX=16, TX=17

    Serial.begin(9600);
    gpsSerial.begin(9600);

    Serial.println("Khoi tao GPS");

    sys_wifi_init();
    sys_capserver_init();
    pinMode(BOOT_PIN, INPUT_PULLUP);                                                // Configure BOOT pin as input with internal pull-up resistor
    attachInterrupt(digitalPinToInterrupt(BOOT_PIN), bootInterruptHandler, RISING); // Attach interrupt handler to rising edge of BOOT pin
    Serial.println("All Done!");
    client.setServer(mqtt_server, mqtt_port);

    // Create queue
    gpsQueue = xQueueCreate(3, sizeof(GPSData)); // Queue có thể chứa 3 phần tử GPSData
    if (gpsQueue == NULL)
    {
        Serial.println("Loi tao GPS queue!");
        while (1)
            ;
    }

    // Tạo các tasks
    xTaskCreate(
        gpsReadTask, // Task function (GPS reader)
        "GPS Task",  // Task name
        10000,       // Stack size (bytes)
        NULL,        // Task parameters
        2,           // Task priority
        NULL         // Task handle
    );

    xTaskCreate(
        task3Function, // Task function
        "Task 3",      // Task name
        10000,         // Stack size (bytes)
        NULL,          // Task parameters
        1,             // Task priority
        NULL           // Task handle
    );
    xTaskCreate(
        task4Function, // Task function
        "Task 4",      // Task name
        10000,         // Stack size (bytes)
        NULL,          // Task parameters
        1,             // Task priority
        NULL           // Task handle
    );
    xTaskCreate(
        task5Function, // Task function
        "Task 5",      // Task name
        10000,         // Stack size (bytes)
        NULL,          // Task parameters
        1,             // Task priority
        NULL           // Task handle
    );
}

void loop() {
  // Chuyển sang chế độ nhận
  digitalWrite(DE_PIN, LOW);
  digitalWrite(RE_PIN, LOW);
  
  // Đọc dữ liệu từ RS485
  if (rs485.available()) {
    String receivedData = rs485.readStringUntil('\n');
    receivedData.trim(); // Loại bỏ ký tự xuống dòng
    
    Serial.print("[RS485] Received from ATmega: ");
    Serial.println(receivedData);

    // Xử lý dữ liệu nhận được ở đây
    // Ví dụ: "TEMP:25.50,HUM:60.30" hoặc JSON
  }
  
  // Có thể thêm delay nếu cần
  delay(100);
}