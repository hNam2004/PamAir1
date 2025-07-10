#include <WiFi.h>
#include <HardwareSerial.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <PubSubClient.h>
#include <SoftwareSerial.h>
#include "sys_capserver.hpp"
#include "sys_wifi.hpp"
#include <TinyGPS++.h>
#include <Arduino.h>

const char *mqtt_ssid = "emqx";
const char *mqtt_password = "public";
const char *mqtt_server = "broker.emqx.io";
const int mqtt_port = 1883; // MQTT default port
const char *mqtt_topic = "esp32/test11";

#define BOOT_PIN 0
#define RXD2 13  // RO từ MAX485 (GPIO16)
#define TXD2 17  // DI đến MAX485 (GPIO17)
#define RE_DE 23 // Điều khiển gửi / nhận (GPIO4)

// Khai báo UART2 cho RS485
HardwareSerial RS485Serial(2);
SoftwareSerial gpsSerial(2, 3);
TinyGPSPlus gps;
// Cấu trúc dữ liệu cảm biến nhận từ RS485
struct SensorData
{
    float temperature;
    float humidity;
    bool dataValid;
};

// Cấu trúc dữ liệu GPS
struct GPSData
{
    float latitude;
    float longitude;
    bool locationValid;
    int satellites;
    float altitude;
};

// Queue để truyền dữ liệu cảm biến và GPS
QueueHandle_t sensorQueue;
QueueHandle_t gpsQueue;

WiFiClient espClient;
PubSubClient client(espClient);
volatile uint8_t Interupt_Flag = 0;

// Hàm parse dữ liệu cảm biến từ chuỗi
SensorData parseSensorData(String data)
{
    SensorData sensorData;
    sensorData.dataValid = false;

    // Ví dụ format: "TEMP:25.50,HUM:60.30"
    // Hoặc format: "25.50,60.30"
    // Hoặc format JSON: {"temp":25.50,"hum":60.30}

    int tempIndex = data.indexOf("TEMP:");
    int humIndex = data.indexOf("HUM:");

    if (tempIndex >= 0 && humIndex >= 0)
    {
        // Format: "TEMP:25.50,HUM:60.30"
        String tempStr = data.substring(tempIndex + 5, data.indexOf(",", tempIndex));
        String humStr = data.substring(humIndex + 4);

        sensorData.temperature = tempStr.toFloat();
        sensorData.humidity = humStr.toFloat();
        sensorData.dataValid = true;
    }
    else if (data.indexOf(",") > 0)
    {
        // Format: "25.50,60.30"
        int commaIndex = data.indexOf(",");
        String tempStr = data.substring(0, commaIndex);
        String humStr = data.substring(commaIndex + 1);

        sensorData.temperature = tempStr.toFloat();
        sensorData.humidity = humStr.toFloat();
        sensorData.dataValid = true;
    }

    return sensorData;
}

// Hàm nhận dữ liệu từ RS485
void processRS485Command()
{
    if (RS485Serial.available())
    {
        String receivedData = RS485Serial.readString();
        receivedData.trim(); // Loại bỏ ký tự xuống dòng

        Serial.print("Nhan du lieu RS485: ");
        Serial.println(receivedData);

        // Parse dữ liệu cảm biến
        if (receivedData.length() > 0)
        {
            SensorData sensorData = parseSensorData(receivedData);

            if (sensorData.dataValid)
            {
                // Gửi dữ liệu vào queue
                if (xQueueSend(sensorQueue, &sensorData, pdMS_TO_TICKS(100)) == pdTRUE)
                {
                    Serial.print("Nhiet do: ");
                    Serial.print(sensorData.temperature);
                    Serial.print(" °C  |  Do am: ");
                    Serial.print(sensorData.humidity);
                    Serial.println(" %");
                    Serial.println("Du lieu da gui vao queue thanh cong");
                }
                else
                {
                    Serial.println("Khong the gui du lieu vao queue!");
                }
            }
            else
            {
                Serial.println("Du lieu khong hop le!");
            }
        }
    }
}

// This function will be called when the BOOT pin transitions from LOW to HIGH (rising edge)
void bootInterruptHandler()
{
    Serial.println("Interupt ocur");
    Interupt_Flag = 1;
}

// Task function for Task 1
// Task đọc cảm biến HDC1000
void task1Function(void *parameter)
{
    while (true)
    {
        // Kiểm tra và xử lý dữ liệu từ RS485
        processRS485Command();

        vTaskDelay(pdMS_TO_TICKS(100)); // Kiểm tra mỗi 100ms
    }
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
        SensorData sensorData;
        GPSData gpsData;
        bool hasSensorData = false;
        bool hasGPSData = false;

        // Đọc dữ liệu cảm biến từ queue
        if (xQueueReceive(sensorQueue, &sensorData, pdMS_TO_TICKS(50)) == pdTRUE)
        {
            hasSensorData = true;
        }

        // Đọc dữ liệu GPS từ queue
        if (xQueueReceive(gpsQueue, &gpsData, pdMS_TO_TICKS(50)) == pdTRUE)
        {
            hasGPSData = true;
        }

        // Gửi dữ liệu lên MQTT nếu có WiFi
        if ((hasSensorData || hasGPSData) && wifiState == WIFI_CONNECTED)
        {
            // Create a character array to hold the JSON string
            char jsonBuffer[256];
            for (int i = 0; i < 256; i++)
            {
                jsonBuffer[i] = 0;
            }

            // Tạo JSON với cả sensor và GPS data
            String jsonString = "{\"device\":\"ESP32_RS485_HDC1080\"";

            if (hasSensorData)
            {
                jsonString += ",\"temperature\":" + String(sensorData.temperature, 2);
                jsonString += ",\"humidity\":" + String(sensorData.humidity, 2);
            }

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
    Serial.begin(9600);
    gpsSerial.begin(9600);
    // Khởi tạo RS485
    pinMode(RE_DE, OUTPUT);
    digitalWrite(RE_DE, LOW);                        // Mặc định là chế độ nhận
    RS485Serial.begin(9600, SERIAL_8N1, RXD2, TXD2); // UART2
    Serial.println("RS485 da khoi tao");

    Serial.println("Cam bien HDC1080 nhan du lieu qua RS485");

    sys_wifi_init();
    sys_capserver_init();
    pinMode(BOOT_PIN, INPUT_PULLUP);                                                // Configure BOOT pin as input with internal pull-up resistor
    attachInterrupt(digitalPinToInterrupt(BOOT_PIN), bootInterruptHandler, RISING); // Attach interrupt handler to rising edge of BOOT pin
    Serial.println("All Done!");
    client.setServer(mqtt_server, mqtt_port);

    // Create queues
    sensorQueue = xQueueCreate(5, sizeof(SensorData)); // Queue có thể chứa 5 phần tử SensorData
    if (sensorQueue == NULL)
    {
        Serial.println("Loi tao sensor queue!");
        while (1)
            ;
    }

    gpsQueue = xQueueCreate(3, sizeof(GPSData)); // Queue có thể chứa 3 phần tử GPSData
    if (gpsQueue == NULL)
    {
        Serial.println("Loi tao GPS queue!");
        while (1)
            ;
    }

    // Tạo các tasks
    xTaskCreate(
        task1Function, // Task function (RS485 handler)
        "Task 1",      // Task name
        10000,         // Stack size (bytes)
        NULL,          // Task parameters
        2,             // Task priority
        NULL           // Task handle
    );

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

void loop()
{
}
