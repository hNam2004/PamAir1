#include <WiFi.h>
#include <HardwareSerial.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <PubSubClient.h>
#include <Wire.h>
#include "ClosedCube_HDC1080.h"
#include "sys_capserver.hpp"
#include "sys_wifi.hpp"

const char *mqtt_ssid = "emqx";
const char *mqtt_password = "public";
const char *mqtt_server = "broker.emqx.io";
const int mqtt_port = 1883; // MQTT default port
const char *mqtt_topic = "esp32/test11";

// Cảm biến HDC1080
ClosedCube_HDC1080 hdc;

// Cấu trúc dữ liệu cảm biến
struct SensorData
{
    float temperature;
    float humidity;
};

#define BOOT_PIN 0

WiFiClient espClient;
PubSubClient client(espClient);
volatile uint8_t Interupt_Flag = 0;

// Queue để truyền dữ liệu cảm biến
QueueHandle_t sensorQueue;

// This function will be called when the BOOT pin transitions from LOW to HIGH (rising edge)
void bootInterruptHandler()
{
    Serial.println("Interupt ocur");
    Interupt_Flag = 1;
}

// Task function để đọc cảm biến HDC1080
void task1Function(void *parameter)
{
    while (true)
    {
        // Đọc dữ liệu từ cảm biến
        float temperature = hdc.readTemperature();
        float humidity = hdc.readHumidity();

        // Kiểm tra dữ liệu hợp lệ
        if (!isnan(temperature) && !isnan(humidity))
        {
            // Tạo struct dữ liệu cảm biến
            SensorData sensorData;
            sensorData.temperature = temperature;
            sensorData.humidity = humidity;

            // Gửi dữ liệu vào queue
            if (xQueueSend(sensorQueue, &sensorData, pdMS_TO_TICKS(100)) == pdTRUE)
            {
                Serial.print("Nhiet do: ");
                Serial.print(temperature);
                Serial.print(" °C  |  Do am: ");
                Serial.print(humidity);
                Serial.println(" %");
            }
            else
            {
                Serial.println("Khong the gui du lieu vao queue!");
            }
        }
        else
        {
            Serial.println("Loi doc cam bien HDC1080!");
        }

        vTaskDelay(pdMS_TO_TICKS(2000)); // Đọc cảm biến mỗi 2 giây
    }
}

// Task function for Task 1

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
        SensorData receivedData;

        // Read data from the queue and remove it
        if (xQueueReceive(sensorQueue, &receivedData, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            if (wifiState == WIFI_CONNECTED)
            {
                // Create a character array to hold the JSON string
                char jsonBuffer[128];
                for (int i = 0; i < 128; i++)
                {
                    jsonBuffer[i] = 0;
                }

                // Format the sensor data into the JSON string
                snprintf(jsonBuffer, sizeof(jsonBuffer),
                         "{\"temperature\":%.2f,\"humidity\":%.2f,\"device\":\"ESP32_HDC1080\"}",
                         receivedData.temperature, receivedData.humidity);

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
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void setup()
{
    Serial.begin(9600);

    // Khởi tạo I2C và cảm biến HDC1080
    Wire.begin();
    hdc.begin(0x40); // Địa chỉ I2C mặc định của HDC1080

    // Kiểm tra cảm biến
    uint16_t manufacturerID = hdc.readManufacturerId();
    if (manufacturerID != 0x5449) // Texas Instruments ID
    {
        Serial.println("Không tìm thấy cảm biến HDC1080!");
        Serial.print("Manufacturer ID: 0x");
        Serial.println(manufacturerID, HEX);
        while (1)
            ; // Dừng nếu không tìm thấy
    }
    Serial.println("HDC1080 bắt đầu hoạt động");

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
        Serial.println("Loi tao queue!");
        while (1)
            ;
    }

    // Tạo task đọc cảm biến
    xTaskCreate(
        task1Function, // Task function
        "Task 1",      // Task name
        10000,         // Stack size (bytes)
        NULL,          // Task parameters
        2,             // Task priority (cao hơn các task khác)
        NULL           // Task handle
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
