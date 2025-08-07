#include <WiFi.h>
#include <HardwareSerial.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <PubSubClient.h>
#include "sys_capserver.hpp"
#include "sys_wifi.hpp"
#include <esp_task_wdt.h> // Thư viện để quản lý WDT

const char *mqtt_ssid = "emqx";
const char *mqtt_password = "public";
const char *mqtt_server = "broker.emqx.io";
const int mqtt_port = 1883;
const char *mqtt_topic_hdc1080 = "esp32/test11/hdc1080";
const char *mqtt_topic_sps30 = "esp32/test11/sps30";

#define BOOT_PIN 0
#define LED_BUILTIN 2
#define RXD2 3
#define TXD2 1
#define RE_DE 13
#define MOSFET_PIN 23
#define WDT_TIMEOUT 10 // Timeout WDT 10 giây

struct SensorData
{
    bool hasHDC1080Data;
    bool hasSPS30Data;
    float temperature;
    float humidity;
    float pm1_0;
    float pm2_5;
    float pm4_0;
    float pm10_0;
    unsigned long timestamp;
};

HardwareSerial RS485Serial(2);
char rs485Buffer[512];
int rs485BufferLen = 0;
SensorData sensorData = {false, false, 0, 0, 0, 0, 0, 0, 0};
QueueHandle_t sensorDataQueue;

WiFiClient espClient;
PubSubClient client(espClient);
volatile uint8_t Interupt_Flag = 0;

void bootInterruptHandler()
{
    Serial.println("Interrupt occurred");
    Interupt_Flag = 1;
}

void task1_WiFiManagement(void *parameter)
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

void task2_RS485Reading(void *parameter)
{
    esp_task_wdt_add(NULL);
    while (true)
    {
        if (wifiState == WIFI_CONFIGURED_NOT_CONNECTED)
        {
            pinMode(2, OUTPUT);
            digitalWrite(2, !digitalRead(2));
        }
        while (RS485Serial.available())
        {
            char c = RS485Serial.read();
            if (rs485BufferLen < sizeof(rs485Buffer) - 1)
            {
                rs485Buffer[rs485BufferLen++] = c;
                rs485Buffer[rs485BufferLen] = '\0';
                if (c == '\n')
                {
                    Serial.print("RS485 Buffer: ");
                    Serial.println(rs485Buffer);
                }
            }
            else
            {
                Serial.println("Warning: RS485 buffer full, clearing...");
                rs485BufferLen = 0;
                rs485Buffer[0] = '\0';
            }
        }
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void task3_HDC1080Processing(void *parameter)
{
    esp_task_wdt_add(NULL);
    while (true)
    {
        char *startHDC = strstr(rs485Buffer, "<HDC1080>");
        char *endHDC = strstr(rs485Buffer, "</HDC1080>");
        if (startHDC && endHDC && endHDC > startHDC)
        {
            int startIndexHDC = startHDC - rs485Buffer;
            int endIndexHDC = endHDC - rs485Buffer + 10;
            char receivedData[128];
            int len = endHDC - (startHDC + 9);
            if (len < sizeof(receivedData) - 1)
            {
                strncpy(receivedData, startHDC + 9, len);
                receivedData[len] = '\0';
                Serial.print("HDC1080 Received: ");
                Serial.println(receivedData);

                char *tempPtr = strstr(receivedData, "TEMP:");
                char *humPtr = strstr(receivedData, "HUM:");
                if (tempPtr && humPtr)
                {
                    float temp = atof(tempPtr + 5);
                    float hum = atof(humPtr + 4);
                    sensorData.temperature = temp;
                    sensorData.humidity = hum;
                    sensorData.hasHDC1080Data = true;
                    sensorData.hasSPS30Data = false;
                    sensorData.timestamp = millis();
                    if (xQueueSend(sensorDataQueue, &sensorData, 0) == pdTRUE)
                    {
                        Serial.println("HDC1080 data sent to queue");
                    }
                    else
                    {
                        Serial.println("Failed to send HDC1080 data to queue");
                    }
                    Serial.printf("HDC1080 - Temp: %.2f°C, Hum: %.2f%%\n", temp, hum);
                }
                else
                {
                    Serial.println("Invalid HDC1080 data format");
                }

                memmove(rs485Buffer, rs485Buffer + endIndexHDC, rs485BufferLen - endIndexHDC + 1);
                rs485BufferLen -= endIndexHDC;
            }
        }
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void task4_SPS30Processing(void *parameter)
{
    esp_task_wdt_add(NULL);
    while (true)
    {
        char *startSPS = strstr(rs485Buffer, "<SPS30>");
        char *endSPS = strstr(rs485Buffer, "</SPS30>");
        if (startSPS && endSPS && endSPS > startSPS)
        {
            int startIndexSPS = startSPS - rs485Buffer;
            int endIndexSPS = endSPS - rs485Buffer + 8;
            char receivedData[256];
            int len = endSPS - (startSPS + 7);
            if (len < sizeof(receivedData) - 1)
            {
                strncpy(receivedData, startSPS + 7, len);
                receivedData[len] = '\0';
                Serial.print("SPS30 Received: ");
                Serial.println(receivedData);

                char *pm1Ptr = strstr(receivedData, "PM1:");
                char *pm25Ptr = strstr(receivedData, "PM2.5:");
                char *pm4Ptr = strstr(receivedData, "PM4:");
                char *pm10Ptr = strstr(receivedData, "PM10:");
                if (pm1Ptr && pm25Ptr && pm4Ptr && pm10Ptr)
                {
                    sensorData.pm1_0 = atof(pm1Ptr + 4);
                    sensorData.pm2_5 = atof(pm25Ptr + 5);
                    sensorData.pm4_0 = atof(pm4Ptr + 4);
                    sensorData.pm10_0 = atof(pm10Ptr + 5);
                    sensorData.hasSPS30Data = true;
                    sensorData.hasHDC1080Data = false;
                    sensorData.timestamp = millis();
                    if (xQueueSend(sensorDataQueue, &sensorData, 0) == pdTRUE)
                    {
                        Serial.println("SPS30 data sent to queue");
                    }
                    else
                    {
                        Serial.println("Failed to send SPS30 data to queue");
                    }
                    Serial.printf("SPS30 - PM1.0: %.2f, PM2.5: %.2f, PM4.0: %.2f, PM10.0: %.2f\n",
                                  sensorData.pm1_0, sensorData.pm2_5, sensorData.pm4_0, sensorData.pm10_0);
                }
                else
                {
                    Serial.println("Invalid SPS30 data format");
                }

                memmove(rs485Buffer, rs485Buffer + endIndexSPS, rs485BufferLen - endIndexSPS + 1);
                rs485BufferLen -= endIndexSPS;
            }
        }
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void task5_MQTTPublishing(void *parameter)
{
    esp_task_wdt_add(NULL);
    SensorData receivedData;
    while (true)
    {
        TickType_t startTime = xTaskGetTickCount();
        if (xQueueReceive(sensorDataQueue, &receivedData, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            Serial.println("Received data from queue");
            if (wifiState == WIFI_CONNECTED && WiFi.status() == WL_CONNECTED)
            {
                if (!client.connected())
                {
                    Serial.println("Attempting MQTT connection...");
                    espClient.stop();
                    if (client.connect("ESP32Client", mqtt_ssid, mqtt_password))
                    {
                        Serial.println("MQTT connected");
                    }
                    else
                    {
                        Serial.print("MQTT connection failed, rc=");
                        Serial.println(client.state());
                        esp_task_wdt_reset();
                        vTaskDelay(pdMS_TO_TICKS(2000));
                        continue;
                    }
                }

                if (!espClient.connected())
                {
                    Serial.println("WiFiClient not connected, skipping MQTT publish");
                    espClient.stop();
                    client.disconnect();
                    esp_task_wdt_reset();
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    continue;
                }

                client.loop();

                if (receivedData.hasHDC1080Data)
                {
                    char jsonBuffer[128];
                    snprintf(jsonBuffer, sizeof(jsonBuffer),
                             "{\"device\":\"HDC1080\",\"t\":%.2f,\"h\":%.2f,\"ts\":%lu}",
                             receivedData.temperature, receivedData.humidity, receivedData.timestamp);
                    Serial.print("HDC1080 JSON length: ");
                    Serial.println(strlen(jsonBuffer));
                    if (client.publish(mqtt_topic_hdc1080, jsonBuffer))
                    {
                        Serial.print("MQTT Published HDC1080: ");
                        Serial.println(jsonBuffer);
                    }
                    else
                    {
                        Serial.println("MQTT publish HDC1080 failed");
                        espClient.stop();
                        client.disconnect();
                    }
                }

                if (receivedData.hasSPS30Data)
                {
                    char jsonBuffer[128];
                    snprintf(jsonBuffer, sizeof(jsonBuffer),
                             "{\"device\":\"SPS30\",\"p1\":%.2f,\"p2\":%.2f,\"p4\":%.2f,\"p10\":%.2f,\"ts\":%lu}",
                             receivedData.pm1_0, receivedData.pm2_5, receivedData.pm4_0, receivedData.pm10_0, receivedData.timestamp);
                    Serial.print("SPS30 JSON length: ");
                    Serial.println(strlen(jsonBuffer));
                    if (client.publish(mqtt_topic_sps30, jsonBuffer))
                    {
                        Serial.print("MQTT Published SPS30: ");
                        Serial.println(jsonBuffer);
                    }
                    else
                    {
                        Serial.println("MQTT publish SPS30 failed");
                        espClient.stop();
                        client.disconnect();
                    }
                }
            }
            else
            {
                Serial.println("WiFi not connected, skipping MQTT publish");
            }
        }
        else
        {
            Serial.println("No data in queue");
        }

        // Reset WDT và kiểm tra thời gian thực hiện
        esp_task_wdt_reset();
        TickType_t elapsed = xTaskGetTickCount() - startTime;
        if (elapsed > pdMS_TO_TICKS(500))
        {
            Serial.print("Warning: MQTT Publishing task took too long: ");
            Serial.print(elapsed * portTICK_PERIOD_MS);
            Serial.println(" ms");
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void task6Function(void *parameter)
{
    esp_task_wdt_add(NULL);
    while (true)
    {
        if (wifiState == WIFI_CONFIGURED_NOT_CONNECTED)
        {
            pinMode(2, OUTPUT);
            digitalWrite(2, !digitalRead(2));
        }
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void setup()
{
    Serial.begin(9600);
    delay(1000);
    Serial.println("=== ESP32 STARTING ===");
    esp_task_wdt_init(WDT_TIMEOUT, true); // Khởi tạo WDT với timeout 10 giây
    pinMode(RE_DE, OUTPUT);
    digitalWrite(RE_DE, LOW);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    pinMode(MOSFET_PIN, OUTPUT);
    digitalWrite(MOSFET_PIN, HIGH);
    RS485Serial.begin(115200, SERIAL_8N1, RXD2, -1);
    Serial.println("RS485 ready (receive-only mode)");
    sys_wifi_init();
    sys_capserver_init();
    pinMode(BOOT_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(BOOT_PIN), bootInterruptHandler, RISING);
    client.setServer(mqtt_server, mqtt_port);
    client.setKeepAlive(60);
    client.setSocketTimeout(60);
    Serial.println("MQTT ready");
    sensorDataQueue = xQueueCreate(10, sizeof(SensorData));
    if (sensorDataQueue == NULL)
    {
        Serial.println("Failed to create sensor data queue");
    }
    Serial.println("Creating tasks...");
    xTaskCreate(task1_WiFiManagement, "WiFi Management", 10000, NULL, 1, NULL);
    xTaskCreate(task2_RS485Reading, "RS485 Reading", 10000, NULL, 1, NULL);
    xTaskCreate(task3_HDC1080Processing, "HDC1080 Processing", 10000, NULL, 1, NULL);
    xTaskCreate(task4_SPS30Processing, "SPS30 Processing", 10000, NULL, 1, NULL);
    xTaskCreate(task5_MQTTPublishing, "MQTT Publishing", 10000, NULL, 1, NULL);
    xTaskCreate(task6Function, "wifi_test", 10000, NULL, 1, NULL);
    Serial.println("All tasks created successfully!");
}

void loop()
{
}