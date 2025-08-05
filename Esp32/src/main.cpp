#include <WiFi.h>
#include <HardwareSerial.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <PubSubClient.h>
#include "sys_capserver.hpp"
#include "sys_wifi.hpp"

const char *mqtt_ssid = "emqx";
const char *mqtt_password = "public";
const char *mqtt_server = "broker.emqx.io";
const int mqtt_port = 1883; // MQTT default port
const char *mqtt_topic = "esp32/test11";

// RS485 and sensor definitions
#define BOOT_PIN 0
#define LED_BUILTIN 2
#define RXD2 3        // MAX485 RO → ESP32 RX
#define TXD2 1        // MAX485 DI ← ESP32 TX
#define RE_DE 13      // MAX485 RE/DE
#define MOSFET_PIN 23 // GPIO23 điều khiển IRF4435

// Data structures for sensor data sharing between tasks
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

// Global variables
HardwareSerial RS485Serial(2);
String rs485Buffer = "";
SensorData sensorData = {false, false, 0, 0, 0, 0, 0, 0, 0};
QueueHandle_t sensorDataQueue;

WiFiClient espClient;
PubSubClient client(espClient);
volatile uint8_t Interupt_Flag = 0;

// This function will be called when the BOOT pin transitions from LOW to HIGH (rising edge)
void bootInterruptHandler()
{
    Serial.println("Interupt ocur");
    Interupt_Flag = 1;
}

// RS485 is configured for receive-only mode
// No sending functions needed

// Task 1: WiFi Connection Management
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

        // Additional WiFi management logic can be added here
        if (wifiState == WIFI_CONNECTED)
        {
            // WiFi is connected, ensure MQTT connection
            if (!client.connected())
            {
                Serial.println("MQTT disconnected, attempting reconnect...");
                if (client.connect("ESP32Client"))
                {
                    Serial.println("MQTT reconnected");
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Task 2: RS485 Data Reading
void task2_RS485Reading(void *parameter)
{
    while (true)
    {
        // LED status indication
        if (wifiState == WIFI_CONFIGURED_NOT_CONNECTED)
        {
            pinMode(2, OUTPUT);
            digitalWrite(2, !digitalRead(2)); // Toggle the LED pin state
        }

        // Read RS485 data
        if (RS485Serial.available())
        {
            String newData = RS485Serial.readStringUntil('\n');
            rs485Buffer += newData;

            // Limit buffer size to prevent memory overflow
            if (rs485Buffer.length() > 500)
            {
                rs485Buffer = rs485Buffer.substring(rs485Buffer.length() - 200);
            }

            Serial.print("RS485 Buffer: ");
            Serial.println(rs485Buffer);
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // Check RS485 more frequently
    }
}

// Task 3: HDC1080 Data Processing
void task3_HDC1080Processing(void *parameter)
{
    while (true)
    {
        // Process HDC1080 data from RS485 buffer
        int startIndexHDC = rs485Buffer.indexOf("<HDC1080>");
        int endIndexHDC = rs485Buffer.indexOf("</HDC1080>");

        if (startIndexHDC >= 0 && endIndexHDC >= 0 && endIndexHDC > startIndexHDC)
        {
            String receivedData = rs485Buffer.substring(startIndexHDC + 9, endIndexHDC);
            receivedData.trim();
            rs485Buffer = rs485Buffer.substring(endIndexHDC + 10);

            if (receivedData.length() > 0)
            {
                Serial.print("HDC1080 Received: ");
                Serial.println(receivedData);

                // Parse temperature and humidity
                int tempIndex = receivedData.indexOf("TEMP:");
                int humIndex = receivedData.indexOf("HUM:");

                if (tempIndex >= 0 && humIndex >= 0)
                {
                    String tempStr = receivedData.substring(tempIndex + 5, receivedData.indexOf(",", tempIndex));
                    String humStr = receivedData.substring(humIndex + 4);
                    if (humStr.indexOf(",") >= 0)
                    {
                        humStr = humStr.substring(0, humStr.indexOf(","));
                    }

                    // Update sensor data
                    sensorData.temperature = tempStr.toFloat();
                    sensorData.humidity = humStr.toFloat();
                    sensorData.hasHDC1080Data = true;
                    sensorData.timestamp = millis();

                    // Send to queue for MQTT task
                    xQueueSend(sensorDataQueue, &sensorData, 0);

                    Serial.printf("HDC1080 - Temp: %.2f°C, Hum: %.2f%%\n",
                                  sensorData.temperature, sensorData.humidity);
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// Task 4: SPS30 Data Processing
void task4_SPS30Processing(void *parameter)
{
    while (true)
    {
        // Process SPS30 data from RS485 buffer
        int startIndexSPS = rs485Buffer.indexOf("<SPS30>");
        int endIndexSPS = rs485Buffer.indexOf("</SPS30>");

        if (startIndexSPS >= 0 && endIndexSPS >= 0 && endIndexSPS > startIndexSPS)
        {
            String receivedData = rs485Buffer.substring(startIndexSPS + 7, endIndexSPS);
            receivedData.trim();
            rs485Buffer = rs485Buffer.substring(endIndexSPS + 8);

            if (receivedData.length() > 0)
            {
                Serial.print("SPS30 Received: ");
                Serial.println(receivedData);

                // Parse PM values
                int pm1Index = receivedData.indexOf("PM1:");
                int pm25Index = receivedData.indexOf("PM2.5:");
                int pm4Index = receivedData.indexOf("PM4:");
                int pm10Index = receivedData.indexOf("PM10:");

                if (pm1Index >= 0 && pm25Index >= 0 && pm4Index >= 0 && pm10Index >= 0)
                {
                    String pm1Str = receivedData.substring(pm1Index + 4, receivedData.indexOf(",", pm1Index));
                    String pm25Str = receivedData.substring(pm25Index + 5, receivedData.indexOf(",", pm25Index));
                    String pm4Str = receivedData.substring(pm4Index + 4, receivedData.indexOf(",", pm4Index));
                    String pm10Str = receivedData.substring(pm10Index + 5, receivedData.indexOf(",", pm10Index));

                    // Update sensor data
                    sensorData.pm1_0 = pm1Str.toFloat();
                    sensorData.pm2_5 = pm25Str.toFloat();
                    sensorData.pm4_0 = pm4Str.toFloat();
                    sensorData.pm10_0 = pm10Str.toFloat();
                    sensorData.hasSPS30Data = true;
                    sensorData.timestamp = millis();

                    // Send to queue for MQTT task
                    xQueueSend(sensorDataQueue, &sensorData, 0);

                    Serial.printf("SPS30 - PM1.0: %.2f, PM2.5: %.2f, PM4.0: %.2f, PM10.0: %.2f\n",
                                  sensorData.pm1_0, sensorData.pm2_5, sensorData.pm4_0, sensorData.pm10_0);
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// Task 5: MQTT Publishing
void task5_MQTTPublishing(void *parameter)
{
    SensorData receivedData;

    while (true)
    {
        // Read data from the queue
        if (xQueueReceive(sensorDataQueue, &receivedData, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            if (wifiState == WIFI_CONNECTED)
            {
                // Ensure MQTT connection
                if (!client.connected())
                {
                    if (client.connect("ESP32Client"))
                    {
                        Serial.println("MQTT connected");
                    }
                    else
                    {
                        Serial.println("MQTT connection failed");
                        vTaskDelay(pdMS_TO_TICKS(1000));
                        continue;
                    }
                }

                client.loop();

                // Create JSON for HDC1080 data
                if (receivedData.hasHDC1080Data)
                {
                    char jsonBuffer[256];
                    snprintf(jsonBuffer, sizeof(jsonBuffer),
                             "{\"device\":\"ESP32_RS485_HDC1080\",\"temperature\":%.2f,\"humidity\":%.2f,\"timestamp\":%lu}",
                             receivedData.temperature, receivedData.humidity, receivedData.timestamp);

                    client.publish(mqtt_topic, jsonBuffer);
                    Serial.print("MQTT Published HDC1080: ");
                    Serial.println(jsonBuffer);
                }

                // Create JSON for SPS30 data
                if (receivedData.hasSPS30Data)
                {
                    char jsonBuffer[256];
                    snprintf(jsonBuffer, sizeof(jsonBuffer),
                             "{\"device\":\"ESP32_RS485_SPS30\",\"pm1_0\":%.2f,\"pm2_5\":%.2f,\"pm4_0\":%.2f,\"pm10_0\":%.2f,\"timestamp\":%lu}",
                             receivedData.pm1_0, receivedData.pm2_5, receivedData.pm4_0, receivedData.pm10_0, receivedData.timestamp);

                    client.publish(mqtt_topic, jsonBuffer);
                    Serial.print("MQTT Published SPS30: ");
                    Serial.println(jsonBuffer);
                }
            }
            else
            {
                Serial.println("WiFi not connected, skipping MQTT publish");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void setup()
{
    Serial.begin(9600);
    delay(1000);
    Serial.println("=== ESP32 STARTING ===");

    // Initialize hardware
    pinMode(RE_DE, OUTPUT);
    digitalWrite(RE_DE, LOW); // Always in receive mode
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    pinMode(MOSFET_PIN, OUTPUT);
    digitalWrite(MOSFET_PIN, HIGH); // Always keep MOSFET on (P-Channel)

    // Initialize RS485 in receive-only mode
    RS485Serial.begin(115200, SERIAL_8N1, RXD2, -1);
    Serial.println("RS485 ready (receive-only mode)");

    // Initialize WiFi and captive portal
    sys_wifi_init();
    sys_capserver_init();

    // Initialize BOOT pin interrupt
    pinMode(BOOT_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(BOOT_PIN), bootInterruptHandler, RISING);

    // Initialize MQTT
    client.setServer(mqtt_server, mqtt_port);
    Serial.println("MQTT ready");

    // Create queue for sensor data communication between tasks
    sensorDataQueue = xQueueCreate(10, sizeof(SensorData));
    if (sensorDataQueue == NULL)
    {
        Serial.println("Failed to create sensor data queue");
    }

    Serial.println("Creating tasks...");

    // Create Task 1: WiFi Management
    xTaskCreate(
        task1_WiFiManagement, // Task function
        "WiFi Management",    // Task name
        10000,                // Stack size (bytes)
        NULL,                 // Task parameters
        2,                    // Task priority (higher for WiFi)
        NULL                  // Task handle
    );

    // Create Task 2: RS485 Data Reading
    xTaskCreate(
        task2_RS485Reading, // Task function
        "RS485 Reading",    // Task name
        10000,              // Stack size (bytes)
        NULL,               // Task parameters
        3,                  // Task priority (highest for data reading)
        NULL                // Task handle
    );

    // Create Task 3: HDC1080 Data Processing
    xTaskCreate(
        task3_HDC1080Processing, // Task function
        "HDC1080 Processing",    // Task name
        10000,                   // Stack size (bytes)
        NULL,                    // Task parameters
        2,                       // Task priority
        NULL                     // Task handle
    );

    // Create Task 4: SPS30 Data Processing
    xTaskCreate(
        task4_SPS30Processing, // Task function
        "SPS30 Processing",    // Task name
        10000,                 // Stack size (bytes)
        NULL,                  // Task parameters
        2,                     // Task priority
        NULL                   // Task handle
    );

    // Create Task 5: MQTT Publishing
    xTaskCreate(
        task5_MQTTPublishing, // Task function
        "MQTT Publishing",    // Task name
        10000,                // Stack size (bytes)
        NULL,                 // Task parameters
        1,                    // Task priority (lower for publishing)
        NULL                  // Task handle
    );

    Serial.println("All tasks created successfully!");
}

void loop()
{
    // Handle Serial commands for debugging
    if (Serial.available())
    {
        String command = Serial.readStringUntil('\n');
        command.trim();

        if (command == "status")
        {
            // Display system status
            Serial.println("=== ESP32 TASK-BASED STATUS ===");
            Serial.print("WiFi: ");
            Serial.println(wifiState == WIFI_CONNECTED ? "Connected" : "Disconnected");
            Serial.print("MQTT: ");
            Serial.println(client.connected() ? "Connected" : "Disconnected");
            Serial.print("RS485 Buffer Length: ");
            Serial.println(rs485Buffer.length());
            Serial.print("Free Heap: ");
            Serial.println(ESP.getFreeHeap());
            Serial.print("Queue Messages: ");
            Serial.println(uxQueueMessagesWaiting(sensorDataQueue));
        }
        else if (command == "help")
        {
            Serial.println("=== Available Commands ===");
            Serial.println("status - Show system status");
            Serial.println("help - Show this help");
            Serial.println("Note: RS485 is in receive-only mode");
        }
    }

    // Small delay to prevent watchdog issues
    delay(100);
}