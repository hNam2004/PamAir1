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



#define BOOT_PIN 0

WiFiClient espClient;
PubSubClient client(espClient);
volatile uint8_t Interupt_Flag = 0;


// This function will be called when the BOOT pin transitions from LOW to HIGH (rising edge)
void bootInterruptHandler()
{
    Serial.println("Interupt ocur");
    Interupt_Flag = 1;
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
        
        // Read data from the queue and remove it
        
        
        if (wifiState == WIFI_CONNECTED)
        {
            // Create a character array to hold the JSON string
            char jsonBuffer[128];
            for(int i=0;i<128;i++){
                jsonBuffer[i] = 0;
            }
            // Format the sensor data into the JSON string
            
            if (!client.connected())
            {
                client.connect("ESP32Client");
            }
            client.loop();
            client.publish(mqtt_topic, jsonBuffer);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void setup()
{
    Serial.begin(9600);
    sys_wifi_init();
    sys_capserver_init();
    pinMode(BOOT_PIN, INPUT_PULLUP);                                                // Configure BOOT pin as input with internal pull-up resistor
    attachInterrupt(digitalPinToInterrupt(BOOT_PIN), bootInterruptHandler, RISING); // Attach interrupt handler to rising edge of BOOT pin
    Serial.println("All Done!");
    client.setServer(mqtt_server, mqtt_port);
    // Create queues

    
    
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
