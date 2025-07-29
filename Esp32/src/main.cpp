#include <WiFi.h>
#include <HardwareSerial.h>
#include <PubSubClient.h>
#include <Arduino.h>
#include "sys_eeprom.hpp"
#include "sys_wifi.hpp"

const char *mqtt_ssid = "emqx";
const char *mqtt_password = "public";
const char *mqtt_server = "broker.emqx.io";
const int mqtt_port = 1883; // MQTT default port
const char *mqtt_topic = "esp32/test11";

#define BOOT_PIN 0
#define LED_BUILTIN 2 // Built-in LED trên ESP32
#define RXD2 34       // MAX485 RO → ESP32 RX (GPIO34)
#define TXD2 35       // MAX485 DI ← ESP32 TX (GPIO35)
#define RE_DE 13      // MAX485 RE/DE ← ESP32 (GPIO4)
#define MOSFET_PIN 23 // GPIO23 dùng để điều khiển IRF4435
// TX_PIN đã xóa vì chỉ nhận dữ liệu

HardwareSerial RS485Serial(2);

WiFiClient espClient;
PubSubClient client(espClient);

String rs485Buffer = "";
void processRS485AndSendMQTT()
{
    if (RS485Serial.available())
    {
        // Bật MOSFET IRF4435 trước khi xử lý dữ liệu
        digitalWrite(MOSFET_PIN, HIGH);
        String newData = RS485Serial.readString();
        rs485Buffer += newData;
        // HDC1080
        int startIndexHDC = rs485Buffer.indexOf("<HDC1080>");
        int endIndexHDC = rs485Buffer.indexOf("</HDC1080>");
        if (startIndexHDC >= 0 && endIndexHDC >= 0 && endIndexHDC > startIndexHDC)
        {
            String receivedData = rs485Buffer.substring(startIndexHDC + 9, endIndexHDC);
            receivedData.trim();
            rs485Buffer = rs485Buffer.substring(endIndexHDC + 10);
            if (receivedData.length() > 0)
            {
                String jsonString = "{\"device\":\"ESP32_RS485_HDC1080\"";
                int tempIndex = receivedData.indexOf("TEMP:");
                int humIndex = receivedData.indexOf("HUM:");
                if (tempIndex >= 0 && humIndex >= 0)
                {
                    int tempEnd = receivedData.indexOf(",", tempIndex);
                    if (tempEnd == -1)
                        tempEnd = receivedData.length();
                    String tempStr = receivedData.substring(tempIndex + 5, tempEnd);
                    String humStr = receivedData.substring(humIndex + 4);
                    jsonString += ",\"sensor_type\":\"HDC1080\"";
                    jsonString += ",\"temperature\":" + tempStr;
                    jsonString += ",\"humidity\":" + humStr;
                }
                jsonString += "}";
                char jsonBuffer[256];
                jsonString.toCharArray(jsonBuffer, sizeof(jsonBuffer));
                if (!client.connected())
                    client.connect("ESP32Client");
                client.loop();
                client.publish(mqtt_topic, jsonBuffer);
            }
        }
        // SPS30
        int startIndexSPS = rs485Buffer.indexOf("<SPS30>");
        int endIndexSPS = rs485Buffer.indexOf("</SPS30>");
        if (startIndexSPS >= 0 && endIndexSPS >= 0 && endIndexSPS > startIndexSPS)
        {
            String receivedData = rs485Buffer.substring(startIndexSPS + 8, endIndexSPS);
            receivedData.trim();
            rs485Buffer = rs485Buffer.substring(endIndexSPS + 9);
            if (receivedData.length() > 0)
            {
                String jsonString = "{\"device\":\"ESP32_RS485_SPS30\"";
                int pm1Index = receivedData.indexOf("PM1.0:");
                int pm25Index = receivedData.indexOf("PM2.5:");
                int pm4Index = receivedData.indexOf("PM4.0:");
                int pm10Index = receivedData.indexOf("PM10:");
                if (pm1Index >= 0 && pm25Index >= 0 && pm4Index >= 0 && pm10Index >= 0)
                {
                    String pm1Str = receivedData.substring(pm1Index + 6, receivedData.indexOf(",", pm1Index));
                    String pm25Str = receivedData.substring(pm25Index + 6, receivedData.indexOf(",", pm25Index));
                    String pm4Str = receivedData.substring(pm4Index + 6, receivedData.indexOf(",", pm4Index));
                    String pm10Str = receivedData.substring(pm10Index + 5);
                    jsonString += ",\"sensor_type\":\"SPS30\"";
                    jsonString += ",\"pm1_0\":" + pm1Str;
                    jsonString += ",\"pm2_5\":" + pm25Str;
                    jsonString += ",\"pm4_0\":" + pm4Str;
                    jsonString += ",\"pm10\":" + pm10Str;
                }
                jsonString += "}";
                char jsonBuffer[256];
                jsonString.toCharArray(jsonBuffer, sizeof(jsonBuffer));
                if (!client.connected())
                    client.connect("ESP32Client");
                client.loop();
                client.publish(mqtt_topic, jsonBuffer);
            }
        }
        // Giới hạn kích thước buffer để tránh tràn bộ nhớ
        if (rs485Buffer.length() > 500)
        {
            rs485Buffer = rs485Buffer.substring(rs485Buffer.length() - 200);
        }
    }
}

void setup()
{
    Serial.begin(9600);
    delay(1000);
    Serial.println("=== ESP32 STARTING ===");
    pinMode(RE_DE, OUTPUT);
    digitalWrite(RE_DE, LOW);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    // Khởi tạo chân điều khiển MOSFET IRF4435
    pinMode(MOSFET_PIN, OUTPUT);
    RS485Serial.begin(115200, SERIAL_8N1, RXD2, -1);
    Serial.println("RS485 ready");
    WiFi.begin(mqtt_ssid, mqtt_password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected");
    client.setServer(mqtt_server, mqtt_port);
    Serial.println("MQTT ready");
}

void loop()
{
    processRS485AndSendMQTT();
    delay(100);
}
