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
#define RXD2 3       // MAX485 RO → ESP32 RX (GPIO34)
#define TXD2 1       // MAX485 DI ← ESP32 TX (GPIO35)
#define RE_DE 13     // MAX485 RE/DE ← ESP32 (GPIO4)
#define MOSFET_PIN 23 // GPIO23 dùng để điều khiển IRF4435

HardwareSerial RS485Serial(2);

WiFiClient espClient;
PubSubClient client(espClient);

String rs485Buffer = "";

// Hàm gửi dữ liệu qua RS485
void sendRS485Data(String data)
{
    digitalWrite(RE_DE, HIGH); // Chế độ gửi
    delay(1);                  // Delay nhỏ để ổn định
    RS485Serial.print(data);
    RS485Serial.flush();      // Đợi gửi xong
    delay(1);                 // Delay nhỏ
    digitalWrite(RE_DE, LOW); // Chế độ nhận
}

// Hàm xử lý dữ liệu HDC1080
void processHDC1080Data(String receivedData)
{
    String jsonString = "{\"device\":\"ESP32_RS485_HDC1080\"";
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

        jsonString += ",\"temperature\":" + tempStr;
        jsonString += ",\"humidity\":" + humStr;
        jsonString += ",\"timestamp\":" + String(millis());
        jsonString += "}";

        char jsonBuffer[256];
        jsonString.toCharArray(jsonBuffer, sizeof(jsonBuffer));

        if (!client.connected())
        {
            if (client.connect("ESP32Client"))
            {
                Serial.println("Reconnected to MQTT");
            }
            else
            {
                Serial.println("MQTT connection failed");
            }
        }
        if (client.connected())
        {
            client.loop();
            client.publish(mqtt_topic, jsonBuffer);
            Serial.print("MQTT Published HDC1080: ");
            Serial.println(jsonString);
        }
    }
}

// Hàm xử lý dữ liệu SPS30
void processSPS30Data(String receivedData)
{
    String jsonString = "{\"device\":\"ESP32_RS485_SPS30\"";

    // Parse các giá trị PM
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

        jsonString += ",\"pm1_0\":" + pm1Str;
        jsonString += ",\"pm2_5\":" + pm25Str;
        jsonString += ",\"pm4_0\":" + pm4Str;
        jsonString += ",\"pm10_0\":" + pm10Str;
        jsonString += ",\"timestamp\":" + String(millis());
        jsonString += "}";

        char jsonBuffer[256];
        jsonString.toCharArray(jsonBuffer, sizeof(jsonBuffer));

        if (!client.connected())
        {
            if (client.connect("ESP32Client"))
            {
                Serial.println("Reconnected to MQTT");
            }
            else
            {
                Serial.println("MQTT connection failed");
            }
        }
        if (client.connected())
        {
            client.loop();
            client.publish(mqtt_topic, jsonBuffer);
            Serial.print("MQTT Published SPS30: ");
            Serial.println(jsonString);
        }
    }
}

void processRS485AndSendMQTT()
{
    if (RS485Serial.available())
    {
        String newData = RS485Serial.readStringUntil('\n');
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
                Serial.print("HDC1080 Received: ");
                Serial.println(receivedData);
                processHDC1080Data(receivedData);
            }
        }
        // SPS30
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
                processSPS30Data(receivedData);
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
    digitalWrite(MOSFET_PIN, LOW); // Luôn bật MOSFET IRF4435 (P-Channel)
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

// Thêm logic kiểm soát nguồn IRF4435
unsigned long lastActivityTime = 0;
const unsigned long inactivityTimeout = 10000; // 10 giây không hoạt động thì tắt nguồn

void checkIRF4435Power()
{
    if (RS485Serial.available())
    {
        lastActivityTime = millis();   // Cập nhật thời gian hoạt động khi có dữ liệu
        digitalWrite(MOSFET_PIN, LOW); // Bật MOSFET nếu có hoạt động
    }
    // Tắt MOSFET nếu không có hoạt động trong 10 giây
    if (millis() - lastActivityTime > inactivityTimeout && digitalRead(MOSFET_PIN) == LOW)
    {
        digitalWrite(MOSFET_PIN, HIGH); // Tắt MOSFET
    }
}

void loop()
{
    processRS485AndSendMQTT();
    checkIRF4435Power();

    // Kiểm tra lệnh từ Serial Monitor
    if (Serial.available())
    {
        String command = Serial.readStringUntil('\n');
        command.trim();

        if (command == "send_test")
        {
            // Gửi lệnh test qua RS485
            String testMessage = "<TEST>ESP32_TEST_MESSAGE</TEST>\n";
            sendRS485Data(testMessage);
            Serial.print("RS485 Test Sent: ");
            Serial.println(testMessage);
        }
        else if (command.startsWith("send:"))
        {
            // Gửi dữ liệu tùy chỉnh qua RS485
            String customData = command.substring(5);
            sendRS485Data(customData + "\n");
            Serial.print("RS485 Custom Sent: ");
            Serial.println(customData);
        }
        else if (command == "status")
        {
            // Hiển thị trạng thái hệ thống
            Serial.println("=== ESP32 RS485 STATUS ===");
            Serial.print("WiFi: ");
            Serial.println(WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected");
            Serial.print("MQTT: ");
            Serial.println(client.connected() ? "Connected" : "Disconnected");
            Serial.print("RS485 Buffer Length: ");
            Serial.println(rs485Buffer.length());
            Serial.print("Free Heap: ");
            Serial.println(ESP.getFreeHeap());
        }
    }

    delay(100);
}