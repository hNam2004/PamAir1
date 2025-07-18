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
#define LED_BUILTIN 2 // Built-in LED trên ESP32
#define RXD2 34       // MAX485 RO → ESP32 RX (GPIO34)
#define TXD2 35       // MAX485 DI ← ESP32 TX (GPIO35)
#define RE_DE 13      // MAX485 RE/DE ← ESP32 (GPIO4)
#define MOSFET_PIN 23 // GPIO5 - điều khiển MOSFET khi nhận tín hiệu RS485
// TX_PIN đã xóa vì chỉ nhận dữ liệu

// Khai báo UART2 cho RS485
HardwareSerial RS485Serial(2);
HardwareSerial gpsSerial(1);
TinyGPSPlus gps;
// SPS30 struct đã xóa

// Cấu trúc dữ liệu cảm biến HDC1080 (giữ lại để tương thích)
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
QueueHandle_t sensorQueue; // Queue cho dữ liệu HDC1080
QueueHandle_t gpsQueue;

WiFiClient espClient;
PubSubClient client(espClient);
volatile uint8_t Interupt_Flag = 0;

// RS485 Protocol variables
byte start_byte_1 = 0x3C;
byte start_byte_2 = 0xC3;
unsigned int rs485_value = 0;

// SPS30 parse function đã xóa

// Hàm parse dữ liệu HDC1080 từ chuỗi với định dạng chuẩn
SensorData parseSensorData(String data)
{
    SensorData sensorData;
    sensorData.dataValid = false;

    // Format chuẩn: "TYPE:HDC1080,TEMP:25.50,HUM:60.30"

    // Kiểm tra TYPE trước
    if (data.indexOf("TYPE:HDC1080") < 0)
    {
        // Thử các format cũ để tương thích ngược
        int tempIndex = data.indexOf("TEMP:");
        int humIndex = data.indexOf("HUM:");

        if (tempIndex >= 0 && humIndex >= 0)
        {
            // Format cũ: "TEMP:25.50,HUM:60.30"
            String tempStr = data.substring(tempIndex + 5, data.indexOf(",", tempIndex));
            String humStr = data.substring(humIndex + 4);

            sensorData.temperature = tempStr.toFloat();
            sensorData.humidity = humStr.toFloat();
            sensorData.dataValid = true;
        }
        else if (data.indexOf(",") > 0)
        {
            // Format cũ: "25.50,60.30"
            int commaIndex = data.indexOf(",");
            String tempStr = data.substring(0, commaIndex);
            String humStr = data.substring(commaIndex + 1);

            sensorData.temperature = tempStr.toFloat();
            sensorData.humidity = humStr.toFloat();
            sensorData.dataValid = true;
        }

        return sensorData;
    }

    // Parse với định dạng mới
    int tempIndex = data.indexOf("TEMP:");
    int humIndex = data.indexOf("HUM:");

    if (tempIndex >= 0 && humIndex >= 0)
    {
        // Trích xuất nhiệt độ
        int tempEnd = data.indexOf(",", tempIndex);
        if (tempEnd == -1)
            tempEnd = data.length();
        String tempStr = data.substring(tempIndex + 5, tempEnd);
        sensorData.temperature = tempStr.toFloat();

        // Trích xuất độ ẩm
        String humStr = data.substring(humIndex + 4);
        sensorData.humidity = humStr.toFloat();

        sensorData.dataValid = true;
    }

    return sensorData;
}

// Buffer để lưu dữ liệu RS485 chưa hoàn chỉnh
String rs485Buffer = "";

// Hàm gửi dữ liệu qua RS485 với định dạng chuẩn (nếu ESP32 cần gửi dữ liệu)
void sendRS485Data(String sensorType, String data)
{
    digitalWrite(RE_DE, HIGH);
    delay(2);

    // Thêm header và footer để đảm bảo tính toàn vẹn dữ liệu
    String formattedData = "<" + sensorType + ">" + data + "</" + sensorType + ">\n";
    RS485Serial.print(formattedData);
    RS485Serial.flush();

    delay(2);
    digitalWrite(RE_DE, LOW); // quay lại chế độ nhận
}

// Hàm nhận dữ liệu từ RS485 với định dạng chuẩn
void processRS485Command()
{
    if (RS485Serial.available())
    {
        // Bật MOSFET khi nhận tín hiệu RS485
        digitalWrite(MOSFET_PIN, HIGH);
        digitalWrite(LED_BUILTIN, HIGH);
        Serial.println("MOSFET ON - RS485 signal received!");

        String newData = RS485Serial.readString();
        rs485Buffer += newData;

        // Tìm kiếm gói dữ liệu HDC1080 hoàn chỉnh với header và footer
        int startIndex = rs485Buffer.indexOf("<HDC1080>");
        int endIndex = rs485Buffer.indexOf("</HDC1080>");

        if (startIndex >= 0 && endIndex >= 0 && endIndex > startIndex)
        {
            // Trích xuất dữ liệu giữa header và footer
            String receivedData = rs485Buffer.substring(startIndex + 9, endIndex);
            receivedData.trim(); // Loại bỏ ký tự xuống dòng và khoảng trắng

            Serial.print("Nhan du lieu RS485: ");
            Serial.println(receivedData);

            // Xóa dữ liệu đã xử lý khỏi buffer
            rs485Buffer = rs485Buffer.substring(endIndex + 10);

            // Parse dữ liệu cảm biến HDC1080
            if (receivedData.length() > 0)
            {
                SensorData sensorData = parseSensorData(receivedData);

                if (sensorData.dataValid)
                {
                    // Gửi dữ liệu HDC1080 vào queue
                    if (xQueueSend(sensorQueue, &sensorData, pdMS_TO_TICKS(100)) == pdTRUE)
                    {
                        Serial.print("HDC1080 - Nhiet do: ");
                        Serial.print(sensorData.temperature);
                        Serial.print(" °C  |  Do am: ");
                        Serial.print(sensorData.humidity);
                        Serial.println(" %");
                        Serial.println("Du lieu HDC1080 da gui vao queue thanh cong");

                        // Giữ MOSFET bật trong 3 giây sau khi xử lý thành công
                        delay(3000);
                        digitalWrite(MOSFET_PIN, LOW);
                        digitalWrite(LED_BUILTIN, LOW);
                        Serial.println("MOSFET OFF - Processing completed");
                    }
                    else
                    {
                        Serial.println("Khong the gui du lieu HDC1080 vao queue!");
                        // Tắt MOSFET nếu lỗi
                        digitalWrite(MOSFET_PIN, LOW);
                        digitalWrite(LED_BUILTIN, LOW);
                    }
                }
                else
                {
                    Serial.println("Du lieu HDC1080 khong hop le!");
                    // Tắt MOSFET nếu dữ liệu không hợp lệ
                    digitalWrite(MOSFET_PIN, LOW);
                    digitalWrite(LED_BUILTIN, LOW);
                }
            }
        }
        else
        {
            // Nếu chưa nhận đủ dữ liệu, tắt MOSFET sau 1 giây
            delay(1000);
            digitalWrite(MOSFET_PIN, LOW);
            digitalWrite(LED_BUILTIN, LOW);
            Serial.println("MOSFET OFF - Incomplete data");
        }

        // Giới hạn kích thước buffer để tránh tràn bộ nhớ
        if (rs485Buffer.length() > 500)
        {
            rs485Buffer = rs485Buffer.substring(rs485Buffer.length() - 200);
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

        // Đọc dữ liệu HDC1080 từ queue
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
            char jsonBuffer[512]; // Tăng kích thước buffer cho SPS30 data
            for (int i = 0; i < 512; i++)
            {
                jsonBuffer[i] = 0;
            }

            // Tạo JSON với dữ liệu HDC1080 và GPS
            String jsonString = "{\"device\":\"ESP32_RS485_HDC1080\"";

            if (hasSensorData)
            {
                jsonString += ",\"sensor_type\":\"HDC1080\"";
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
    delay(1000);
    Serial.println("=== ESP32 STARTING ===");

    // GPS tạm thời bị vô hiệu hóa
    Serial.println("GPS disabled - no pins defined");

    // Khởi tạo MOSFET control
    pinMode(MOSFET_PIN, OUTPUT);
    digitalWrite(MOSFET_PIN, LOW); // Mặc định tắt MOSFET
    Serial.println("MOSFET control initialized");

    // Khởi tạo RS485 - chỉ nhận dữ liệu
    Serial.println("Khoi tao RS485...");
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    pinMode(RE_DE, OUTPUT);
    digitalWrite(RE_DE, LOW);                        // Mặc định là chế độ nhận
    RS485Serial.begin(115200, SERIAL_8N1, RXD2, -1); // Baudrate 9600
    Serial.println("RS485 da khoi tao - co the gui/nhan du lieu");

    Serial.println("ESP32 nhan du lieu HDC1080 tu ATmega qua RS485");
    Serial.println("RS485: RX=34, TX=35, RE_DE=13");

    Serial.println("Khoi tao WiFi...");
    sys_wifi_init();
    Serial.println("WiFi da khoi tao");

    Serial.println("Khoi tao CapServer...");
    sys_capserver_init();
    Serial.println("CapServer da khoi tao");

    Serial.println("Cau hinh GPIO...");
    pinMode(BOOT_PIN, INPUT_PULLUP);                                                // Configure BOOT pin as input with internal pull-up resistor
    attachInterrupt(digitalPinToInterrupt(BOOT_PIN), bootInterruptHandler, RISING); // Attach interrupt handler to rising edge of BOOT pin
    Serial.println("GPIO da cau hinh");

    Serial.println("Khoi tao MQTT...");
    client.setServer(mqtt_server, mqtt_port);
    Serial.println("MQTT da khoi tao");

    // Create queues
    Serial.println("Tao queues...");
    sensorQueue = xQueueCreate(5, sizeof(SensorData)); // Queue có thể chứa 5 phần tử SensorData (HDC1080)
    if (sensorQueue == NULL)
    {
        Serial.println("Loi tao sensor queue!");
        while (1)
            ;
    }
    Serial.println("Sensor queue da tao");

    // SPS30 queue đã xóa

    gpsQueue = xQueueCreate(3, sizeof(GPSData)); // Queue có thể chứa 3 phần tử GPSData
    if (gpsQueue == NULL)
    {
        Serial.println("Loi tao GPS queue!");
        while (1)
            ;
    }

    // Tạo các tasks
    Serial.println("Tao tasks...");
    xTaskCreate(
        task1Function, // Task function (RS485 handler)
        "Task 1",      // Task name
        10000,         // Stack size (bytes)
        NULL,          // Task parameters
        2,             // Task priority
        NULL           // Task handle
    );
    Serial.println("Task 1 da tao");

    // GPS task disabled - no GPS pins defined
    // xTaskCreate(
    //     gpsReadTask, // Task function (GPS reader)
    //     "GPS Task",  // Task name
    //     10000,       // Stack size (bytes)
    //     NULL,        // Task parameters
    //     2,           // Task priority
    //     NULL         // Task handle
    // );

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
