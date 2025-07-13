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
#define RXD2 34  // RO từ MAX485 (GPIO16)
#define TXD2 35  // DI đến MAX485 (GPIO17)
#define RE_DE 13 // Điều khiển gửi / nhận (GPIO4)
#define GPS_RX 16  // Chân RX cho GPS
#define GPS_TX 17  // Chân TX cho GPS

// Khai báo UART2 cho RS485
HardwareSerial RS485Serial(2);
HardwareSerial gpsSerial(1);
TinyGPSPlus gps;
// Cấu trúc dữ liệu cảm biến SPS30 nhận từ RS485
struct SPS30Data
{
    float pm1_0;        // PM1.0 [μg/m³]
    float pm2_5;        // PM2.5 [μg/m³]
    float pm4_0;        // PM4.0 [μg/m³]
    float pm10_0;       // PM10.0 [μg/m³]
    float nc0_5;        // Number concentration PM0.5 [#/cm³]
    float nc1_0;        // Number concentration PM1.0 [#/cm³]
    float nc2_5;        // Number concentration PM2.5 [#/cm³]
    float nc4_0;        // Number concentration PM4.0 [#/cm³]
    float nc10_0;       // Number concentration PM10.0 [#/cm³]
    float typical_size; // Typical particle size [μm]
    bool dataValid;
};

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
QueueHandle_t sps30Queue;  // Queue cho dữ liệu SPS30
QueueHandle_t gpsQueue;

WiFiClient espClient;
PubSubClient client(espClient);
volatile uint8_t Interupt_Flag = 0;

// Hàm parse dữ liệu SPS30 từ chuỗi theo định dạng chuẩn
SPS30Data parseSPS30Data(String data)
{
    SPS30Data sps30Data;
    sps30Data.dataValid = false;

    // Khởi tạo giá trị mặc định
    sps30Data.pm1_0 = 0;
    sps30Data.pm2_5 = 0;
    sps30Data.pm4_0 = 0;
    sps30Data.pm10_0 = 0;
    sps30Data.nc0_5 = 0;
    sps30Data.nc1_0 = 0;
    sps30Data.nc2_5 = 0;
    sps30Data.nc4_0 = 0;
    sps30Data.nc10_0 = 0;
    sps30Data.typical_size = 0;

    // Format chuẩn từ ATmega: "TYPE:SPS30,PM1:1.23,PM2.5:2.34,PM4:3.45,PM10:4.56,NC0.5:123.4,NC1:234.5,NC2.5:345.6,NC4:456.7,NC10:567.8,SIZE:1.23"

    // Kiểm tra TYPE trước
    if (data.indexOf("TYPE:SPS30") < 0)
    {
        return sps30Data; // Không phải dữ liệu SPS30
    }

    // Parse PM values với định dạng mới
    int pm1Index = data.indexOf("PM1:");
    int pm25Index = data.indexOf("PM2.5:");
    int pm4Index = data.indexOf("PM4:");
    int pm10Index = data.indexOf("PM10:");

    if (pm1Index >= 0 && pm25Index >= 0 && pm4Index >= 0 && pm10Index >= 0)
    {
        // Parse PM1
        String pm1Str = data.substring(pm1Index + 4, data.indexOf(",", pm1Index));
        sps30Data.pm1_0 = pm1Str.toFloat();

        // Parse PM2.5
        String pm25Str = data.substring(pm25Index + 6, data.indexOf(",", pm25Index));
        sps30Data.pm2_5 = pm25Str.toFloat();

        // Parse PM4
        String pm4Str = data.substring(pm4Index + 4, data.indexOf(",", pm4Index));
        sps30Data.pm4_0 = pm4Str.toFloat();

        // Parse PM10
        int pm10End = data.indexOf(",", pm10Index);
        if (pm10End == -1)
            pm10End = data.length(); // Nếu là phần tử cuối
        String pm10Str = data.substring(pm10Index + 5, pm10End);
        sps30Data.pm10_0 = pm10Str.toFloat();

        sps30Data.dataValid = true;

        // Parse NC values (nếu có)
        int nc05Index = data.indexOf("NC0.5:");
        int nc1Index = data.indexOf("NC1:");
        int nc25Index = data.indexOf("NC2.5:");
        int nc4Index = data.indexOf("NC4:");
        int nc10Index = data.indexOf("NC10:");
        int sizeIndex = data.indexOf("SIZE:");

        if (nc05Index >= 0)
        {
            int nc05End = data.indexOf(",", nc05Index);
            if (nc05End == -1)
                nc05End = data.length();
            String nc05Str = data.substring(nc05Index + 6, nc05End);
            sps30Data.nc0_5 = nc05Str.toFloat();
        }
        if (nc1Index >= 0)
        {
            int nc1End = data.indexOf(",", nc1Index);
            if (nc1End == -1)
                nc1End = data.length();
            String nc1Str = data.substring(nc1Index + 4, nc1End);
            sps30Data.nc1_0 = nc1Str.toFloat();
        }
        if (nc25Index >= 0)
        {
            int nc25End = data.indexOf(",", nc25Index);
            if (nc25End == -1)
                nc25End = data.length();
            String nc25Str = data.substring(nc25Index + 6, nc25End);
            sps30Data.nc2_5 = nc25Str.toFloat();
        }
        if (nc4Index >= 0)
        {
            int nc4End = data.indexOf(",", nc4Index);
            if (nc4End == -1)
                nc4End = data.length();
            String nc4Str = data.substring(nc4Index + 4, nc4End);
            sps30Data.nc4_0 = nc4Str.toFloat();
        }
        if (nc10Index >= 0)
        {
            int nc10End = data.indexOf(",", nc10Index);
            if (nc10End == -1)
                nc10End = data.length();
            String nc10Str = data.substring(nc10Index + 5, nc10End);
            sps30Data.nc10_0 = nc10Str.toFloat();
        }
        if (sizeIndex >= 0)
        {
            String sizeStr = data.substring(sizeIndex + 5);
            sps30Data.typical_size = sizeStr.toFloat();
        }
    }

    return sps30Data;
}

// Hàm parse dữ liệu HDC1080 từ chuỗi (giữ lại để tương thích)
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
        String newData = RS485Serial.readString();
        rs485Buffer += newData;

        // Tìm kiếm gói dữ liệu hoàn chỉnh với header và footer
        int startIndex = rs485Buffer.indexOf("<SPS30>");
        int endIndex = rs485Buffer.indexOf("</SPS30>");

        if (startIndex >= 0 && endIndex >= 0 && endIndex > startIndex)
        {
            // Trích xuất dữ liệu giữa header và footer
            String receivedData = rs485Buffer.substring(startIndex + 7, endIndex);
            receivedData.trim(); // Loại bỏ ký tự xuống dòng và khoảng trắng

            Serial.print("Nhan du lieu RS485: ");
            Serial.println(receivedData);

            // Xóa dữ liệu đã xử lý khỏi buffer
            rs485Buffer = rs485Buffer.substring(endIndex + 8);

            // Parse dữ liệu cảm biến
            if (receivedData.length() > 0)
            {
                // Kiểm tra TYPE để xác định loại dữ liệu
                if (receivedData.indexOf("TYPE:SPS30") >= 0)
                {
                    // Dữ liệu từ SPS30
                    SPS30Data sps30Data = parseSPS30Data(receivedData);

                    if (sps30Data.dataValid)
                    {
                        // Gửi dữ liệu SPS30 vào queue
                        if (xQueueSend(sps30Queue, &sps30Data, pdMS_TO_TICKS(100)) == pdTRUE)
                        {
                            Serial.print("SPS30 - PM1: ");
                            Serial.print(sps30Data.pm1_0);
                            Serial.print(" μg/m³  |  PM2.5: ");
                            Serial.print(sps30Data.pm2_5);
                            Serial.print(" μg/m³  |  PM4: ");
                            Serial.print(sps30Data.pm4_0);
                            Serial.print(" μg/m³  |  PM10: ");
                            Serial.print(sps30Data.pm10_0);
                            Serial.println(" μg/m³");
                            Serial.println("Du lieu SPS30 da gui vao queue thanh cong");
                        }
                        else
                        {
                            Serial.println("Khong the gui du lieu SPS30 vao queue!");
                        }
                    }
                    else
                    {
                        Serial.println("Du lieu SPS30 khong hop le!");
                    }
                }
                else if (receivedData.indexOf("TYPE:HDC1080") >= 0 || receivedData.indexOf("TEMP:") >= 0)
                {
                    // Dữ liệu từ HDC1080 (nhiệt độ/độ ẩm)
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
                        }
                        else
                        {
                            Serial.println("Khong the gui du lieu HDC1080 vao queue!");
                        }
                    }
                    else
                    {
                        Serial.println("Du lieu HDC1080 khong hop le!");
                    }
                }
                else
                {
                    Serial.println("Khong nhan dien duoc loai du lieu!");
                }
            }
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
        SPS30Data sps30Data;
        GPSData gpsData;
        bool hasSensorData = false;
        bool hasSPS30Data = false;
        bool hasGPSData = false;

        // Đọc dữ liệu HDC1080 từ queue
        if (xQueueReceive(sensorQueue, &sensorData, pdMS_TO_TICKS(50)) == pdTRUE)
        {
            hasSensorData = true;
        }

        // Đọc dữ liệu SPS30 từ queue
        if (xQueueReceive(sps30Queue, &sps30Data, pdMS_TO_TICKS(50)) == pdTRUE)
        {
            hasSPS30Data = true;
        }

        // Đọc dữ liệu GPS từ queue
        if (xQueueReceive(gpsQueue, &gpsData, pdMS_TO_TICKS(50)) == pdTRUE)
        {
            hasGPSData = true;
        }

        // Gửi dữ liệu lên MQTT nếu có WiFi
        if ((hasSensorData || hasSPS30Data || hasGPSData) && wifiState == WIFI_CONNECTED)
        {
            // Create a character array to hold the JSON string
            char jsonBuffer[512]; // Tăng kích thước buffer cho SPS30 data
            for (int i = 0; i < 512; i++)
            {
                jsonBuffer[i] = 0;
            }

            // Tạo JSON với cả sensor, SPS30 và GPS data
            String jsonString = "{\"device\":\"ESP32_RS485_Multi_Sensor\"";

            if (hasSensorData)
            {
                jsonString += ",\"temperature\":" + String(sensorData.temperature, 2);
                jsonString += ",\"humidity\":" + String(sensorData.humidity, 2);
            }

            if (hasSPS30Data)
            {
                jsonString += ",\"pm1_0\":" + String(sps30Data.pm1_0, 2);
                jsonString += ",\"pm2_5\":" + String(sps30Data.pm2_5, 2);
                jsonString += ",\"pm4_0\":" + String(sps30Data.pm4_0, 2);
                jsonString += ",\"pm10_0\":" + String(sps30Data.pm10_0, 2);

                // Thêm NC data nếu có
                if (sps30Data.nc0_5 > 0 || sps30Data.nc1_0 > 0)
                {
                    jsonString += ",\"nc0_5\":" + String(sps30Data.nc0_5, 1);
                    jsonString += ",\"nc1_0\":" + String(sps30Data.nc1_0, 1);
                    jsonString += ",\"nc2_5\":" + String(sps30Data.nc2_5, 1);
                    jsonString += ",\"nc4_0\":" + String(sps30Data.nc4_0, 1);
                    jsonString += ",\"nc10_0\":" + String(sps30Data.nc10_0, 1);
                    jsonString += ",\"typical_size\":" + String(sps30Data.typical_size, 2);
                }
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
    gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
    // Khởi tạo RS485
    pinMode(RE_DE, OUTPUT);
    digitalWrite(RE_DE, LOW);                    // Mặc định là chế độ nhận
    RS485Serial.begin(9600, SERIAL_8N1, 24, 26); // UART2
    Serial.println("RS485 da khoi tao");

    Serial.println("ESP32 nhan du lieu tu cam bien SPS30 va HDC1080 qua RS485");

    sys_wifi_init();
    sys_capserver_init();
    pinMode(BOOT_PIN, INPUT_PULLUP);                                                // Configure BOOT pin as input with internal pull-up resistor
    attachInterrupt(digitalPinToInterrupt(BOOT_PIN), bootInterruptHandler, RISING); // Attach interrupt handler to rising edge of BOOT pin
    Serial.println("All Done!");
    client.setServer(mqtt_server, mqtt_port);

    // Create queues
    sensorQueue = xQueueCreate(5, sizeof(SensorData)); // Queue có thể chứa 5 phần tử SensorData (HDC1080)
    if (sensorQueue == NULL)
    {
        Serial.println("Loi tao sensor queue!");
        while (1)
            ;
    }

    sps30Queue = xQueueCreate(5, sizeof(SPS30Data)); // Queue có thể chứa 5 phần tử SPS30Data
    if (sps30Queue == NULL)
    {
        Serial.println("Loi tao SPS30 queue!");
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
