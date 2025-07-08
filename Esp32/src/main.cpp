#include <WiFi.h>
#include <HardwareSerial.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <PubSubClient.h>
#include <Wire.h>              // giao tiếp I2C với arduino
#include <LiquidCrystal_I2C.h> // Điều khiển màn hình I2C thông qua giao tiếp arduino
#include <ArduinoJson.h>       // Thư viện xử lý JSON, cài từ Library Manager
#include "sys_capserver.hpp"
#include "sys_wifi.hpp"
#include "sys_eeprom.hpp"

// WiFi & MQTT Broker
const char *mqtt_ssid = "emqx";
const char *mqtt_password = "public";
const char *mqtt_server = "broker.emqx.io";
const int mqtt_port = 1883; // MQTT default port


// Cấu hình cảm biến & bơm
#define SOIL_MOISTURE_PIN 1 // ESP32-S3 dùng chân ADC1 (GPIO1-GPIO10 là ADC1)
#define RELAY_PIN 5
#define BOOT_PIN 0 // Thay đổi từ GPIO 0 sang GPIO 4 để tránh xung đột với chân BOOT của ESP32-S3

// Cấu hình LCD 16x2 (địa chỉ mặc định 0x27 hoặc 0x3F)
LiquidCrystal_I2C lcd(0x27, 16, 2); // lcd (16x2)

WiFiClient espClient;
PubSubClient client(espClient);
volatile uint8_t Interupt_Flag = 0;

int moistureThreshold = 40;     // Ngưỡng độ ẩm (%), nếu thấp hơn sẽ bật bơm
bool manualPumpControl = false; // để ở chế độ tự động nếu true thì là chế độ thủ công
bool pumpState = false;         // máy bơm mặc định là tắt

// Hiệu chỉnh cảm biến độ ẩm đất
int value_dry = 1023; // Giá trị ADC khi đất khô
int value_wet = 0;    // Giá trị ADC khi đất ướt

// This function will be called when the BOOT pin transitions from LOW to HIGH (rising edge)
void bootInterruptHandler()
{
    Serial.println("Interupt ocur");
    Interupt_Flag = 1;
}

// Hàm cập nhật relay ngay lập tức
void updateRelay()
{
    digitalWrite(RELAY_PIN, pumpState ? LOW : HIGH);
    Serial.println(pumpState ? "Bơm ĐANG BẬT" : "Bơm ĐANG TẮT");
}

// Biến để lưu trữ thông điệp MQTT nhận được
volatile bool newMqttMessageReceived = false;
char receivedTopic[50];
char receivedPayload[200];
unsigned int receivedLength = 0;

// Callback khi nhận lệnh từ MQTT - giữ callback đơn giản và nhanh
void callback(char *topic, byte *payload, unsigned int length)
{
    // Chỉ sao chép dữ liệu và đặt cờ, không xử lý trong callback
    if (length < sizeof(receivedPayload) && strlen(topic) < sizeof(receivedTopic))
    {
        strncpy(receivedTopic, topic, sizeof(receivedTopic) - 1);
        receivedTopic[sizeof(receivedTopic) - 1] = '\0'; // Đảm bảo null-terminated

        memcpy(receivedPayload, payload, length);
        receivedPayload[length] = '\0'; // Đảm bảo null-terminated

        receivedLength = length;
        newMqttMessageReceived = true;
    }
}

// Hàm xử lý thông điệp MQTT (được gọi từ task, không phải từ callback)
void processMqttMessage()
{
    if (!newMqttMessageReceived)
    {
        return;
    }

    // Đánh dấu đã xử lý
    newMqttMessageReceived = false;

    // Xử lý thông điệp
    String message = String(receivedPayload);
    Serial.print("Nhận được thông điệp MQTT: ");
    Serial.println(message);

    // Kiểm tra xem có phải là lệnh trực tiếp không (ON, OFF, AUTO)
    if (message == "ON" || message == "on" || message == "On")
    {
        Serial.println("Nhận được lệnh trực tiếp: ON");
        if (String(receivedTopic) == String(mqtt_pump_control_topic))
        {
            manualPumpControl = true;
            pumpState = true;
            updateRelay(); // Cập nhật relay ngay lập tức
            Serial.println("Bật bơm!");
        }
        return;
    }
    else if (message == "OFF" || message == "off" || message == "Off")
    {
        Serial.println("Nhận được lệnh trực tiếp: OFF");
        if (String(receivedTopic) == String(mqtt_pump_control_topic))
        {
            manualPumpControl = true;
            pumpState = false;
            updateRelay(); // Cập nhật relay ngay lập tức
            Serial.println("Tắt bơm!");
        }
        return;
    }
    else if (message == "AUTO" || message == "auto" || message == "Auto")
    {
        Serial.println("Nhận được lệnh trực tiếp: AUTO");
        if (String(receivedTopic) == String(mqtt_pump_control_topic))
        {
            manualPumpControl = false;
            Serial.println("Chế độ tự động!");
        }
        return;
    }

    // Nếu không phải lệnh trực tiếp, thử phân tích JSON
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, message);

    if (error)
    {
        Serial.print("Lỗi JSON: ");
        Serial.println(error.f_str());
        Serial.println("Thử xử lý như lệnh thông thường...");

        // Nếu không phải JSON, xử lý như lệnh thông thường
        if (String(receivedTopic) == String(mqtt_pump_control_topic))
        {
            if (message.indexOf("ON") >= 0)
            {
                manualPumpControl = true;
                pumpState = true;
                updateRelay(); // Cập nhật relay ngay lập tức
                Serial.println("Bật bơm!");
            }
            else if (message.indexOf("OFF") >= 0)
            {
                manualPumpControl = true;
                pumpState = false;
                updateRelay(); // Cập nhật relay ngay lập tức
                Serial.println("Tắt bơm!");
            }
            else if (message.indexOf("AUTO") >= 0)
            {
                manualPumpControl = false;
                Serial.println("Chế độ tự động!");
            }
        }
        return;
    }

    // Xử lý JSON
    const char *command = doc["message"]; // Lấy giá trị "message"
    if (!command)
    {
        // Thử các trường khác
        command = doc["command"];
        if (!command)
        {
            command = doc["value"];
            if (!command)
            {
                Serial.println("Không tìm thấy trường lệnh trong JSON");
                return;
            }
        }
    }

    Serial.print("Lệnh từ JSON: ");
    Serial.println(command);

    if (String(receivedTopic) == String(mqtt_pump_control_topic))
    {
        if (strcmp(command, "ON") == 0)
        {
            manualPumpControl = true;
            pumpState = true;
            updateRelay(); // Cập nhật relay ngay lập tức
            Serial.println("Bật bơm!");
        }
        else if (strcmp(command, "OFF") == 0)
        {
            manualPumpControl = true;
            pumpState = false;
            updateRelay(); // Cập nhật relay ngay lập tức
            Serial.println("Tắt bơm!");
        }
        else if (strcmp(command, "AUTO") == 0)
        {
            manualPumpControl = false;
            Serial.println("Chế độ tự động!");
        }
    }
}

// Kết nối lại MQTT nếu bị mất kết nối
bool reconnect()
{
    // Giới hạn số lần thử kết nối để tránh treo
    static const int MAX_RECONNECT_ATTEMPTS = 3;
    int attempts = 0;

    while (!client.connected() && attempts < MAX_RECONNECT_ATTEMPTS)
    {
        Serial.print("Kết nối MQTT...");
        // Tạo client ID ngẫu nhiên để tránh xung đột
        String clientId = "esp_ktlt_";
        clientId += String(random(0xffff), HEX);

        if (client.connect(clientId.c_str()))
        {
            Serial.println("Đã kết nối!");

            // Đăng ký nhận thông báo từ topic pump_control
            if (client.subscribe(mqtt_pump_control_topic))
            {
                Serial.print("Đã đăng ký topic: ");
                Serial.println(mqtt_pump_control_topic);
            }
            else
            {
                Serial.println("Lỗi đăng ký topic");
            }
            return true;
        }
        else
        {
            attempts++;
            Serial.print("Thất bại, mã lỗi: ");
            Serial.print(client.state());
            Serial.print(" - Lần thử: ");
            Serial.print(attempts);
            Serial.println("/3");
            // Sử dụng vTaskDelay thay vì delay để không chặn các task khác
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    return false;
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
    // Biến để theo dõi thời gian kết nối MQTT
    unsigned long lastMqttReconnectAttempt = 0;
    const unsigned long mqttReconnectInterval = 10000; // 10 giây

    // Biến để theo dõi thời gian cập nhật cảm biến
    unsigned long lastSensorUpdate = 0;
    const unsigned long sensorUpdateInterval = 5000; // 5 giây

    while (true)
    {
        unsigned long currentMillis = millis();

        // Xử lý thông điệp MQTT nếu có
        if (newMqttMessageReceived)
        {
            processMqttMessage();
        }

        if (wifiState == WIFI_CONNECTED)
        {
            // Kiểm tra và xử lý kết nối MQTT
            if (!client.connected())
            {
                if (currentMillis - lastMqttReconnectAttempt > mqttReconnectInterval)
                {
                    lastMqttReconnectAttempt = currentMillis;
                    // Thử kết nối lại MQTT
                    if (reconnect())
                    {
                        lastMqttReconnectAttempt = 0; // Reset nếu kết nối thành công
                    }
                }
            }
            else
            {
                // Xử lý các gói tin MQTT đến
                client.loop();
            }

            // Cập nhật cảm biến và điều khiển bơm theo chu kỳ
            if (currentMillis - lastSensorUpdate > sensorUpdateInterval)
            {
                lastSensorUpdate = currentMillis;

                // Đọc độ ẩm đất
                int soilMoistureRaw = analogRead(SOIL_MOISTURE_PIN);                          // đọc giá trị analog
                int soilMoisturePercent = map(soilMoistureRaw, value_dry, value_wet, 0, 100); // chuyển về %
                soilMoisturePercent = constrain(soilMoisturePercent, 0, 100);                 // giới hạn trong khoảng 0-100% nếu giá trị vượt

                Serial.print("Độ ẩm đất: ");
                Serial.print(soilMoisturePercent);
                Serial.println("%");

                // Điều khiển bơm dựa trên độ ẩm đất
                bool oldPumpState = pumpState; // Lưu trạng thái cũ
                if (!manualPumpControl)
                {
                    if (soilMoisturePercent < moistureThreshold)
                    {
                        pumpState = true;
                    }
                    else
                    {
                        pumpState = false;
                    }
                }

                // Chỉ cập nhật relay nếu trạng thái thay đổi hoặc lần đầu
                if (oldPumpState != pumpState)
                {
                    updateRelay();
                }
                else
                {
                    // Vẫn cập nhật để đảm bảo relay đúng trạng thái
                    digitalWrite(RELAY_PIN, pumpState ? LOW : HIGH);
                }

                // Cập nhật hiển thị LCD - không xóa màn hình
                lcd.setCursor(0, 0); // đặt đến vị trí 0 0 in giá trị
                lcd.print("Do am: ");
                lcd.print(soilMoisturePercent);
                lcd.print("%    "); // Thêm khoảng trắng để xóa ký tự thừa

                lcd.setCursor(0, 1); // qua vị trí 0 1 in tiếp
                lcd.print("Bom: ");
                lcd.print(pumpState ? "BAT   " : "TAT   "); // Thêm khoảng trắng để xóa ký tự thừa

                // Chỉ gửi dữ liệu lên MQTT nếu đã kết nối
                if (client.connected())
                {
                    // Gửi dữ liệu lên MQTT
                    char jsonBuffer[128];
                    StaticJsonDocument<200> doc;
                    doc["soil_moisture"] = soilMoisturePercent;
                    doc["pump_status"] = pumpState ? "ON" : "OFF";
                    serializeJson(doc, jsonBuffer);

                    // Publish to the base topic with all data
                    if (!client.publish(mqtt_base_topic, jsonBuffer))
                    {
                        Serial.println("Lỗi khi gửi dữ liệu tổng hợp");
                    }

                    // Publish individual values to specific topics
                    if (!client.publish(mqtt_humidity_topic, String(soilMoisturePercent).c_str()))
                    {
                        Serial.println("Lỗi khi gửi dữ liệu độ ẩm");
                    }

                    if (!client.publish(mqtt_pump_status_topic, pumpState ? "ON" : "OFF"))
                    {
                        Serial.println("Lỗi khi gửi trạng thái bơm");
                    }
                }
            }
        }

        // Sử dụng vTaskDelay ngắn để đảm bảo phản hồi nhanh với các sự kiện
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void setup()
{
    Serial.begin(115200);
    // Đợi Serial sẵn sàng cho ESP32-S3 với USB CDC
    while (!Serial && millis() < 5000)
    {
        delay(10);
    }
    delay(1000); // Thêm delay để ổn định

    Serial.println("ESP32-S3 Starting...");

    // Khởi tạo seed cho random - sử dụng chân ADC hợp lệ cho ESP32-S3
    randomSeed(analogRead(2));

    // Khởi động LCD với địa chỉ 0x27
    // ESP32-S3 sử dụng chân I2C khác: SDA=8, SCL=9
    Wire.begin(8, 9);

    // Kiểm tra kết nối với LCD
    Serial.println("Kiểm tra kết nối với LCD tại địa chỉ 0x27...");
    Wire.beginTransmission(0x27);
    byte error = Wire.endTransmission();

    if (error == 0)
    {
        Serial.println("Kết nối thành công với LCD tại địa chỉ 0x27");
    }
    else
    {
        Serial.println("Không thể kết nối với LCD tại địa chỉ 0x27");
        Serial.println("Mã lỗi: " + String(error));
        Serial.println("Kiểm tra lại kết nối dây SCL (22), SDA (21), GND và VCC");
    }

    // Khởi tạo LCD
    Serial.println("Bắt đầu khởi tạo LCD...");

    // Thử lại kết nối I2C trước khi khởi tạo
    Wire.beginTransmission(0x27);
    byte error2 = Wire.endTransmission();

    if (error2 == 0)
    {
        Serial.println("Kết nối I2C vẫn ổn, tiếp tục khởi tạo LCD");
    }
    else
    {
        Serial.println("Vẫn không thể kết nối I2C, thử lại với tốc độ thấp hơn");
        Wire.setClock(10000); // Giảm tốc độ I2C xuống 10kHz
        Wire.beginTransmission(0x27);
        error2 = Wire.endTransmission();
        if (error2 == 0)
        {
            Serial.println("Kết nối I2C thành công với tốc độ thấp");
        }
        else
        {
            Serial.println("Vẫn không thể kết nối I2C, có thể có vấn đề với phần cứng");
        }
    }

    // Khởi tạo LCD
    lcd.init();
    Serial.println("LCD đã được khởi tạo");

    // Bật đèn nền
    lcd.backlight();
    Serial.println("Đã bật đèn nền LCD");

    delay(100);
    // Hiển thị thông báo
    lcd.setCursor(0, 0);
    lcd.print("Khoi dong...");
    Serial.println("Đã ghi 'Khoi dong...' lên LCD");

    // Cấu hình chân cảm biến và relay
    pinMode(SOIL_MOISTURE_PIN, INPUT);
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, HIGH); // Relay thường đóng, mức LOW để bật

    sys_wifi_init();
    sys_capserver_init();
    pinMode(BOOT_PIN, INPUT_PULLUP);                                                // Configure BOOT pin as input with internal pull-up resistor
    attachInterrupt(digitalPinToInterrupt(BOOT_PIN), bootInterruptHandler, RISING); // Attach interrupt handler to rising edge of BOOT pin

    // Hiển thị thông tin về MQTT topics cố định
    Serial.println("Sử dụng MQTT topics cố định:");
    Serial.print("MQTT Status Topic: ");
    Serial.println(mqtt_base_topic);
    Serial.print("MQTT Control Topic: ");
    Serial.println(mqtt_pump_control_topic);

    // Cấu hình MQTT với timeout ngắn hơn để tránh treo
    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(callback);
    client.setSocketTimeout(5); // Timeout 5 giây cho socket

    // Khởi tạo các biến MQTT
    newMqttMessageReceived = false;
    memset(receivedTopic, 0, sizeof(receivedTopic));
    memset(receivedPayload, 0, sizeof(receivedPayload));

    // Kiểm tra lại LCD trước khi bắt đầu các task
    Serial.println("Kiểm tra lại LCD...");

    // Thử tắt và bật đèn nền để kiểm tra
    lcd.noBacklight();
    delay(500);
    lcd.backlight();
    delay(500);

    // Không xóa màn hình, chỉ ghi đè lên nội dung hiện tại
    lcd.setCursor(0, 0);
    lcd.print("He thong san");
    lcd.setCursor(0, 1);
    lcd.print("sang hoat dong");
    Serial.println("Đã hiển thị thông báo khởi động");
    delay(2000);

    Serial.println("All Done!");

    // Tạo các task với ưu tiên khác nhau
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
        2,             // Task priority - cao hơn để đảm bảo xử lý MQTT kịp thời
        NULL           // Task handle
    );
}

void loop()
{
}
