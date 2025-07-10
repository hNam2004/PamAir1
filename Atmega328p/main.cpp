#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include "Adafruit_HDC1000.h"

// TinyRTOS includes
#include <TinyRTOS.h>

// Chân cho ATmega328P
#define RXD2 13  // RO từ MAX485 (Digital Pin 13)
#define TXD2 17  // DI đến MAX485 (Digital Pin 17)
#define RE_DE 23 // Điều khiển gửi / nhận (Digital Pin 23)

// Khai báo SoftwareSerial cho RS485
SoftwareSerial RS485Serial(RXD2, TXD2);

// Khai báo đối tượng HDC1000
Adafruit_HDC1000 hdc = Adafruit_HDC1000();

// Cấu trúc dữ liệu cảm biến
struct SensorData
{
    float temperature;
    float humidity;
    bool dataValid;
};

// Biến chia sẻ dữ liệu giữa các task
volatile SensorData sharedSensorData;
volatile bool newDataAvailable = false;

// Hàm đọc dữ liệu thực từ HDC1000
float getHDC1000Temperature()
{
    float temp = hdc.readTemperature();

    // Kiểm tra dữ liệu hợp lệ
    if (isnan(temp) || temp < -40 || temp > 125)
    {
        Serial.println("Loi doc nhiet do HDC1000!");
        return -999; // Trả về giá trị lỗi
    }

    return temp;
}

float getHDC1000Humidity()
{
    float hum = hdc.readHumidity();

    // Kiểm tra dữ liệu hợp lệ
    if (isnan(hum) || hum < 0 || hum > 100)
    {
        Serial.println("Loi doc do am HDC1000!");
        return -999; // Trả về giá trị lỗi
    }

    return hum;
}

// Hàm gửi dữ liệu qua RS485
void sendRS485Data(String data)
{
    digitalWrite(RE_DE, HIGH); // Chuyển sang chế độ gửi
    delay(1);

    RS485Serial.print(data);
    RS485Serial.flush();

    delay(1);
}

// Task 1: Đọc cảm biến và gửi RS485
void task1Function()
{
    float temperature = getHDC1000Temperature();
    float humidity = getHDC1000Humidity();

    if (temperature != -999 && humidity != -999)
    {
        String dataString = "TEMP:" + String(temperature, 2) + ",HUM:" + String(humidity, 2);

        sendRS485Data(dataString);

        Serial.print("Da gui qua RS485: ");
        Serial.println(dataString);

        // Lưu dữ liệu vào biến chia sẻ
        noInterrupts();
        sharedSensorData.temperature = temperature;
        sharedSensorData.humidity = humidity;
        sharedSensorData.dataValid = true;
        newDataAvailable = true;
        interrupts();
    }
    else
    {
        Serial.println("Loi lay du lieu cam bien HDC1000!");
    }
}

// Task 2: Hiển thị dữ liệu
void task2Function()
{
    if (newDataAvailable)
    {
        SensorData localData;

        noInterrupts();
        localData = sharedSensorData;
        newDataAvailable = false;
        interrupts();

        if (localData.dataValid)
        {
            Serial.print("Du lieu cam bien - Nhiet do: ");
            Serial.print(localData.temperature);
            Serial.print(" °C  |  Do am: ");
            Serial.print(localData.humidity);
            Serial.println(" %");
        }
    }
}

void setup()
{
    Serial.begin(9600);

    // Khởi tạo I2C và HDC1000
    Wire.begin();
    if (!hdc.begin())
    {
        Serial.println("Khong tim thay HDC1000!");
        Serial.println("Kiem tra ket noi I2C...");
        while (1)
            ; // Dừng chương trình nếu không tìm thấy HDC1000
    }
    Serial.println("HDC1000 da khoi tao thanh cong!");

    // Khởi tạo RS485
    pinMode(RE_DE, OUTPUT);
    digitalWrite(RE_DE, LOW);
    RS485Serial.begin(9600);
    Serial.println("RS485 da khoi tao");

    Serial.println("Cam bien HDC1000 gui du lieu qua RS485");
    Serial.println("ATmega328P with TinyRTOS ready!");

    // Khởi tạo TinyRTOS
    TinyRTOS_Init();

    // Tạo tasks với TinyRTOS
    TinyRTOS_CreateTask(task1Function, 2000); // Task 1 chạy mỗi 2 giây
    TinyRTOS_CreateTask(task2Function, 1000); // Task 2 chạy mỗi 1 giây

    Serial.println("TinyRTOS tasks created!");

    // Bắt đầu scheduler
    TinyRTOS_Start();
}

void loop()
{
    // TinyRTOS scheduler sẽ quản lý
    TinyRTOS_Schedule();
}
