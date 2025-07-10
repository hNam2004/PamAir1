#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include "Adafruit_HDC1000.h"

// Chân cho ATmega328P
#define RXD2 13
#define TXD2 17
#define RE_DE 23

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

// Biến chia sẻ dữ liệu giữa các nhiệm vụ
volatile SensorData sharedSensorData;
volatile bool newDataAvailable = false;

// Thời gian kiểm tra nhiệm vụ
unsigned long previousTask1Millis = 0;
unsigned long previousTask2Millis = 0;
const unsigned long task1Interval = 2000; // 2 giây
const unsigned long task2Interval = 1000; // 1 giây

// Hàm đọc dữ liệu thực từ HDC1000
float getHDC1000Temperature()
{
    float temp = hdc.readTemperature();
    if (isnan(temp) || temp < -40 || temp > 125)
    {
        Serial.println("Loi doc nhiet do HDC1000!");
        return -999;
    }
    return temp;
}

float getHDC1000Humidity()
{
    float hum = hdc.readHumidity();
    if (isnan(hum) || hum < 0 || hum > 100)
    {
        Serial.println("Loi doc do am HDC1000!");
        return -999;
    }
    return hum;
}

// Gửi dữ liệu qua RS485
void sendRS485Data(String data)
{
    digitalWrite(RE_DE, HIGH);
    delay(1);
    RS485Serial.print(data);
    RS485Serial.flush();
    delay(1);
    digitalWrite(RE_DE, LOW); // quay lại chế độ nhận
}

// Nhiệm vụ 1: đọc cảm biến và gửi RS485
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

// Nhiệm vụ 2: hiển thị dữ liệu
void task2Function()
{
    if (newDataAvailable)
    {
        SensorData localData;
        noInterrupts();
        localData.temperature = sharedSensorData.temperature;
        localData.humidity = sharedSensorData.humidity;
        localData.dataValid = sharedSensorData.dataValid;
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

    Wire.begin();
    if (!hdc.begin())
    {
        Serial.println("Khong tim thay HDC1000!");
        while (1)
            ;
    }

    pinMode(RE_DE, OUTPUT);
    digitalWrite(RE_DE, LOW);
    RS485Serial.begin(9600);

    Serial.println("Cam bien HDC1000 gui du lieu qua RS485");
    Serial.println("ATmega328P khong dung RTOS, dung millis() de tao task logic.");
}

void loop()
{
    unsigned long currentMillis = millis();

    // Kiểm tra nếu đến thời gian thực hiện task1
    if (currentMillis - previousTask1Millis >= task1Interval)
    {
        previousTask1Millis = currentMillis;
        task1Function();
    }

    // Kiểm tra nếu đến thời gian thực hiện task2
    if (currentMillis - previousTask2Millis >= task2Interval)
    {
        previousTask2Millis = currentMillis;
        task2Function();
    }
}
