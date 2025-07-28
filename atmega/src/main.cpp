#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include "sensirion_uart.h"
#include "sensirion_shdlc.h"
#include "sps30.h"

// Chân UART cho SPS30
#define SPS30_RX 8
#define SPS30_TX 9

// Tạo SoftwareSerial cho SPS30
SoftwareSerial sps30Serial(SPS30_RX, SPS30_TX); // RX, TX

// Flag để kiểm tra trạng thái SPS30
bool sps30Available = false;

// SPS30 UART Implementation Functions
void sensirion_uart_init()
{
    sps30Serial.begin(115200);
}

int16_t sensirion_uart_tx(uint16_t data_len, const uint8_t *data)
{
    return sps30Serial.write(data, data_len);
}

int16_t sensirion_uart_rx(uint16_t max_data_len, uint8_t *data)
{
    int16_t i = 0;
    unsigned long start = millis();
    while (i < max_data_len && (millis() - start) < 100)
    { // Timeout 100ms
        if (sps30Serial.available())
        {
            data[i++] = sps30Serial.read();
        }
    }
    return i;
}

void sensirion_sleep_usec(uint32_t useconds)
{
    if (useconds >= 1000)
    {
        delay(useconds / 1000);
    }
    else
    {
        delayMicroseconds(useconds);
    }
}

// Hàm đọc dữ liệu HDC1000
bool getHDC1000Data(double *temperature, double *humidity)
{
    uint8_t Byte[4];
    uint16_t temp, hum;

    Wire.beginTransmission(0x40);
    Wire.write(0x00);
    Wire.endTransmission();
    delay(20);
    Wire.requestFrom(0x40, 4);

    if (Wire.available() >= 4)
    {
        Byte[0] = Wire.read();
        Byte[1] = Wire.read();
        Byte[2] = Wire.read();
        Byte[3] = Wire.read();

        temp = (((unsigned int)Byte[0] << 8) | Byte[1]);
        hum = (((unsigned int)Byte[2] << 8) | Byte[3]);
        *temperature = (double)(temp) / 65536.0 * 165.0 - 40.0;
        *humidity = (double)(hum) / 65536.0 * 100.0;

        if (*temperature < -40 || *temperature > 125 || *humidity > 100)
            return false;
        return true;
    }
    return false;
}

void setup()
{
    Serial.begin(115200);
    Wire.begin();          // Khởi tạo I2C cho HDC1000
    sensirion_uart_init(); // Khởi tạo UART cho SPS30

    // Cấu hình HDC1000
    Wire.beginTransmission(0x40);
    Wire.write(0x02);
    Wire.write(0x90);
    Wire.write(0x00);
    Wire.endTransmission();
    delay(20);
    Serial.println("HDC1000 initialized!");

    // Khởi tạo SPS30
    Serial.println("Starting SPS30...");
    int16_t ret = sps30_probe();
    if (ret != 0)
    {
        Serial.print("SPS30 not found! Error: ");
        Serial.println(ret);
        sps30Available = false;
    }
    else
    {
        Serial.println("SPS30 ready!");
        sps30Available = true;

        // Cấu hình auto cleaning
        ret = sps30_set_fan_auto_cleaning_interval_days(1);
        if (ret)
        {
            Serial.print("Set fan auto cleaning interval failed! Error: ");
            Serial.println(ret);
        }

        // Bắt đầu đo SPS30
        ret = sps30_start_measurement();
        delay(1000);
        if (ret != 0)
        {
            Serial.print("Start measurement failed! Error: ");
            Serial.println(ret);
            sps30Available = false;
        }
        else
        {
            Serial.println("SPS30 measurement started!");
        }
    }

    delay(1000);
}

void loop()
{
    static unsigned long previousMillis = 0;
    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= 2000)
    { // 2 giây
        previousMillis = currentMillis;

        // Đọc SPS30 trước
        struct sps30_measurement m;
        int16_t ret = sps30_read_measurement(&m);

        if (ret == 0)
        {
            Serial.println("=== SPS30 Data ===");
            Serial.print("PM1.0: ");
            Serial.print(m.mc_1p0);
            Serial.println(" μg/m³");
            Serial.print("PM2.5: ");
            Serial.print(m.mc_2p5);
            Serial.println(" μg/m³");
            Serial.print("PM4.0: ");
            Serial.print(m.mc_4p0);
            Serial.println(" μg/m³");
            Serial.print("PM10:  ");
            Serial.print(m.mc_10p0);
            Serial.println(" μg/m³");
            Serial.println("==================");
        }
        else
        {
            Serial.print("SPS30 read failed! Error: ");
            Serial.println(ret);
            Serial.println("[DEBUG] Attempting to reinitialize SPS30...");
            int16_t probeRet = sps30_probe();
            if (probeRet == 0)
            {
                Serial.println("[DEBUG] SPS30 probe OK. Setting auto fan cleaning...");
                int16_t fanRet = sps30_set_fan_auto_cleaning_interval_days(1);
                if (fanRet)
                {
                    Serial.print("[DEBUG] Set fan auto cleaning interval failed! Error: ");
                    Serial.println(fanRet);
                }
                Serial.println("[DEBUG] Starting SPS30 measurement...");
                int16_t startRet = sps30_start_measurement();
                delay(1000);
                if (startRet == 0)
                {
                    Serial.println("[DEBUG] SPS30 measurement started!");
                }
                else
                {
                    Serial.print("[DEBUG] Start measurement failed! Error: ");
                    Serial.println(startRet);
                }
            }
            else
            {
                Serial.print("[DEBUG] SPS30 probe failed! Error: ");
                Serial.println(probeRet);
            }
        }

        // Luôn đọc HDC1000 sau khi đọc SPS30
        double temperature, humidity;
        if (getHDC1000Data(&temperature, &humidity))
        {
            Serial.print("HDC1000: ");
            Serial.print(temperature, 1);
            Serial.print(" °C, ");
            Serial.print(humidity, 1);
            Serial.println("%");
        }
        else
        {
            Serial.println("HDC1000 read failed!");
        }
    }
}