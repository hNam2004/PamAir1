#include <Arduino.h>
#include <SoftwareSerial.h>
#include "sensirion_uart.h"
#include "sensirion_shdlc.h"
#include "sensirion_uart.h"
#include "sps30.h"
// Chân UART cho SPS30
#define SPS30_RX 8
#define SPS30_TX 9

SoftwareSerial sps30Serial(SPS30_RX, SPS30_TX); // RX, TX

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
    { // timeout 100ms
        if (sps30Serial.available())
        {
            data[i++] = sps30Serial.read();
        }
    }
    return i;
}

void sensirion_sleep_usec(uint32_t useconds)
{
    delayMicroseconds(useconds);
}

void setup()
{
    Serial.begin(115200); // Đổi baud rate
    sensirion_uart_init();

    Serial.println("Starting SPS30...");

    int16_t ret = sps30_probe();
    if (ret != 0)
    {
        Serial.print("SPS30 not found! Error: ");
        Serial.println(ret);
    }
    else
    {
        Serial.println("SPS30 ready!");

        // Bắt đầu đo
        ret = sps30_set_fan_auto_cleaning_interval_days(1);
        if (ret){
            Serial.print("Set fan auto cleaning interval failed! Error: ");
            Serial.println(ret);
        }
        
        ret = sps30_start_measurement();
        if (ret != 0)
        {
            Serial.print("Start measurement failed! Error: ");
            Serial.println(ret);
        }
        else
        {
            Serial.println("Measurement started!");
        }
    }
}

void loop()
{
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
        Serial.print("Read measurement failed! Error: ");
        Serial.println(ret);
    }

    delay(2000); // Đọc mỗi 2 giây
}