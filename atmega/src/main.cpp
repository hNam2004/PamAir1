#define USE_UART // Comment dòng này để sử dụng I2C thay vì UART cho SPS30
#define PLOTTER_FORMAT // Comment dòng này để tắt định dạng serial plotter

#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#ifdef USE_UART
#include "sensirion_uart.h"
#include "sensirion_shdlc.h"
#endif
#include "sps30.h"

// Chân UART cho SPS30
#ifdef USE_UART
#define SPS30_RX 30
#define SPS30_TX 31
SoftwareSerial sps30Serial(SPS30_RX, SPS30_TX); // RX, TX
#endif

// Chân RS485
SoftwareSerial rs485(14, 15); // receive pin, transmit pin
const byte ENABLE_PIN = 12;

// Flag kiểm tra trạng thái SPS30
bool sps30Available = false;

#ifdef USE_UART
// SPS30 UART Implementation Functions
void sensirion_uart_init() {
    sps30Serial.begin(115200);
}

int16_t sensirion_uart_tx(uint16_t data_len, const uint8_t *data) {
    return sps30Serial.write(data, data_len);
}

int16_t sensirion_uart_rx(uint16_t max_data_len, uint8_t *data) {
    int16_t i = 0;
    unsigned long start = millis();
    while (i < max_data_len && (millis() - start) < 100) { // Timeout 100ms
        if (sps30Serial.available()) {
            data[i++] = sps30Serial.read();
        }
    }
    return i;
}

void sensirion_sleep_usec(uint32_t useconds) {
    if (useconds >= 1000) {
        delay(useconds / 1000);
    } else {
        delayMicroseconds(useconds);
    }
}
#endif

// Hàm đọc dữ liệu HDC1000
bool getHDC1000Data(double *temperature, double *humidity) {
    uint8_t Byte[4];
    uint16_t temp, hum;

    Wire.beginTransmission(0x40);
    Wire.write(0x00);
    Wire.endTransmission();
    delay(20);
    Wire.requestFrom(0x40, 4);

    if (Wire.available() >= 4) {
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

void setup() {
    Serial.begin(115200);
    Wire.begin(); // Khởi tạo I2C cho HDC1000
    rs485.begin(115200); // Khởi tạo SoftwareSerial cho RS485
    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, LOW); // Mặc định ở chế độ nhận

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
#ifdef USE_UART
    sensirion_uart_init();
#else
    sensirion_i2c_init();
#endif

    int16_t ret = sps30_probe();
    if (ret != 0) {
        Serial.print("SPS30 probe failed! Error: ");
        Serial.println(ret);
        sps30Available = false;
    } else {
        Serial.println("SPS30 ready!");
        sps30Available = true;

        // Cấu hình auto cleaning
        uint8_t auto_clean_days = 4;
        ret = sps30_set_fan_auto_cleaning_interval_days(auto_clean_days);
        if (ret) {
            Serial.print("Set fan auto cleaning interval failed! Error: ");
            Serial.println(ret);
        }

        // Bắt đầu đo SPS30
        ret = sps30_start_measurement();
        delay(1000);
        if (ret != 0) {
            Serial.print("Start measurement failed! Error: ");
            Serial.println(ret);
            sps30Available = false;
        } else {
            Serial.println("SPS30 measurement started!");
        }
    }

#ifdef SPS30_LIMITED_I2C_BUFFER_SIZE
    Serial.println("Your Arduino hardware has a limitation that only");
    Serial.println("allows reading the mass concentrations. For more");
    Serial.println("information, please check");
    Serial.println("https://github.com/Sensirion/arduino-sps#esp8266-partial-legacy-support");
#endif

    delay(1000);
}

void loop() {
    static unsigned long previousMillis = 0;
    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= 2000) { // 2 giây
        previousMillis = currentMillis;

        // Đọc SPS30
        struct sps30_measurement m;
        int16_t ret;
        uint16_t data_ready;

#ifdef USE_UART
        ret = sps30_read_measurement(&m);
#else
        do {
            ret = sps30_read_data_ready(&data_ready);
            if (ret < 0) {
                Serial.print("Error reading data-ready flag: ");
                Serial.println(ret);
            } else if (!data_ready) {
                Serial.println("Data not ready, no new measurement available");
            } else {
                break;
            }
            delay(100);
        } while (1);
        ret = sps30_read_measurement(&m);
#endif

        if (ret == 0) {
            sps30Available = true;
#ifndef PLOTTER_FORMAT
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
#ifndef SPS30_LIMITED_I2C_BUFFER_SIZE
            Serial.print("NC 0.5: ");
            Serial.println(m.nc_0p5);
            Serial.print("NC 1.0: ");
            Serial.println(m.nc_1p0);
            Serial.print("NC 2.5: ");
            Serial.println(m.nc_2p5);
            Serial.print("NC 4.0: ");
            Serial.println(m.nc_4p0);
            Serial.print("NC 10.0: ");
            Serial.println(m.nc_10p0);
            Serial.print("Typical particle size: ");
            Serial.println(m.typical_particle_size);
#endif
            Serial.println("==================");
#else
            Serial.print(m.nc_0p5);
            Serial.print(" ");
            Serial.print(m.nc_1p0 - m.nc_0p5);
            Serial.print(" ");
            Serial.print(m.nc_2p5 - m.nc_1p0);
            Serial.print(" ");
            Serial.print(m.nc_4p0 - m.nc_2p5);
            Serial.print(" ");
            Serial.print(m.nc_10p0 - m.nc_4p0);
            Serial.println();
#endif

            // Gửi dữ liệu SPS30 qua RS485
            String sps30Data = "TYPE:SPS30,PM1.0:" + String(m.mc_1p0, 2) + ",PM2.5:" + String(m.mc_2p5, 2) + ",PM4.0:" + String(m.mc_4p0, 2) + ",PM10:" + String(m.mc_10p0, 2);
            String sps30Packet = "<SPS30>" + sps30Data + "</SPS30>\n";
            digitalWrite(ENABLE_PIN, HIGH);
            delay(10);
            rs485.print(sps30Packet);
            rs485.flush();
            delay(10);
            digitalWrite(ENABLE_PIN, LOW);
        } else {
            Serial.print("SPS30 read failed! Error: ");
            Serial.println(ret);
            Serial.println("[DEBUG] Attempting to reinitialize SPS30...");
            int16_t probeRet = sps30_probe();
            if (probeRet == 0) {
                Serial.println("[DEBUG] SPS30 probe OK. Setting auto fan cleaning...");
                int16_t fanRet = sps30_set_fan_auto_cleaning_interval_days(4);
                if (fanRet) {
                    Serial.print("[DEBUG] Set fan auto cleaning interval failed! Error: ");
                    Serial.println(fanRet);
                }
                Serial.println("[DEBUG] Starting SPS30 measurement...");
                int16_t startRet = sps30_start_measurement();
                delay(1000);
                if (startRet == 0) {
                    Serial.println("[DEBUG] SPS30 measurement started!");
                } else {
                    Serial.print("[DEBUG] Start measurement failed! Error: ");
                    Serial.println(startRet);
                }
            } else {
                Serial.print("[DEBUG] SPS30 probe failed! Error: ");
                Serial.println(probeRet);
            }
            sps30Available = false;
        }

        // Đọc HDC1000
        double temperature, humidity;
        if (getHDC1000Data(&temperature, &humidity)) {
            Serial.print("HDC1000: ");
            Serial.print(temperature, 1);
            Serial.print(" °C, ");
            Serial.print(humidity, 1);
            Serial.println("%");

            // Gửi dữ liệu qua RS485
            String data = "TYPE:HDC1080,TEMP:" + String(temperature, 2) + ",HUM:" + String(humidity, 2);
            String packet = "<HDC1080>" + data + "</HDC1080>\n";
            digitalWrite(ENABLE_PIN, HIGH);
            delay(10);
            rs485.print(packet);
            rs485.flush();
            delay(10);
            digitalWrite(ENABLE_PIN, LOW);
        } else {
            Serial.println("HDC1000 read failed!");
        }
    }
}