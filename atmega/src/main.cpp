#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include "sensirion_uart.h"
#include "sensirion_shdlc.h"
#include "sps30.h"

// Chân điều khiển TX và Serial2 (SoftwareSerial)
#define TX_PIN 15
#define RX_PIN 14
#define TX2_PIN 12

// Chân UART cho SPS30
#define SPS30_RX 8
#define SPS30_TX 9

// Tạo SoftwareSerial thay cho Serial2
SoftwareSerial Serial2(RX_PIN, TX2_PIN);
SoftwareSerial sps30Serial(SPS30_RX, SPS30_TX); // RX, TX cho SPS30

// Cấu trúc dữ liệu cảm biến HDC1000
struct HDC1000Data
{
    double temperature; // Nhiệt độ [°C]
    double humidity;    // Độ ẩm [%]
    bool dataValid;
};

// Cấu trúc dữ liệu cảm biến SPS30
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

// Biến chia sẻ dữ liệu giữa các nhiệm vụ
volatile HDC1000Data sharedHDCData;
volatile SPS30Data sharedSPS30Data;
volatile bool newHDCDataAvailable = false;
volatile bool newSPS30DataAvailable = false;

// Thời gian kiểm tra nhiệm vụ - Tối ưu intervals
unsigned long previousTask1Millis = 0;
unsigned long previousTask2Millis = 0;
unsigned long previousTask3Millis = 0;
const unsigned long task1Interval = 3000; // 3 giây - HDC1000 (giảm tần suất)
const unsigned long task2Interval = 5000; // 5 giây - SPS30 (tăng để ổn định)
const unsigned long task3Interval = 1000; // 1 giây - Display data

// Flags để tối ưu hiển thị
bool sps30Available = false;

// Constants cho giao tiếp Serial2
byte start_byte_1 = 0x3C;
byte start_byte_2 = 0xC3;
unsigned int value = 0;

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
    if (useconds >= 1000)
    {
        delay(useconds / 1000);
    }
    else
    {
        delayMicroseconds(useconds);
    }
}

// Hàm đọc cảm biến HDC1000 (tối ưu)
double readSensor(double *temperature)
{
    uint8_t Byte[4];
    uint16_t temp;
    uint16_t humidity;

    Wire.beginTransmission(0x40);
    Wire.write(0x00);
    Wire.endTransmission();
    delay(20);
    Wire.requestFrom(0x40, 4);

    if (4 <= Wire.available())
    {
        Byte[0] = Wire.read();
        Byte[1] = Wire.read();
        Byte[2] = Wire.read();
        Byte[3] = Wire.read();

        temp = (((unsigned int)Byte[0] << 8) | Byte[1]);
        *temperature = (double)(temp) / 65536.0 * 165.0 - 40.0;

        humidity = (((unsigned int)Byte[2] << 8) | Byte[3]);

        return (double)(humidity) / 65536.0 * 100.0;
    }

    return -1; // Lỗi đọc
}

// Hàm đọc dữ liệu HDC1000 (tối ưu)
bool getHDC1000Data(HDC1000Data *data)
{
    double temperature;
    double humidity;

    humidity = readSensor(&temperature);

    // Kiểm tra dữ liệu hợp lệ
    if (humidity < 0 || temperature < -40 || temperature > 125 || humidity > 100)
    {
        return false; // Không in lỗi để giảm Serial output
    }

    data->temperature = temperature;
    data->humidity = humidity;
    data->dataValid = true;

    return true;
}

// Hàm đọc dữ liệu SPS30 (tối ưu với error handling)
bool getSPS30Data(SPS30Data *data)
{
    if (!sps30Available)
    {
        Serial.println("Debug: SPS30 khong available!");
        data->dataValid = false;
        return false;
    }

    Serial.println("Debug: Bat dau doc SPS30...");
    struct sps30_measurement m;
    int16_t ret = sps30_read_measurement(&m);
    Serial.print("Debug: SPS30 read result = ");
    Serial.println(ret);

    if (ret == 0)
    {
        // Kiểm tra dữ liệu hợp lệ trước khi sao chép
        if (m.mc_1p0 >= 0 && m.mc_2p5 >= 0 && m.mc_4p0 >= 0 && m.mc_10p0 >= 0)
        {
            data->pm1_0 = m.mc_1p0;
            data->pm2_5 = m.mc_2p5;
            data->pm4_0 = m.mc_4p0;
            data->pm10_0 = m.mc_10p0;
            data->nc0_5 = m.nc_0p5;
            data->nc1_0 = m.nc_1p0;
            data->nc2_5 = m.nc_2p5;
            data->nc4_0 = m.nc_4p0;
            data->nc10_0 = m.nc_10p0;
            data->typical_size = m.typical_particle_size;
            data->dataValid = true;
            return true;
        }
    }

    // In lỗi để debug
    Serial.print("SPS30 read failed! Error: ");
    Serial.println(ret);

    data->dataValid = false;
    return false;
}

// Gửi dữ liệu qua RS485 với định dạng chuẩn
void sendRS485Data(String data)
{
    digitalWrite(TX_PIN, HIGH); // Chế độ gửi
    delayMicroseconds(1200);

    // Thêm header và footer để đảm bảo tính toàn vẹn dữ liệu
    String formattedData = "<HDC1080>" + data + "</HDC1080>\n";
    Serial2.print(formattedData);
    Serial2.flush();

    delayMicroseconds(1200);
    digitalWrite(TX_PIN, LOW); // Chế độ nhận
}

// Nhiệm vụ 1: đọc cảm biến HDC1000 và gửi RS485
void task1Function()
{
    HDC1000Data sensorData;

    if (getHDC1000Data(&sensorData))
    {
        // Tạo chuỗi dữ liệu theo định dạng chuẩn để gửi qua RS485
        // Format: TYPE:HDC1080,TEMP:value,HUM:value
        String dataString = "TYPE:HDC1080";
        dataString += ",TEMP:" + String(sensorData.temperature, 2);
        dataString += ",HUM:" + String(sensorData.humidity, 2);

        sendRS485Data(dataString);

        Serial.print("Da gui qua RS485: ");
        Serial.println(dataString);

        // Cập nhật dữ liệu chia sẻ
        noInterrupts();
        sharedHDCData.temperature = sensorData.temperature;
        sharedHDCData.humidity = sensorData.humidity;
        sharedHDCData.dataValid = sensorData.dataValid;
        newHDCDataAvailable = true;
        interrupts();
    }
    else
    {
        Serial.println("Loi lay du lieu cam bien HDC1000!");
    }
}

// Nhiệm vụ 2: đọc cảm biến SPS30 và gửi Serial2
void task2Function()
{
    if (!sps30Available)
        return; // Skip nếu SPS30 không có

    SPS30Data sensorData;

    if (getSPS30Data(&sensorData))
    {
        // Tạo chuỗi dữ liệu SPS30 (tối ưu string concatenation)
        String dataString = "PM1:" + String(sensorData.pm1_0, 1) +
                            ",PM2.5:" + String(sensorData.pm2_5, 1) +
                            ",PM4:" + String(sensorData.pm4_0, 1) +
                            ",PM10:" + String(sensorData.pm10_0, 1);

        sendRS485Data(dataString);

        // Cập nhật dữ liệu chia sẻ
        noInterrupts();
        sharedSPS30Data.pm1_0 = sensorData.pm1_0;
        sharedSPS30Data.pm2_5 = sensorData.pm2_5;
        sharedSPS30Data.pm4_0 = sensorData.pm4_0;
        sharedSPS30Data.pm10_0 = sensorData.pm10_0;
        sharedSPS30Data.dataValid = sensorData.dataValid;
        newSPS30DataAvailable = true;
        interrupts();
    }
}

// Nhiệm vụ 3: hiển thị dữ liệu tổng hợp (tối ưu)
void task3Function()
{
    bool hasData = false;

    // Hiển thị HDC1000 nếu có dữ liệu mới
    if (newHDCDataAvailable)
    {
        HDC1000Data localData;
        noInterrupts();
        localData.temperature = sharedHDCData.temperature;
        localData.humidity = sharedHDCData.humidity;
        localData.dataValid = sharedHDCData.dataValid;
        newHDCDataAvailable = false;
        interrupts();

        if (localData.dataValid)
        {
            Serial.print("HDC: ");
            Serial.print(localData.temperature, 1);
            Serial.print("°C, ");
            Serial.print(localData.humidity, 1);
            Serial.print("%");
            hasData = true;
        }
    }

    // Hiển thị SPS30 nếu có dữ liệu mới
    if (newSPS30DataAvailable && sps30Available)
    {
        SPS30Data localData;
        noInterrupts();
        localData.pm1_0 = sharedSPS30Data.pm1_0;
        localData.pm2_5 = sharedSPS30Data.pm2_5;
        localData.pm4_0 = sharedSPS30Data.pm4_0;
        localData.pm10_0 = sharedSPS30Data.pm10_0;
        localData.dataValid = sharedSPS30Data.dataValid;
        newSPS30DataAvailable = false;
        interrupts();

        if (localData.dataValid)
        {
            if (hasData)
                Serial.print(" | ");
            Serial.print("SPS30: PM2.5=");
            Serial.print(localData.pm2_5, 1);
            Serial.print(", PM10=");
            Serial.print(localData.pm10_0, 1);
            hasData = true;
        }
    }

    if (hasData)
        Serial.println();
}

void setup()
{
    // Cấu hình LED và TX_PIN
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    pinMode(TX_PIN, OUTPUT);
    digitalWrite(TX_PIN, HIGH);

    Serial.begin(115200);
    Serial2.begin(115200); // Baudrate phải khớp với ESP32
    Serial.println("Setup done");
    delay(2000);

    // Khởi tạo I2C cho HDC1000
    Wire.begin();

    // Cấu hình HDC1000
    Wire.beginTransmission(0x40);
    Wire.write(0x02);
    Wire.write(0x90);
    Wire.write(0x00);
    Wire.endTransmission();

    delay(20);

    Serial.println("HDC1000 khoi tao thanh cong!");

    // Khởi tạo SPS30 UART
    Serial.println("Starting SPS30...");
    sensirion_uart_init();

    int16_t ret = sps30_probe();
    if (ret != 0)
    {
        Serial.print("SPS30 not found! Error: ");
        Serial.println(ret);
        Serial.println("Tiep tuc voi HDC1000...");
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

    Serial.println("ATmega328P gui du lieu HDC1000 + SPS30 qua Serial2");
    Serial.println("HDC1000 I2C: SDA=A4, SCL=A5");
    Serial.println("SPS30 UART: Sensirion protocol");

    delay(1000);
}

void loop()
{
    double temperature;
    double humidity;
    humidity = readSensor(&temperature);

    Serial.print("Temperature: ");
    Serial.print(temperature, 2);
    Serial.print(" °C, Humidity: ");
    Serial.print(humidity, 2);
    Serial.println(" %");

    int c;
    unsigned long currentMillis = millis();

    // Xử lý nhận dữ liệu từ RS485 (nếu cần)
    if (Serial2.available())
    {
        String receivedData = Serial2.readString();
        receivedData.trim();

        // Kiểm tra xem có phải là lệnh từ ESP32 không
        if (receivedData.indexOf("<ESP32>") >= 0 && receivedData.indexOf("</ESP32>") >= 0)
        {
            // Trích xuất dữ liệu giữa header và footer
            int startIndex = receivedData.indexOf("<ESP32>") + 7;
            int endIndex = receivedData.indexOf("</ESP32>");
            String command = receivedData.substring(startIndex, endIndex);

            Serial.print("Nhan lenh tu ESP32: ");
            Serial.println(command);

            // Xử lý lệnh nếu cần
            if (command == "GET_DATA")
            {
                // Gửi dữ liệu ngay lập tức
                task1Function();
            }
        }
    }

    // Xử lý lệnh từ Serial (để debug)
    if (Serial.available())
    {
        String command = Serial.readStringUntil('\n');
        command.trim();

        if (command == "send")
        {
            // Gửi dữ liệu ngay lập tức
            task1Function();
        }
    }

    // Kiểm tra nếu đến thời gian thực hiện task1 (đọc cảm biến và gửi Serial2)
    if (currentMillis - previousTask1Millis >= task1Interval)
    {
        previousTask1Millis = currentMillis;
        task1Function();
    }

    // Kiểm tra nếu đến thời gian thực hiện task2 (hiển thị dữ liệu HDC1000)
    if (currentMillis - previousTask2Millis >= task2Interval)
    {
        previousTask2Millis = currentMillis;
        task2Function();
    }

    // Kiểm tra nếu đến thời gian thực hiện task3 (hiển thị dữ liệu)
    if (currentMillis - previousTask3Millis >= task3Interval)
    {
        previousTask3Millis = currentMillis;
        task3Function();
    }

    delay(2);
    digitalWrite(LED_BUILTIN, LOW);
}
