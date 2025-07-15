#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>

// Chân điều khiển TX và Serial2 (SoftwareSerial)
#define TX_PIN 15
#define RX_PIN 14
#define TX2_PIN 12

// Tạo SoftwareSerial thay cho Serial2
SoftwareSerial Serial2(RX_PIN, TX2_PIN);

// Cấu trúc dữ liệu cảm biến HDC1000
struct HDC1000Data
{
    double temperature; // Nhiệt độ [°C]
    double humidity;    // Độ ẩm [%]
    bool dataValid;
};

// Biến chia sẻ dữ liệu giữa các nhiệm vụ
volatile HDC1000Data sharedSensorData;
volatile bool newDataAvailable = false;

// Thời gian kiểm tra nhiệm vụ
unsigned long previousTask1Millis = 0;
unsigned long previousTask2Millis = 0;
const unsigned long task1Interval = 2000; // 2 giây
const unsigned long task2Interval = 1000; // 1 giây

// Constants cho giao tiếp Serial2
byte start_byte_1 = 0x3C;
byte start_byte_2 = 0xC3;
unsigned int value = 0;

// Hàm đọc cảm biến HDC1000
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

// Hàm đọc dữ liệu HDC1000
bool getHDC1000Data(HDC1000Data *data)
{
    double temperature;
    double humidity;

    humidity = readSensor(&temperature);

    // Kiểm tra dữ liệu hợp lệ
    if (humidity < 0 || temperature < -40 || temperature > 125 || humidity > 100)
    {
        Serial.println("Loi doc du lieu HDC1000!");
        return false;
    }

    data->temperature = temperature;
    data->humidity = humidity;
    data->dataValid = true;

    return true;
}

// Gửi dữ liệu qua Serial2 với định dạng mới
void sendSerial2Data(String data)
{
    // Chuyển đổi string thành số để gửi
    unsigned int dataValue = data.toInt(); // Hoặc có thể hash string

    digitalWrite(TX_PIN, HIGH);
    delayMicroseconds(1200);

    Serial2.write(start_byte_1);
    Serial2.write(start_byte_2);
    Serial2.write((byte)(dataValue & 0x00FF));
    Serial2.write((byte)(dataValue >> 8));
    Serial2.flush();

    delayMicroseconds(1200);
    digitalWrite(TX_PIN, LOW);
}

// Nhiệm vụ 1: đọc cảm biến HDC1000 và gửi RS485
void task1Function()
{
    HDC1000Data sensorData;

    if (getHDC1000Data(&sensorData))
    {
        // Tạo chuỗi dữ liệu để gửi qua RS485
        String dataString = "TEMP:" + String(sensorData.temperature, 2) +
                            ",HUM:" + String(sensorData.humidity, 2);

        sendSerial2Data(dataString);

        Serial.print("Da gui qua Serial2: ");
        Serial.println(dataString);

        // Cập nhật dữ liệu chia sẻ
        noInterrupts();
        sharedSensorData.temperature = sensorData.temperature;
        sharedSensorData.humidity = sensorData.humidity;
        sharedSensorData.dataValid = sensorData.dataValid;
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
        HDC1000Data localData;
        noInterrupts();
        localData.temperature = sharedSensorData.temperature;
        localData.humidity = sharedSensorData.humidity;
        localData.dataValid = sharedSensorData.dataValid;
        newDataAvailable = false;
        interrupts();

        if (localData.dataValid)
        {
            Serial.println("=== Du lieu cam bien HDC1000 ===");
            Serial.print("Nhiet do: ");
            Serial.print(localData.temperature);
            Serial.println(" °C");
            Serial.print("Do am: ");
            Serial.print(localData.humidity);
            Serial.println(" %");
            Serial.println("================================");
        }
    }
}

void setup()
{
    // Cấu hình LED và TX_PIN
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    pinMode(TX_PIN, OUTPUT);
    digitalWrite(TX_PIN, HIGH);

    Serial.begin(115200);
    Serial2.begin(115200);
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
    Serial.println("ATmega328P gui du lieu HDC1000 qua Serial2");
    Serial.println("HDC1000 I2C: SDA=A4, SCL=A5");

    delay(1000);
}

void loop()
{
    int c;
    unsigned long currentMillis = millis();

    // Xử lý nhận dữ liệu từ Serial2
    while (!Serial.available() && Serial2.available() < 4)
        ;

    digitalWrite(LED_BUILTIN, HIGH);

    if (Serial2.available() >= 4)
    {
        if (Serial2.read() == start_byte_1)
        {
            if (Serial2.read() == start_byte_2)
            {
                value = Serial2.read();
                value += (((unsigned int)Serial2.read()) << 8) & 0xFF00;
                Serial.print("< ");
                Serial.println(value);
                value = 0;
            }
        }
    }

    // Xử lý gửi dữ liệu từ Serial
    if (Serial.available())
    {
        c = Serial.read();
        if ((c >= '0') && (c <= '9'))
            value = 10 * value + c - '0';
        else if (c == 's')
        {
            Serial.print("> ");
            Serial.println(value);
            digitalWrite(TX_PIN, HIGH);
            delayMicroseconds(1200);
            Serial2.write(start_byte_1);
            Serial2.write(start_byte_2);
            Serial2.write((byte)(value & 0x00FF));
            Serial2.write((byte)(value >> 8));
            Serial2.flush();
            delayMicroseconds(1200);
            digitalWrite(TX_PIN, LOW);
            value = 0;
        }
    }

    // Kiểm tra nếu đến thời gian thực hiện task1 (đọc cảm biến và gửi Serial2)
    if (currentMillis - previousTask1Millis >= task1Interval)
    {
        previousTask1Millis = currentMillis;
        task1Function();
    }

    // Kiểm tra nếu đến thời gian thực hiện task2 (hiển thị dữ liệu)
    if (currentMillis - previousTask2Millis >= task2Interval)
    {
        previousTask2Millis = currentMillis;
        task2Function();
    }

    delay(2);
    digitalWrite(LED_BUILTIN, LOW);
}
