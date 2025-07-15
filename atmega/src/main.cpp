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
    Serial.println("ATmega328P gui du lieu HDC1000 qua RS485");
    Serial.println("HDC1000 I2C: SDA=A4, SCL=A5");
    Serial.println("RS485: RX=14, TX=15, RE_DE=12");

    delay(1000);
}

void loop()
{
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

    // Kiểm tra nếu đến thời gian thực hiện task2 (hiển thị dữ liệu)
    if (currentMillis - previousTask2Millis >= task2Interval)
    {
        previousTask2Millis = currentMillis;
        task2Function();
    }

    delay(2);
    digitalWrite(LED_BUILTIN, LOW);
}
