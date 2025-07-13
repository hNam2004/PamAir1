#include <Arduino.h>
#include <SoftwareSerial.h>
#include "sps30.h"

// Chân cho Max485
#define RXD2 14  // RS485 RX
#define TXD2 15  // RS485 TX
#define RE_DE 12 // RS485 Direction Control

#define SPS30_RX 30 // SPS30 UART RX (ATmega328P TX)
#define SPS30_TX 31 // SPS30 UART TX (ATmega328P RX)

// Khai báo SoftwareSerial cho RS485 và SPS30
SoftwareSerial RS485Serial(RXD2, TXD2);
SoftwareSerial SPS30Serial(SPS30_TX, SPS30_RX); // RX, TX

// Khai báo đối tượng SPS30 (sử dụng thư viện paulvha)
SPS30 sps30;

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
volatile SPS30Data sharedSensorData;
volatile bool newDataAvailable = false;

// Thời gian kiểm tra nhiệm vụ
unsigned long previousTask1Millis = 0;
unsigned long previousTask2Millis = 0;
const unsigned long task1Interval = 3000; // 3 giây (SPS30 cần thời gian đọc)
const unsigned long task2Interval = 1000; // 1 giây

// Constants
#define SENSOR_ERROR_VALUE -999.0f
#define RS485_DELAY_MS 2

// Hàm đọc dữ liệu từ SPS30 (sử dụng thư viện paulvha)
bool getSPS30Data(SPS30Data *data)
{
    struct sps_values val;
    uint8_t ret;

    // Đọc dữ liệu từ SPS30
    ret = sps30.GetValues(&val);
    if (ret != SPS30_ERR_OK)
    {
        Serial.print("Loi doc du lieu SPS30: ");
        Serial.println(ret);
        return false;
    }

    // Kiểm tra tính hợp lệ của dữ liệu
    if (isnan(val.MassPM1) || isnan(val.MassPM2) || isnan(val.MassPM4) || isnan(val.MassPM10))
    {
        Serial.println("Du lieu SPS30 khong hop le!");
        return false;
    }

    // Sao chép dữ liệu
    data->pm1_0 = val.MassPM1;
    data->pm2_5 = val.MassPM2;
    data->pm4_0 = val.MassPM4;
    data->pm10_0 = val.MassPM10;
    data->nc0_5 = val.NumPM0;
    data->nc1_0 = val.NumPM1;
    data->nc2_5 = val.NumPM2;
    data->nc4_0 = val.NumPM4;
    data->nc10_0 = val.NumPM10;
    data->typical_size = val.PartSize;
    data->dataValid = true;

    return true;
}

// Gửi dữ liệu qua RS485 với định dạng chuẩn
void sendRS485Data(String data)
{
    digitalWrite(RE_DE, HIGH);
    delay(RS485_DELAY_MS);

    // Thêm header và footer để đảm bảo tính toàn vẹn dữ liệu
    String formattedData = "<SPS30>" + data + "</SPS30>\n";
    RS485Serial.print(formattedData);
    RS485Serial.flush();

    delay(RS485_DELAY_MS);
    digitalWrite(RE_DE, LOW); // quay lại chế độ nhận
}

// Nhiệm vụ 1: đọc cảm biến SPS30 và gửi RS485
void task1Function()
{
    SPS30Data sensorData;

    if (getSPS30Data(&sensorData))
    {
        // Tạo chuỗi dữ liệu theo định dạng chuẩn để gửi qua RS485
        // Format: TYPE:SPS30,PM1:value,PM2.5:value,PM4:value,PM10:value[,NC0.5:value,NC1:value,NC2.5:value,NC4:value,NC10:value,SIZE:value]
        String dataString = "TYPE:SPS30";
        dataString += ",PM1:" + String(sensorData.pm1_0, 2);
        dataString += ",PM2.5:" + String(sensorData.pm2_5, 2);
        dataString += ",PM4:" + String(sensorData.pm4_0, 2);
        dataString += ",PM10:" + String(sensorData.pm10_0, 2);

#ifndef SPS30_LIMITED_I2C_BUFFER_SIZE
        dataString += ",NC0.5:" + String(sensorData.nc0_5, 1);
        dataString += ",NC1:" + String(sensorData.nc1_0, 1);
        dataString += ",NC2.5:" + String(sensorData.nc2_5, 1);
        dataString += ",NC4:" + String(sensorData.nc4_0, 1);
        dataString += ",NC10:" + String(sensorData.nc10_0, 1);
        dataString += ",SIZE:" + String(sensorData.typical_size, 2);
#endif

        sendRS485Data(dataString);

        Serial.print("Da gui qua RS485: ");
        Serial.println(dataString);

        // Cập nhật dữ liệu chia sẻ
        noInterrupts();
        sharedSensorData.pm1_0 = sensorData.pm1_0;
        sharedSensorData.pm2_5 = sensorData.pm2_5;
        sharedSensorData.pm4_0 = sensorData.pm4_0;
        sharedSensorData.pm10_0 = sensorData.pm10_0;
        sharedSensorData.nc0_5 = sensorData.nc0_5;
        sharedSensorData.nc1_0 = sensorData.nc1_0;
        sharedSensorData.nc2_5 = sensorData.nc2_5;
        sharedSensorData.nc4_0 = sensorData.nc4_0;
        sharedSensorData.nc10_0 = sensorData.nc10_0;
        sharedSensorData.typical_size = sensorData.typical_size;
        sharedSensorData.dataValid = sensorData.dataValid;
        newDataAvailable = true;
        interrupts();
    }
    else
    {
        Serial.println("Loi lay du lieu cam bien SPS30!");
    }
}

// Nhiệm vụ 2: hiển thị dữ liệu
void task2Function()
{
    if (newDataAvailable)
    {
        SPS30Data localData;
        noInterrupts();
        localData.pm1_0 = sharedSensorData.pm1_0;
        localData.pm2_5 = sharedSensorData.pm2_5;
        localData.pm4_0 = sharedSensorData.pm4_0;
        localData.pm10_0 = sharedSensorData.pm10_0;
        localData.nc0_5 = sharedSensorData.nc0_5;
        localData.nc1_0 = sharedSensorData.nc1_0;
        localData.nc2_5 = sharedSensorData.nc2_5;
        localData.nc4_0 = sharedSensorData.nc4_0;
        localData.nc10_0 = sharedSensorData.nc10_0;
        localData.typical_size = sharedSensorData.typical_size;
        localData.dataValid = sharedSensorData.dataValid;
        newDataAvailable = false;
        interrupts();

        if (localData.dataValid)
        {
            Serial.println("=== Du lieu cam bien SPS30 ===");
            Serial.print("PM1.0: ");
            Serial.print(localData.pm1_0);
            Serial.println(" μg/m³");
            Serial.print("PM2.5: ");
            Serial.print(localData.pm2_5);
            Serial.println(" μg/m³");
            Serial.print("PM4.0: ");
            Serial.print(localData.pm4_0);
            Serial.println(" μg/m³");
            Serial.print("PM10:  ");
            Serial.print(localData.pm10_0);
            Serial.println(" μg/m³");

#ifndef SPS30_LIMITED_I2C_BUFFER_SIZE
            Serial.print("NC0.5: ");
            Serial.print(localData.nc0_5);
            Serial.println(" #/cm³");
            Serial.print("NC1.0: ");
            Serial.print(localData.nc1_0);
            Serial.println(" #/cm³");
            Serial.print("NC2.5: ");
            Serial.print(localData.nc2_5);
            Serial.println(" #/cm³");
            Serial.print("NC4.0: ");
            Serial.print(localData.nc4_0);
            Serial.println(" #/cm³");
            Serial.print("NC10:  ");
            Serial.print(localData.nc10_0);
            Serial.println(" #/cm³");
            Serial.print("Kich thuoc TB: ");
            Serial.print(localData.typical_size);
            Serial.println(" μm");
#endif
            Serial.println("==============================");
        }
    }
}

void setup()
{
    Serial.begin(9600);
    delay(2000);

    // Cấu hình RS485
    pinMode(RE_DE, OUTPUT);
    digitalWrite(RE_DE, LOW);
    RS485Serial.begin(9600);

    // Khởi tạo UART cho SPS30
    SPS30Serial.begin(115200); // SPS30 UART default baudrate

    // Khởi tạo SPS30 với UART (sử dụng thư viện paulvha)
    if (!sps30.begin(SERIALPORT2))
    {
        Serial.println("Khong the khoi tao SPS30 qua UART!");
        while (1)
            ;
    }

    // Cấu hình SPS30 để sử dụng SoftwareSerial
    sps30.EnableDebugging(0); // Tắt debug

    if (!sps30.probe())
    {
        Serial.println("SPS30 sensor probing failed!");
        while (1)
            ;
    }

    Serial.println("SPS30 sensor probing successful!");

    // Cấu hình auto cleaning (4 ngày)
    uint8_t ret = sps30.SetAutoCleanInt(4 * 24 * 3600); // 4 ngày tính bằng giây
    if (ret != SPS30_ERR_OK)
    {
        Serial.print("Loi cau hinh auto-clean interval: ");
        Serial.println(ret);
    }

    // Bắt đầu đo
    if (!sps30.start())
    {
        Serial.println("Loi bat dau do luong!");
        while (1)
            ;
    }

    Serial.println("SPS30 bat dau do luong thanh cong!");
    Serial.println("ATmega328P gui du lieu SPS30 qua RS485");

#ifdef SPS30_LIMITED_I2C_BUFFER_SIZE
    Serial.println("Chu y: Chi doc duoc Mass Concentration do gioi han buffer I2C");
#endif

    delay(1000);
}

void loop()
{
    unsigned long currentMillis = millis();

    // Kiểm tra nếu đến thời gian thực hiện task1 (đọc cảm biến và gửi RS485)
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
}
