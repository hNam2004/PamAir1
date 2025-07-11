#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <sps30.h>

// Chân cho ATmega328P (sửa lại cho phù hợp)
#define RXD2 2
#define TXD2 3
#define RE_DE 4

// Khai báo SoftwareSerial cho RS485
SoftwareSerial RS485Serial(RXD2, TXD2);

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

// Hàm đọc dữ liệu từ SPS30
bool getSPS30Data(SPS30Data *data)
{
    struct sps30_measurement m;
    uint16_t data_ready;
    int16_t ret;

    // Kiểm tra dữ liệu có sẵn không
    ret = sps30_read_data_ready(&data_ready);
    if (ret < 0 || !data_ready)
    {
        return false;
    }

    // Đọc dữ liệu đo
    ret = sps30_read_measurement(&m);
    if (ret < 0)
    {
        Serial.println("Loi doc du lieu SPS30!");
        return false;
    }

    // Kiểm tra tính hợp lệ của dữ liệu
    if (isnan(m.mc_1p0) || isnan(m.mc_2p5) || isnan(m.mc_4p0) || isnan(m.mc_10p0))
    {
        Serial.println("Du lieu SPS30 khong hop le!");
        return false;
    }

    // Sao chép dữ liệu
    data->pm1_0 = m.mc_1p0;
    data->pm2_5 = m.mc_2p5;
    data->pm4_0 = m.mc_4p0;
    data->pm10_0 = m.mc_10p0;

#ifndef SPS30_LIMITED_I2C_BUFFER_SIZE
    data->nc0_5 = m.nc_0p5;
    data->nc1_0 = m.nc_1p0;
    data->nc2_5 = m.nc_2p5;
    data->nc4_0 = m.nc_4p0;
    data->nc10_0 = m.nc_10p0;
    data->typical_size = m.typical_particle_size;
#else
    // Nếu buffer hạn chế, đặt giá trị mặc định
    data->nc0_5 = 0;
    data->nc1_0 = 0;
    data->nc2_5 = 0;
    data->nc4_0 = 0;
    data->nc10_0 = 0;
    data->typical_size = 0;
#endif

    data->dataValid = true;
    return true;
}

// Gửi dữ liệu qua RS485
void sendRS485Data(String data)
{
    digitalWrite(RE_DE, HIGH);
    delay(RS485_DELAY_MS);
    RS485Serial.print(data);
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
        // Tạo chuỗi dữ liệu để gửi qua RS485
        String dataString = "PM1.0:" + String(sensorData.pm1_0, 2) +
                            ",PM2.5:" + String(sensorData.pm2_5, 2) +
                            ",PM4.0:" + String(sensorData.pm4_0, 2) +
                            ",PM10:" + String(sensorData.pm10_0, 2);

#ifndef SPS30_LIMITED_I2C_BUFFER_SIZE
        dataString += ",NC0.5:" + String(sensorData.nc0_5, 1) +
                      ",NC1.0:" + String(sensorData.nc1_0, 1) +
                      ",NC2.5:" + String(sensorData.nc2_5, 1) +
                      ",NC4.0:" + String(sensorData.nc4_0, 1) +
                      ",NC10:" + String(sensorData.nc10_0, 1) +
                      ",SIZE:" + String(sensorData.typical_size, 2);
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

    // Khởi tạo I2C và SPS30
    sensirion_i2c_init();

    while (sps30_probe() != 0)
    {
        Serial.println("SPS30 sensor probing failed!");
        delay(500);
    }

    Serial.println("SPS30 sensor probing successful!");

    // Cấu hình auto cleaning (4 ngày)
    int16_t ret = sps30_set_fan_auto_cleaning_interval_days(4);
    if (ret)
    {
        Serial.print("Loi cau hinh auto-clean interval: ");
        Serial.println(ret);
    }

    // Bắt đầu đo
    ret = sps30_start_measurement();
    if (ret < 0)
    {
        Serial.println("Loi bat dau do luong!");
        while (1)
            ; // Dừng nếu không thể bắt đầu đo
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
