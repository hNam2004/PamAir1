#include <Arduino.h>
#include <SoftwareSerial.h>
#include "sensirion_uart.h"
#include "sps30.h"
#include <Wire.h>

// Cấu hình RS485
#define RXD2 10 // Pin 14 (Nhận từ MAX485)
#define TXD2 11 // Pin 15 (Gửi đến MAX485)
#define RE_DE 8 // Pin 12 (Điều khiển gửi/nhận MAX485)

// Khởi tạo SoftwareSerial cho RS485
SoftwareSerial RS485Serial(RXD2, TXD2);

struct sps30_measurement m;
char serial[SPS30_MAX_SERIAL_LEN];
const uint8_t AUTO_CLEAN_DAYS = 4;
struct sps30_version_information version_information;

double readSensor(double* temperature)
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

    temp = (Byte[0] << 8) | Byte[1];
    *temperature = (double)temp / 65536 * 165 - 40;
    humidity = (Byte[2] << 8) | Byte[3];
    return (double)humidity / 65536 * 100;
  }
  else
  {
    *temperature = 0.0; // Giá trị mặc định nếu lỗi
    return -1.0;        // Báo lỗi
  }
}

// Hàm gửi dữ liệu qua RS485
void sendRS485Data(String data)
{
  digitalWrite(RE_DE, HIGH); // Chế độ gửi
  delay(2);                  // Đợi ổn định
  RS485Serial.print(data);
  RS485Serial.flush();       // Đảm bảo dữ liệu được gửi hết
  delay(10);                 // Đợi gửi xong
  digitalWrite(RE_DE, LOW);  // Chuyển về chế độ nhận
}

// Hàm tạo chuỗi dữ liệu HDC1080 theo định dạng ESP32 mong đợi
String createHDC1080DataString(double temperature, double humidity)
{
  String dataString = "<HDC1080>";
  dataString += "TEMP:" + String(temperature, 2);
  dataString += ",HUM:" + String(humidity, 2);
  dataString += "</HDC1080>\n";
  return dataString;
}

// Hàm tạo chuỗi dữ liệu SPS30 theo định dạng ESP32 mong đợi
String createSPS30DataString(struct sps30_measurement *measurement)
{
  String dataString = "<SPS30>TYPE:SPS30";
  dataString += ",PM1:" + String(measurement->mc_1p0, 2);
  dataString += ",PM2.5:" + String(measurement->mc_2p5, 2);
  dataString += ",PM4:" + String(measurement->mc_4p0, 2);
  dataString += ",PM10:" + String(measurement->mc_10p0, 2);
  dataString += ",NC0.5:" + String(measurement->nc_0p5, 1);
  dataString += ",NC1:" + String(measurement->nc_1p0, 1);
  dataString += ",NC2.5:" + String(measurement->nc_2p5, 1);
  dataString += ",NC4:" + String(measurement->nc_4p0, 1);
  dataString += ",NC10:" + String(measurement->nc_10p0, 1);
  dataString += ",SIZE:" + String(measurement->typical_particle_size, 2);
  dataString += "</SPS30>\n";
  return dataString;
}

void setup()
{
  Serial.begin(115200); // Khởi tạo Serial cho debug
  while (!Serial)
    ; // Đợi Serial sẵn sàng
  Wire.begin();

  // Khởi tạo HDC1080
  Wire.beginTransmission(0x40);
  Wire.write(0x02); 
  Wire.write(0x90); 
  Wire.write(0x00); 
  Serial.println("HDC1080 initialized");
  Wire.endTransmission();

  delay(20);

  // Khởi tạo RS485
  pinMode(RE_DE, OUTPUT);
  digitalWrite(RE_DE, LOW); // Chế độ nhận mặc định
  RS485Serial.begin(115200);

  Serial.println("=== ATmega328P RS485 SPS30 STARTING ===");

  // Khởi tạo giao tiếp UART cho SPS30
  while (sensirion_uart_open() != 0)
  {
    Serial.println("UART init failed");
    delay(1000);
  }

  // Kiểm tra cảm biến SPS30
  while (sps30_probe() != 0)
  {
    Serial.println("SPS30 sensor probing failed");
    delay(1000);
  }
  Serial.println("SPS30 sensor probing successful");

  // Đọc thông tin phiên bản SPS30
  int16_t ret = sps30_read_version(&version_information);
  if (ret)
  {
    Serial.print("error ");
    Serial.print(ret);
    Serial.println(" reading version information");
  }
  else
  {
    Serial.print("FW: ");
    Serial.print(version_information.firmware_major);
    Serial.print(".");
    Serial.print(version_information.firmware_minor);
    Serial.print(" HW: ");
    Serial.print(version_information.hardware_revision);
    Serial.print(", SHDLC: ");
    Serial.print(version_information.shdlc_major);
    Serial.print(".");
    Serial.println(version_information.shdlc_minor);
  }

  // Đọc số serial của SPS30
  ret = sps30_get_serial(serial);
  if (ret)
  {
    Serial.print("error ");
    Serial.print(ret);
    Serial.println(" reading serial");
  }
  else
  {
    Serial.print("SPS30 Serial: ");
    Serial.println(serial);
  }

  // Đặt chu kỳ tự làm sạch quạt
  ret = sps30_set_fan_auto_cleaning_interval_days(AUTO_CLEAN_DAYS);
  if (ret)
  {
    Serial.print("error ");
    Serial.print(ret);
    Serial.println(" setting the auto-clean interval");
  }

  Serial.println("RS485 SPS30 ready - Starting measurements...");
}

void loop()
{
  int16_t ret;
  double temperature;
  double humidity;

  // Đọc dữ liệu từ HDC1080
  humidity = readSensor(&temperature);
  if (humidity >= 0)
  {
    Serial.print("Temperature: ");
    Serial.print(temperature, 2);
    Serial.print(" °C, Humidity: ");
    Serial.print(humidity, 2);
    Serial.println(" %");

    // Tạo và gửi dữ liệu HDC1080 qua RS485
    String hdc1080Message = createHDC1080DataString(temperature, humidity);
    sendRS485Data(hdc1080Message);
    Serial.print("RS485 Sent HDC1080: ");
    Serial.println(hdc1080Message);
  }
  else
  {
    Serial.println("Lỗi đọc cảm biến HDC1080");
  }

  // Bắt đầu đo lường SPS30
  ret = sps30_start_measurement();
  if (ret < 0)
  {
    Serial.println("error starting measurement");
    delay(3000);
    return;
  }

  // Đọc dữ liệu SPS30
  ret = sps30_read_measurement(&m);
  if (ret < 0)
  {
    Serial.println("error reading measurement");
  }
  else
  {
    if (SPS30_IS_ERR_STATE(ret))
    {
      Serial.print("Chip state: ");
      Serial.print(SPS30_GET_ERR_STATE(ret));
      Serial.println(" - measurements may not be accurate");
    }

    // Hiển thị dữ liệu trên Serial Monitor
    Serial.print("PM1.0: ");
    Serial.print(m.mc_1p0, 2);
    Serial.print(" | PM2.5: ");
    Serial.print(m.mc_2p5, 2);
    Serial.print(" | PM4.0: ");
    Serial.print(m.mc_4p0, 2);
    Serial.print(" | PM10: ");
    Serial.println(m.mc_10p0, 2);

    // Tạo và gửi dữ liệu SPS30 qua RS485
    String sps30Message = createSPS30DataString(&m);
    sendRS485Data(sps30Message);
    Serial.print("RS485 Sent SPS30: ");
    Serial.println(sps30Message);
  }

  // Kiểm tra lệnh từ Serial
  if (Serial.available())
  {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command == "send")
    {
      Serial.println("Manual send triggered");
      // Gửi dữ liệu HDC1080
      humidity = readSensor(&temperature);
      if (humidity >= 0)
      {
        String hdc1080Message = createHDC1080DataString(temperature, humidity);
        sendRS485Data(hdc1080Message);
        Serial.print("Manual RS485 Sent HDC1080: ");
        Serial.println(hdc1080Message);
      }

      // Gửi dữ liệu SPS30
      if (sps30_read_measurement(&m) >= 0)
      {
        String sps30Message = createSPS30DataString(&m);
        sendRS485Data(sps30Message);
        Serial.print("Manual RS485 Sent SPS30: ");
        Serial.println(sps30Message);
      }
    }
  }

  delay(3000); // Gửi dữ liệu mỗi 3 giây
}