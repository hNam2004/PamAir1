/*
 * ATmega328P RS485 Communication with SPS30 Sensor
 * Sends sensor data via RS485 to ESP32
 */

#include <Arduino.h>
#include <SoftwareSerial.h>
#include "sensirion_uart.h"
#include "sps30.h"

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

// Hàm gửi dữ liệu qua RS485
void sendRS485Data(String data)
{
  digitalWrite(RE_DE, HIGH); // Chế độ gửi
  delay(2);                  // Delay để ổn định
  RS485Serial.print(data);
  delay(10);                // Đợi gửi xong
  digitalWrite(RE_DE, LOW); // Chế độ nhận
}

// Hàm tạo chuỗi dữ liệu SPS30 theo format chuẩn
String createSPS30DataString(struct sps30_measurement *measurement)
{
  String dataString = "TYPE:SPS30";
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
  return dataString;
}

void setup()
{
  Serial.begin(115200); // Initialize Serial for console output
  while (!Serial)
    ; // Wait for Serial to be ready (needed for some Arduino boards)

  // Khởi tạo RS485
  pinMode(RE_DE, OUTPUT);
  digitalWrite(RE_DE, LOW); // Chế độ nhận mặc định
  RS485Serial.begin(115200);

  Serial.println("=== ATmega328P RS485 SPS30 STARTING ===");

  // Initialize UART communication
  while (sensirion_uart_open() != 0)
  {
    Serial.println("UART init failed");
    delay(1000); // Sleep for 1 second
  }

  // Probe the SPS30 sensor
  while (sps30_probe() != 0)
  {
    Serial.println("SPS30 sensor probing failed");
    delay(1000); // Sleep for 1 second
  }
  Serial.println("SPS30 sensor probing successful");

  // Read version information
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

  // Read serial number
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

  // Set fan auto-cleaning interval
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

  // Start measurement
  ret = sps30_start_measurement();
  if (ret < 0)
  {
    Serial.println("error starting measurement");
    delay(3000);
    return;
  }

  // Đọc dữ liệu SPS30 mỗi 3 giây
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

    // Tạo chuỗi dữ liệu theo format chuẩn
    String dataPayload = createSPS30DataString(&m);
    String rs485Message = "<SPS30>" + dataPayload + "</SPS30>\n";

    // Gửi qua RS485
    sendRS485Data(rs485Message);

    // Hiển thị dữ liệu đã gửi
    Serial.print("RS485 Sent: ");
    Serial.println(rs485Message);
  }

  // Kiểm tra lệnh từ Serial
  if (Serial.available())
  {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command == "send")
    {
      Serial.println("Manual send triggered");
      // Gửi dữ liệu ngay lập tức
      if (sps30_read_measurement(&m) >= 0)
      {
        String dataPayload = createSPS30DataString(&m);
        String rs485Message = "<SPS30>" + dataPayload + "</SPS30>\n";
        sendRS485Data(rs485Message);
        Serial.print("Manual RS485 Sent: ");
        Serial.println(rs485Message);
      }
    }
  }

  delay(3000); // Gửi dữ liệu mỗi 3 giây
}
