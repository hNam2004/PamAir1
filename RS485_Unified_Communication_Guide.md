# Hướng dẫn Code RS485 Đồng nhất cho ESP32 và ATmega328P

## Tổng quan

Code này cung cấp giao tiếp RS485 đồng nhất giữa ATmega328P (gửi) và ESP32 (nhận/gửi) với khả năng:

- **ATmega328P**: Đọc SPS30 và gửi dữ liệu qua RS485
- **ESP32**: Nhận dữ liệu RS485, xử lý và gửi MQTT, đồng thời có thể gửi lệnh qua RS485

## Cấu hình phần cứng

### ATmega328P (Transmitter)
```
- RXD2: Pin 14 (Nhận từ MAX485)
- TXD2: Pin 15 (Gửi đến MAX485)
- RE_DE: Pin 12 (Điều khiển gửi/nhận MAX485)
- SPS30: Kết nối qua UART Serial
```

### ESP32 (Receiver/Transmitter)
```
- RXD2: GPIO 34 (RO từ MAX485)
- TXD2: GPIO 35 (DI đến MAX485)
- RE_DE: GPIO 13 (Điều khiển gửi/nhận MAX485)
```

### Kết nối MAX485
```
ATmega328P ---- MAX485 ---- MAX485 ---- ESP32
    TXD2    -->    DI         RO    -->   RXD2
    RXD2    <--    RO         DI    <--   TXD2
    RE_DE   -->   RE/DE     RE/DE  -->   RE_DE
```

## Format dữ liệu chuẩn

### SPS30 Data Format
```
<SPS30>TYPE:SPS30,PM1:1.23,PM2.5:2.34,PM4:3.45,PM10:4.56,NC0.5:123.4,NC1:234.5,NC2.5:345.6,NC4:456.7,NC10:567.8,SIZE:1.23</SPS30>
```

### HDC1080 Data Format (tương thích)
```
<HDC1080>TYPE:HDC1080,TEMP:25.50,HUM:60.30</HDC1080>
```

### Test Message Format
```
<TEST>ESP32_TEST_MESSAGE</TEST>
```

## Chức năng chính

### ATmega328P Functions

#### `sendRS485Data(String data)`
- Gửi dữ liệu qua RS485
- Tự động điều khiển RE/DE pin
- Có delay để đảm bảo ổn định

#### `createSPS30DataString(struct sps30_measurement* measurement)`
- Tạo chuỗi dữ liệu SPS30 theo format chuẩn
- Bao gồm tất cả thông số PM và NC

### ESP32 Functions

#### `sendRS485Data(String data)`
- Gửi dữ liệu qua RS485 từ ESP32
- Tự động điều khiển RE/DE pin
- Sử dụng flush() để đảm bảo gửi hoàn tất

#### `processHDC1080Data(String receivedData)`
- Xử lý dữ liệu HDC1080 nhận được
- Parse temperature và humidity
- Tạo JSON và gửi MQTT

#### `processSPS30Data(String receivedData)`
- Xử lý dữ liệu SPS30 nhận được
- Parse các giá trị PM
- Tạo JSON và gửi MQTT

#### `processRS485AndSendMQTT()`
- Nhận và xử lý dữ liệu RS485
- Tự động phân loại theo header
- Buffer management để tránh tràn bộ nhớ

## Lệnh điều khiển

### ATmega328P Serial Commands
```
send        - Gửi dữ liệu SPS30 ngay lập tức
```

### ESP32 Serial Commands
```
send_test   - Gửi tin nhắn test qua RS485
send:data   - Gửi dữ liệu tùy chỉnh qua RS485
status      - Hiển thị trạng thái hệ thống
```

## Luồng hoạt động

### ATmega328P
1. Khởi tạo RS485 và SPS30
2. Đọc dữ liệu SPS30 mỗi 3 giây
3. Tạo chuỗi dữ liệu theo format chuẩn
4. Gửi qua RS485 với header/footer
5. Hiển thị dữ liệu trên Serial Monitor
6. Hỗ trợ lệnh manual send

### ESP32
1. Khởi tạo RS485, WiFi, MQTT
2. Liên tục nhận dữ liệu từ RS485
3. Parse dữ liệu theo header (SPS30/HDC1080)
4. Xử lý và gửi lên MQTT
5. Hỗ trợ gửi lệnh qua RS485
6. Quản lý buffer và kết nối

## JSON Output Examples

### SPS30 MQTT Message
```json
{
  "device": "ESP32_RS485_SPS30",
  "pm1_0": 1.23,
  "pm2_5": 2.34,
  "pm4_0": 3.45,
  "pm10_0": 4.56,
  "timestamp": 12345678
}
```

### HDC1080 MQTT Message
```json
{
  "device": "ESP32_RS485_HDC1080",
  "temperature": 25.50,
  "humidity": 60.30,
  "timestamp": 12345678
}
```

## Cấu hình Baudrate
- **RS485**: 115200 bps
- **Serial Monitor**: 115200 bps (ATmega), 9600 bps (ESP32)

## Tính năng bảo vệ
- Buffer overflow protection (max 500 chars)
- Automatic MQTT reconnection
- Error handling cho sensor readings
- RE/DE pin automatic control
- Memory management

## Troubleshooting

### Không nhận được dữ liệu
1. Kiểm tra kết nối MAX485
2. Kiểm tra baudrate (115200)
3. Kiểm tra RE/DE pin connections
4. Kiểm tra power supply

### Dữ liệu bị lỗi
1. Kiểm tra format header/footer
2. Kiểm tra buffer overflow
3. Kiểm tra timing delays

### MQTT không gửi
1. Kiểm tra WiFi connection
2. Kiểm tra MQTT broker settings
3. Kiểm tra JSON format

## Debug Commands

### ATmega328P Debug
```
Serial Monitor -> send
```

### ESP32 Debug
```
Serial Monitor -> status
Serial Monitor -> send_test
Serial Monitor -> send:<custom_data>
```

## Lưu ý quan trọng

1. **Timing**: ATmega gửi mỗi 3 giây, ESP32 check mỗi 100ms
2. **Buffer**: Tự động dọn dẹp khi > 500 chars
3. **Error Recovery**: Tự động reconnect MQTT và WiFi
4. **Compatibility**: Hỗ trợ cả SPS30 và HDC1080 format
5. **Bidirectional**: ESP32 có thể gửi lệnh về ATmega (nếu cần)
