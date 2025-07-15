# Hướng dẫn Giao tiếp RS485 HDC1080 giữa ATmega328P và ESP32

## Tổng quan hệ thống

Hệ thống này bao gồm:
- **ATmega328P**: Đọc dữ liệu từ cảm biến HDC1080 (nhiệt độ/độ ẩm) và gửi qua RS485
- **ESP32**: Nhận dữ liệu từ RS485, xử lý và gửi lên MQTT/WiFi

## Cấu hình phần cứng

### ATmega328P (Transmitter)
```
- RX_PIN: Pin 14 (Nhận từ MAX485)
- TX2_PIN: Pin 12 (Gửi đến MAX485)
- TX_PIN: Pin 15 (Điều khiển gửi/nhận MAX485)
- HDC1080: Kết nối qua I2C (SDA=A4, SCL=A5)
```

### ESP32 (Receiver)
```
- RXD2: GPIO 34 (RO từ MAX485)
- TXD2: GPIO 35 (DI đến MAX485)
- RE_DE: GPIO 13 (Điều khiển gửi/nhận MAX485)
- GPS: UART1 (chưa cấu hình chân cụ thể)
```

### Kết nối MAX485
```
ATmega328P ---- MAX485 ---- MAX485 ---- ESP32
    TX2_PIN -->    DI         RO    -->   RXD2
    RX_PIN  <--    RO         DI    <--   TXD2
    TX_PIN  -->   RE/DE     RE/DE  -->   RE_DE
```

## Format dữ liệu chuẩn hóa

### Cấu trúc gói tin RS485
```
<HDC1080>DATA_PAYLOAD</HDC1080>\n
```

### Dữ liệu HDC1080 từ ATmega
**Format gửi:**
```
<HDC1080>TYPE:HDC1080,TEMP:31.07,HUM:72.26</HDC1080>
```

**Payload:**
```
TYPE:HDC1080,TEMP:31.07,HUM:72.26
```

## Cấu trúc dữ liệu

### SensorData (HDC1080 - Nhiệt độ/Độ ẩm)
```cpp
struct SensorData {
    float temperature;  // Nhiệt độ [°C]
    float humidity;     // Độ ẩm [%]
    bool dataValid;
};
```

### HDC1000Data (ATmega)
```cpp
struct HDC1000Data {
    double temperature; // Nhiệt độ [°C]
    double humidity;    // Độ ẩm [%]
    bool dataValid;
};
```

## Luồng hoạt động

### ATmega328P (Transmitter)
1. Đọc dữ liệu từ HDC1080 qua I2C mỗi 2 giây
2. Tạo chuỗi dữ liệu theo format chuẩn: `TYPE:HDC1080,TEMP:x,HUM:y`
3. Đóng gói với header/footer: `<HDC1080>data</HDC1080>\n`
4. Gửi qua RS485 với MAX485 (baudrate 115200)
5. Hiển thị dữ liệu trên Serial Monitor

### ESP32 (Receiver)
1. **Task 1**: Nhận dữ liệu từ RS485
   - Sử dụng buffer để lưu dữ liệu chưa hoàn chỉnh
   - Tìm kiếm gói tin hoàn chỉnh với header `<HDC1080>` và footer `</HDC1080>`
   - Kiểm tra TYPE để xác định loại cảm biến
   - Parse dữ liệu và đưa vào Queue
2. **Task GPS**: Đọc dữ liệu GPS (chưa cấu hình chân)
3. **Task WiFi**: Quản lý kết nối WiFi
4. **Task MQTT**: Gửi dữ liệu lên MQTT broker

## JSON Output lên MQTT

### Với dữ liệu HDC1080 + GPS
```json
{
  "device": "ESP32_RS485_HDC1080",
  "sensor_type": "HDC1080",
  "temperature": 31.07,
  "humidity": 72.26,
  "latitude": 21.028511,
  "longitude": 105.804817,
  "satellites": 8,
  "altitude": 12.5
}
```

## Cấu hình MQTT
```cpp
const char *mqtt_server = "broker.emqx.io";
const int mqtt_port = 1883;
const char *mqtt_topic = "esp32/test11";
```

## Tính năng chuẩn hóa

### 1. **Header/Footer Protection**
- Mỗi gói tin được bao bọc bởi `<HDC1080>` và `</HDC1080>`
- Đảm bảo tính toàn vẹn dữ liệu trong môi trường nhiễu
- Hỗ trợ phát hiện và loại bỏ dữ liệu bị cắt xén

### 2. **Type Identification**
- Mỗi payload bắt đầu với `TYPE:HDC1080`
- Tự động nhận diện loại cảm biến
- Hỗ trợ tương thích ngược với format cũ

### 3. **Buffer Management**
- ESP32 sử dụng buffer để xử lý dữ liệu không đồng bộ
- Tự động ghép nối các gói tin bị phân mảnh
- Giới hạn kích thước buffer để tránh tràn bộ nhớ

## Lưu ý quan trọng

1. **Baudrate**: Cả hai thiết bị sử dụng 115200 bps
2. **Timing**: ATmega gửi mỗi 2 giây, ESP32 kiểm tra liên tục
3. **Queue**: ESP32 sử dụng FreeRTOS Queue để đồng bộ dữ liệu
4. **Error handling**: Kiểm tra tính hợp lệ của dữ liệu trước khi xử lý
5. **Compatibility**: Hỗ trợ cả format mới và cũ
6. **Data Integrity**: Sử dụng header/footer để đảm bảo tính toàn vẹn
7. **Buffer Size**: Giới hạn buffer tối đa 500 ký tự

## Dữ liệu mẫu từ Serial Monitor

Dựa trên hình ảnh bạn cung cấp, dữ liệu sẽ có dạng:
```
Temperature: 31.10 °C, Humidity: 72.45 %
Temperature: 31.07 °C, Humidity: 72.36 %
Temperature: 31.09 °C, Humidity: 72.36 %
Temperature: 31.07 °C, Humidity: 72.26 %
```

## Troubleshooting

1. **Không nhận được dữ liệu**: Kiểm tra kết nối MAX485 và RE/DE pin
2. **Dữ liệu sai**: Kiểm tra baudrate (115200) và format chuỗi
3. **MQTT không gửi**: Kiểm tra kết nối WiFi và MQTT broker
4. **HDC1080 không hoạt động**: Kiểm tra kết nối I2C (SDA=A4, SCL=A5)
5. **GPS không hoạt động**: Cần cấu hình chân RX/TX cho GPS

## Lệnh debug

### ATmega328P
- Gửi `send` qua Serial để gửi dữ liệu ngay lập tức
- Monitor Serial để xem dữ liệu HDC1080

### ESP32
- Monitor Serial để xem dữ liệu nhận được từ RS485
- Kiểm tra WiFi và MQTT connection status
