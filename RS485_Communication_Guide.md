# Hướng dẫn Giao tiếp RS485 giữa ATmega328P và ESP32

## Tổng quan hệ thống

Hệ thống này bao gồm:

- **ATmega328P**: Đọc dữ liệu từ cảm biến SPS30 và gửi qua RS485
- **ESP32**: Nhận dữ liệu từ RS485, xử lý và gửi lên MQTT/WiFi

## Cấu hình phần cứng

### ATmega328P (Transmitter)

```
- RXD2: Pin 14 (Nhận từ MAX485)
- TXD2: Pin 15 (Gửi đến MAX485)
- RE_DE: Pin 12 (Điều khiển gửi/nhận MAX485)
- SPS30: Kết nối qua UArt
```

### ESP32 (Receiver)

```
- RXD2: GPIO 34 (RO từ MAX485)
- TXD2: GPIO 35 (DI đến MAX485)
- RE_DE: GPIO 13 (Điều khiển gửi/nhận MAX485)
- GPS: UART1 (GPIO 24/26)
```

### Kết nối MAX485

```
ATmega328P ---- MAX485 ---- MAX485 ---- ESP32
    TXD2    -->    DI         RO    -->   RXD2
    RXD2    <--    RO         DI    <--   TXD2
    RE_DE   -->   RE/DE     RE/DE  -->   RE_DE
```

## Format dữ liệu chuẩn hóa

### Cấu trúc gói tin RS485

```
<SENSOR_TYPE>DATA_PAYLOAD</SENSOR_TYPE>\n
```

### Dữ liệu SPS30 từ ATmega

**Format gửi:**

```
<SPS30>TYPE:SPS30,PM1:1.23,PM2.5:2.34,PM4:3.45,PM10:4.56,NC0.5:123.4,NC1:234.5,NC2.5:345.6,NC4:456.7,NC10:567.8,SIZE:1.23</SPS30>
```

**Payload:**

```
TYPE:SPS30,PM1:1.23,PM2.5:2.34,PM4:3.45,PM10:4.56,NC0.5:123.4,NC1:234.5,NC2.5:345.6,NC4:456.7,NC10:567.8,SIZE:1.23
```

### Dữ liệu HDC1080 (tương thích)

**Format gửi:**

```
<HDC1080>TYPE:HDC1080,TEMP:25.50,HUM:60.30</HDC1080>
```

**Payload:**

```
TYPE:HDC1080,TEMP:25.50,HUM:60.30
```

## Cấu trúc dữ liệu

### SPS30Data (Cảm biến bụi mịn)

```cpp
struct SPS30Data {
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
```

### SensorData (HDC1080 - Nhiệt độ/Độ ẩm)

```cpp
struct SensorData {
    float temperature;  // Nhiệt độ [°C]
    float humidity;     // Độ ẩm [%]
    bool dataValid;
};
```

## Luồng hoạt động

### ATmega328P (Transmitter)

1. Đọc dữ liệu từ SPS30 mỗi 3 giây
2. Tạo chuỗi dữ liệu theo format chuẩn: `TYPE:SPS30,PM1:x,PM2.5:y,...`
3. Đóng gói với header/footer: `<SPS30>data</SPS30>\n`
4. Gửi qua RS485 với MAX485
5. Hiển thị dữ liệu trên Serial Monitor

### ESP32 (Receiver)

1. **Task 1**: Nhận dữ liệu từ RS485
   - Sử dụng buffer để lưu dữ liệu chưa hoàn chỉnh
   - Tìm kiếm gói tin hoàn chỉnh với header `<SPS30>` và footer `</SPS30>`
   - Kiểm tra TYPE để xác định loại cảm biến
   - Parse dữ liệu và đưa vào Queue tương ứng
2. **Task GPS**: Đọc dữ liệu GPS
3. **Task WiFi**: Quản lý kết nối WiFi
4. **Task MQTT**: Gửi dữ liệu lên MQTT broker

## JSON Output lên MQTT

### Với dữ liệu SPS30 + GPS

```json
{
  "device": "ESP32_RS485_Multi_Sensor",
  "pm1_0": 1.23,
  "pm2_5": 2.34,
  "pm4_0": 3.45,
  "pm10_0": 4.56,
  "nc0_5": 123.4,
  "nc1_0": 234.5,
  "nc2_5": 345.6,
  "nc4_0": 456.7,
  "nc10_0": 567.8,
  "typical_size": 1.23,
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

- Mỗi gói tin được bao bọc bởi `<SENSOR_TYPE>` và `</SENSOR_TYPE>`
- Đảm bảo tính toàn vẹn dữ liệu trong môi trường nhiễu
- Hỗ trợ phát hiện và loại bỏ dữ liệu bị cắt xén

### 2. **Type Identification**

- Mỗi payload bắt đầu với `TYPE:SENSOR_NAME`
- Tự động nhận diện loại cảm biến
- Hỗ trợ mở rộng cho nhiều loại cảm biến khác

### 3. **Buffer Management**

- ESP32 sử dụng buffer để xử lý dữ liệu không đồng bộ
- Tự động ghép nối các gói tin bị phân mảnh
- Giới hạn kích thước buffer để tránh tràn bộ nhớ

### 4. **Error Recovery**

- Kiểm tra format trước khi parse
- Bỏ qua dữ liệu không hợp lệ
- Tự động làm sạch buffer khi cần thiết

## Lưu ý quan trọng

1. **Baudrate**: Cả hai thiết bị sử dụng 9600 bps
2. **Timing**: ATmega gửi mỗi 3 giây, ESP32 kiểm tra mỗi 100ms
3. **Queue**: ESP32 sử dụng FreeRTOS Queue để đồng bộ dữ liệu
4. **Error handling**: Kiểm tra tính hợp lệ của dữ liệu trước khi xử lý
5. **Compatibility**: Hỗ trợ cả SPS30 và HDC1080 format
6. **Data Integrity**: Sử dụng header/footer để đảm bảo tính toàn vẹn
7. **Buffer Size**: Giới hạn buffer tối đa 500 ký tự

## Troubleshooting

1. **Không nhận được dữ liệu**: Kiểm tra kết nối MAX485 và RE/DE pin
2. **Dữ liệu sai**: Kiểm tra baudrate và format chuỗi
3. **MQTT không gửi**: Kiểm tra kết nối WiFi và MQTT broker
4. **GPS không hoạt động**: Kiểm tra kết nối UART1 và antenna GPS
