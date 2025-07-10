#include <Arduino.h>
#include <TinyGPS++.h>
#include <WiFi.h>
#include <HardwareSerial.h>

// Cấu hình Wi-Fi
const char* ssid = "TEN_WIFI";
const char* password = "MAT_KHAU_WIFI";

// Cấu hình server (ví dụ: Thingspeak, Firebase, hoặc server riêng)
const char* serverUrl = "api.thingspeak.com";
const int serverPort = 80;

// Khởi tạo UART2 cho GPS (RX2=16, TX2=17) - Đã kết nối chân 2,3
HardwareSerial gpsSerial(2); // UART2: RX=16 (GPIO16), TX=17 (GPIO17)
TinyGPSPlus gps;

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600, SERIAL_8N1, 2, 3); // RX=2, TX=3 (đã đấu dây)

  // Kết nối Wi-Fi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected! IP: " + WiFi.localIP().toString());
}

void loop() {
  // Đọc dữ liệu GPS
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      if (gps.location.isValid()) {
        sendToServer(gps.location.lat(), gps.location.lng());
      }
    }
  }

  // Kiểm tra lỗi GPS
  if (millis() > 5000 && gps.charsProcessed() < 10) {
    Serial.println("No GPS signal!");
  }
  delay(2000); // Gửi dữ liệu mỗi 2 giây
}

// Gửi dữ liệu lên server qua HTTP GET
void sendToServer(float lat, float lng) {
  WiFiClient client;

  if (client.connect(serverUrl, serverPort)) {
    String url = "/update?api_key=YOUR_API_KEY&field1=" + String(lat, 6) + "&field2=" + String(lng, 6);
    client.print(String("GET ") + url + " HTTP/1.1\r\n" +
                 "Host: " + serverUrl + "\r\n" +
                 "Connection: close\r\n\r\n");

    Serial.println("Sent: " + url);
    client.stop();
  } else {
    Serial.println("Failed to connect to server");
  }
}