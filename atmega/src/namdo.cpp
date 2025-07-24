#include <Arduino.h>
#include <Wire.h>

void setup() {
  Serial.begin(9600); 
  Wire.begin();

  // Khởi tạo HDC1080
  Wire.beginTransmission(0x40);
  Wire.write(0x02); 
  Wire.write(0x90); 
  Wire.write(0x00); 
  Serial.println("HDC1080 initialized");
  Wire.endTransmission();

  delay(20);
}

double readSensor(double* temperature)
{
  uint8_t Byte[4];
  int value;
  uint16_t temp;
  uint16_t humidity;

  Wire.beginTransmission(0x40);
  Wire.write(0x00); 
  Wire.endTransmission();
  delay(20);
  Wire.requestFrom(0x40, 4);

  if(4 <= Wire.available())
  {
    Byte[0] = Wire.read();
    Byte[1] = Wire.read();
    Byte[2] = Wire.read();
    Byte[3] = Wire.read();

    temp = (((unsigned int)Byte[0] <<8 | Byte[1]));
    *temperature = (double)(temp) / 65536 * 165 - 40;
    humidity = (((unsigned int)Byte[2] <<8 | Byte[3]));
    return (double)(humidity) / 65536 * 100;
  }
}

void loop() {
  double temperature;
  double humidity;

  humidity = readSensor(&temperature);

    Serial.print("Temperature: ");
    Serial.print(temperature ,2);
    Serial.print(" °C, Humidity: ");
    Serial.print(humidity, 2);
    Serial.println(" %");

    delay(1000);
}
