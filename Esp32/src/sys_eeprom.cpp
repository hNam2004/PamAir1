#include <EEPROM.h>
#include <sys_eeprom.hpp>
#include <Arduino.h>

char sys_eeprom_ssid[MAX_SSID_LENGTH];
char sys_eeprom_password[MAX_PASSWORD_LENGTH];
char sys_eeprom_device_id[MAX_DEVICE_ID_LENGTH];

void clearWiFiCredentialsInEEPROM()
{
    EEPROM.begin(512);

    // Clear SSID
    for (int i = 0; i < MAX_SSID_LENGTH; i++)
    {
        EEPROM.write(SSID_EEPROM_ADDR + i, '\0');
        sys_eeprom_ssid[i] = '\0';
    }

    // Clear password
    for (int i = 0; i < MAX_PASSWORD_LENGTH; i++)
    {
        EEPROM.write(PASSWORD_EEPROM_ADDR + i, '\0');
        sys_eeprom_password[i] = '\0';
    }

    EEPROM.commit();
}

void readWiFiCredentialsFromEEPROM()
{
    EEPROM.begin(512);
    
    // Đọc SSID
    for (int i = 0; i < MAX_SSID_LENGTH; i++)
    {
        sys_eeprom_ssid[i] = EEPROM.read(SSID_EEPROM_ADDR + i);
    }
    // Đảm bảo kết thúc chuỗi
    sys_eeprom_ssid[MAX_SSID_LENGTH - 1] = '\0';

    // Đọc password
    for (int i = 0; i < MAX_PASSWORD_LENGTH; i++)
    {
        sys_eeprom_password[i] = EEPROM.read(PASSWORD_EEPROM_ADDR + i);
    }
    // Đảm bảo kết thúc chuỗi
    sys_eeprom_password[MAX_PASSWORD_LENGTH - 1] = '\0';
}

void saveWiFiCredentialsToEEPROM(const char *ssid, const char *password)
{
    EEPROM.begin(512);
    
    // Lưu SSID
    for (int i = 0; i < MAX_SSID_LENGTH; i++)
    {
        if (i < strlen(ssid)) {
            EEPROM.write(SSID_EEPROM_ADDR + i, ssid[i]);
            sys_eeprom_ssid[i] = ssid[i];
        } else {
            EEPROM.write(SSID_EEPROM_ADDR + i, '\0');
            sys_eeprom_ssid[i] = '\0';
        }
    }

    // Lưu password
    for (int i = 0; i < MAX_PASSWORD_LENGTH; i++)
    {
        if (i < strlen(password)) {
            EEPROM.write(PASSWORD_EEPROM_ADDR + i, password[i]);
            sys_eeprom_password[i] = password[i];
        } else {
            EEPROM.write(PASSWORD_EEPROM_ADDR + i, '\0');
            sys_eeprom_password[i] = '\0';
        }
    }

    EEPROM.commit();
}

void readDeviceIdFromEEPROM()
{
    EEPROM.begin(512);
    
    // Đọc device ID
    for (int i = 0; i < MAX_DEVICE_ID_LENGTH; i++) {
        sys_eeprom_device_id[i] = EEPROM.read(DEVICE_ID_EEPROM_ADDR + i);
    }
    
    // Đảm bảo kết thúc chuỗi
    sys_eeprom_device_id[MAX_DEVICE_ID_LENGTH - 1] = '\0';
    
    // Nếu chuỗi rỗng hoặc không hợp lệ, đặt thành rỗng để tạo mới
    if (sys_eeprom_device_id[0] == '\0' || sys_eeprom_device_id[0] == 0xFF) {
        sys_eeprom_device_id[0] = '\0';
    }
}

void saveDeviceIdToEEPROM(const char *device_id)
{
    EEPROM.begin(512);
    
    // Lưu device ID
    for (int i = 0; i < MAX_DEVICE_ID_LENGTH; i++)
    {
        if (i < strlen(device_id)) {
            EEPROM.write(DEVICE_ID_EEPROM_ADDR + i, device_id[i]);
            sys_eeprom_device_id[i] = device_id[i];
        } else {
            EEPROM.write(DEVICE_ID_EEPROM_ADDR + i, '\0');
            sys_eeprom_device_id[i] = '\0';
        }
    }
    
    EEPROM.commit();
}

void generateAndSaveDeviceId()
{
    // Đảm bảo EEPROM đã được bắt đầu
    EEPROM.begin(512);
    
    // Tạo số ngẫu nhiên 8 chữ số
    randomSeed(analogRead(0) + millis());
    long randomNumber = random(10000000, 99999999);
    
    // Chuyển thành chuỗi
    char deviceId[MAX_DEVICE_ID_LENGTH];
    snprintf(deviceId, MAX_DEVICE_ID_LENGTH, "ESP%ld", randomNumber);
    
    // Lưu vào EEPROM
    saveDeviceIdToEEPROM(deviceId);
    
    Serial.print("Generated new device ID: ");
    Serial.println(deviceId);
}
