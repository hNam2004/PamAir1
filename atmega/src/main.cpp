/*
 * Copyright (c) 2018, Sensirion AG
 * All rights reserved.
 * (License details remain the same as provided in the original code)
 */

#include <Arduino.h>
#include "sensirion_uart.h"
#include "sps30.h"

struct sps30_measurement m;
char serial[SPS30_MAX_SERIAL_LEN];
const uint8_t AUTO_CLEAN_DAYS = 4;
struct sps30_version_information version_information;

void setup() {
  Serial.begin(115200); // Initialize Serial for console output
  while (!Serial); // Wait for Serial to be ready (needed for some Arduino boards)

  // Initialize UART communication
  while (sensirion_uart_open() != 0) {
    Serial.println("UART init failed");
    delay(1000); // Sleep for 1 second
  }

  // Probe the SPS30 sensor
  while (sps30_probe() != 0) {
    Serial.println("SPS30 sensor probing failed");
    delay(1000); // Sleep for 1 second
  }
  Serial.println("SPS30 sensor probing successful");

  // Read version information
  int16_t ret = sps30_read_version(&version_information);
  if (ret) {
    Serial.print("error ");
    Serial.print(ret);
    Serial.println(" reading version information");
  } else {
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
  if (ret) {
    Serial.print("error ");
    Serial.print(ret);
    Serial.println(" reading serial");
  } else {
    Serial.print("SPS30 Serial: ");
    Serial.println(serial);
  }

  // Set fan auto-cleaning interval
  ret = sps30_set_fan_auto_cleaning_interval_days(AUTO_CLEAN_DAYS);
  if (ret) {
    Serial.print("error ");
    Serial.print(ret);
    Serial.println(" setting the auto-clean interval");
  }
}

void loop() {
  int16_t ret;

  // Start measurement
  ret = sps30_start_measurement();
  if (ret < 0) {
    Serial.println("error starting measurement");
  } else {
    Serial.println("measurements started");
  }

  // Take measurements for 60 seconds
  for (int i = 0; i < 60; ++i) {
    ret = sps30_read_measurement(&m);
    if (ret < 0) {
      Serial.println("error reading measurement");
    } else {
      if (SPS30_IS_ERR_STATE(ret)) {
        Serial.print("Chip state: ");
        Serial.print(SPS30_GET_ERR_STATE(ret));
        Serial.println(" - measurements may not be accurate");
      }

      Serial.println("measured values:");
      Serial.print("\t");
      Serial.print(m.mc_1p0, 2);
      Serial.println(" pm1.0");
      Serial.print("\t");
      Serial.print(m.mc_2p5, 2);
      Serial.println(" pm2.5");
      Serial.print("\t");
      Serial.print(m.mc_4p0, 2);
      Serial.println(" pm4.0");
      Serial.print("\t");
      Serial.print(m.mc_10p0, 2);
      Serial.println(" pm10.0");
      Serial.print("\t");
      Serial.print(m.nc_0p5, 2);
      Serial.println(" nc0.5");
      Serial.print("\t");
      Serial.print(m.nc_1p0, 2);
      Serial.println(" nc1.0");
      Serial.print("\t");
      Serial.print(m.nc_2p5, 2);
      Serial.println(" nc2.5");
      Serial.print("\t");
      Serial.print(m.nc_4p0, 2);
      Serial.println(" nc4.0");
      Serial.print("\t");
      Serial.print(m.nc_10p0, 2);
      Serial.println(" nc10.0");
      Serial.print("\t");
      Serial.print(m.typical_particle_size, 2);
      Serial.println(" typical particle size");
      Serial.println();
    }
    delay(1000); // Sleep for 1 second
  }

  // Stop measurement to save power
  ret = sps30_stop_measurement();
  if (ret) {
    Serial.println("Stopping measurement failed");
  }

  // Enter sleep mode if firmware version supports it
  if (version_information.firmware_major >= 2) {
    ret = sps30_sleep();
    if (ret) {
      Serial.println("Entering sleep failed");
    }
  }

  Serial.println("No measurements for 1 minute");
  delay(60000); // Sleep for 1 minute

  // Wake up sensor if in sleep mode
  if (version_information.firmware_major >= 2) {
    ret = sps30_wake_up();
    if (ret) {
      Serial.print("Error ");
      Serial.print(ret);
      Serial.println(" waking up sensor");
    }
  }
}