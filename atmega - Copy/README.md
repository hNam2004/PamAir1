# SPS30 Particulate Matter Sensor Project

This project interfaces with the Sensirion SPS30 particulate matter sensor using Arduino and PlatformIO.

## Hardware Requirements

- Arduino Uno or ATmega328P
- Sensirion SPS30 sensor
- Jumper wires for connections

## Wiring Connections

Connect the SPS30 sensor to your Arduino as follows:

| SPS30 Pin | Arduino Pin | Description |
|-----------|-------------|-------------|
| VDD       | 5V          | Power supply |
| GND       | GND         | Ground |
| RX        | Pin 3       | UART receive (SPS30 RX) |
| TX        | Pin 2       | UART transmit (SPS30 TX) |
| SEL       | GND         | Interface select (GND for UART) |

## Pin Configuration

The default pin configuration in the code is:
- `SPS30_RX_PIN = 2` (Arduino pin 2 connects to SPS30 TX)
- `SPS30_TX_PIN = 3` (Arduino pin 3 connects to SPS30 RX)

You can modify these pins in `src/sensirion_uart_implementation.c` if needed.

## Features

- **Automatic sensor initialization**: The code automatically detects and initializes the SPS30 sensor
- **Error handling**: Comprehensive error detection and recovery mechanisms
- **LED status indication**: 
  - Rapid blinking during initialization/errors
  - Slow blinking during normal operation
- **Complete measurement data**: Displays all available PM measurements including:
  - Mass concentrations (PM1.0, PM2.5, PM4.0, PM10.0)
  - Number concentrations for different particle sizes
  - Typical particle size

## Building and Uploading

1. Install PlatformIO in VS Code or use PlatformIO CLI
2. Open the project folder in PlatformIO
3. Build the project:
   ```bash
   pio run
   ```
4. Upload to Arduino:
   ```bash
   pio run --target upload
   ```
5. Monitor serial output:
   ```bash
   pio device monitor
   ```

## Serial Output

The sensor outputs measurement data every second in the following format:

```
------------------------------
PM 1.0: 12.34 μg/m³
PM 2.5: 15.67 μg/m³
PM 4.0: 18.90 μg/m³
PM 10.0: 21.23 μg/m³
Number concentration PM 0.5: 123.45 #/cm³
Number concentration PM 1.0: 98.76 #/cm³
Number concentration PM 2.5: 54.32 #/cm³
Number concentration PM 4.0: 21.09 #/cm³
Number concentration PM 10.0: 8.76 #/cm³
Typical particle size: 0.87 μm
------------------------------
```

## Troubleshooting

### Common Issues

1. **"SPS30 probe failed"**: 
   - Check wiring connections
   - Ensure SPS30 is powered correctly (5V)
   - Verify SEL pin is connected to GND

2. **"UART initialization failed"**:
   - Check if pins 2 and 3 are available
   - Modify pin definitions if needed

3. **"Communication error detected"**:
   - Check for loose connections
   - Ensure proper grounding
   - The system will automatically try to reinitialize

### LED Status Indicators

- **Rapid blinking (500ms)**: Sensor initialization or error state
- **Slow blinking (1000ms)**: Normal operation
- **Solid on/off**: Check connections

## Code Structure

- `src/main.cpp`: Main application logic
- `src/sensirion_uart_implementation.c`: UART communication implementation
- `src/sensirion_uart.h`: UART interface definitions
- `src/sensirion_arch_config.h`: Architecture-specific configurations
- `platformio.ini`: PlatformIO project configuration

## Dependencies

The project uses the following libraries:
- `sensirion/sensirion-sps@^1.2.0`: Official Sensirion SPS30 library
- `SoftwareSerial`: For UART communication
- `Wire`: I2C library (included for compatibility)

## License

This project is based on Sensirion's SPS30 library and follows the same BSD 3-Clause License.
