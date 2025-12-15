# brz_display
Subaru BRZ/GT86 aux display

ESP32, TFT 1.9 ips 320x170, ADS1X15, SN65HVD230, pressure sensor 150PSI

<img alt="PXL_20230612_061920956.jpg" src="https://github.com/dodox1/brz_display/blob/main/PXL_20230612_061920956.jpg?raw=true" data-hpc="true" class="Box-sc-g0xbh4-0 kzRgrI">



# BRZ Display - Pressure and CANBus Receiver

This project is designed for an ESP32-based system that displays oil pressure, temperature, and RPM readings on a TFT screen. It also integrates data from a CAN bus and an ADS1115 ADC sensor.

## Features

- **CAN Bus Communication**: Receives and processes data from a CAN bus, specifically for oil temperature, coolant temperature, and RPM values.
- **Pressure Sensor Integration**: Reads and calculates oil pressure values using an ADC and pressure sensor.
- **TFT Display**: Displays real-time data such as oil pressure, oil temperature, coolant temperature, and RPM on a graphical interface.
- **Graphical Visualization**: Includes a graph for visualizing pressure trends over time.
- **Median Filtering**: Applies a median filter to smooth out sensor noise.

---

## Hardware Requirements

- **Microcontroller**: ESP32 Mini D1
- **Pressure Sensor**: 150 PSI pressure sensor
  - Voltage output:
    - 0 PSI: 0.5V
    - 75 PSI: 2.5V
    - 150 PSI: 4.5V
- **Analog-to-Digital Converter (ADC)**: ADS1115
- **CAN bus transceiver**: SN65HVD230
- **Display**: TFT screen compatible with `TFT_eSPI` library.

---

## Software Dependencies

- **Arduino Libraries**:
  - `Arduino.h`
  - `ESP32CAN`
  - `CAN_config`
  - `ADS1X15`
  - `MedianFilterLib2`
  - `TFT_eSPI`

Ensure these libraries are installed before uploading the code.

---

## Wiring

| Component           | ESP32 Pin  | Notes                          |
|---------------------|------------|--------------------------------|
| CAN Transceiver RX  | GPIO 13    | Connect to CAN transceiver SN65HVD230 |
| CAN Transceiver TX  | GPIO 15    |                                |
| ADS1115 SDA         | GPIO 16    | I2C data line                 |
| ADS1115 SCL         | GPIO 17    | I2C clock line                |
| TFT Display         | SPI Pins   | Ensure proper SPI connections |

---

## Pressure Calculation

The pressure sensor outputs voltages corresponding to the measured pressure. The calculation is done using the formula:
```c
pressure = (voltage - 0.461) / 4 * 1000; // in kPa
```
Voltage is read through an ADS1115 ADC with a voltage divider to match the sensor's range.

---

## CAN Bus Configuration

The CAN bus is configured to operate at 500Kbps with the following filters:
- **Message ID 0x360**: Oil and coolant temperatures
- **Message ID 0x140**: RPM values

---

## Usage

1. **Setup**:
   - Connect all hardware as per the wiring diagram.
   - Install the required libraries.
   - Upload the `brz_display.ino` file to the ESP32.

2. **Operation**:
   - Upon startup, the device displays the Subaru logo.
   - Real-time data for oil pressure, oil temperature, coolant temperature, and RPM will be displayed.
   - The graph will update periodically to visualize pressure trends.

---

## Key Variables

- `rawPressure`: Raw pressure value from the sensor.
- `pressure`: Filtered pressure value, displayed on the screen.
- `voltage`: Voltage read from the sensor after applying the voltage divider and calibration constant.
- `oilTemperature`: Oil temperature in Celsius.
- `coolantTemperature`: Coolant temperature in Celsius.
- `rpm`: Engine RPM.

---

## Troubleshooting

- **No Display Output**:
  - Check TFT connections and initialization code.
- **Incorrect Pressure Reading**:
  - Verify the calibration constants for the voltage divider.
  - Check the wiring of the pressure sensor.
- **No CAN Data**:
  - Ensure proper CAN bus wiring and transceiver functionality.
  - Verify that the CAN bus speed matches the source.

---

## Future Improvements

- Add error handling for ADC and CAN bus initialization.
- Optimize graph update logic.
- Allow dynamic configuration of calibration constants.
- Implement a debug flag to toggle serial output.
- Optimize power consumption for battery-operated setups.

---

## License

This project is open-source and available under the MIT License.

## Author

Created by [dodox1](https://github.com/dodox1).
