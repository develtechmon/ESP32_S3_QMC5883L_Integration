# ESP32_S3_QMC5883L_Integration

## Pin and Schematic Connection
Below is schematic and pin configuration to use this code

| ESP32-S3 Super Mini | QMC5883L |
|---------------------|----------|
| VDD (5V)            | 5V       |
| GND                 | GND      |
| SCL                 | 5        |
| SDA                 | 4        |

## Sensor Used
Below is the orientation for this code
<img width="774" height="846" alt="image" src="https://github.com/user-attachments/assets/ed52395b-f0c3-4171-887b-a0b5d1960722" />

## Calibration
To use this sensor, you have to calibrate the sensor properly to determine proper `NESD` with
respect to geographical orientation.

Please run the calibration code and copy and paste final calibration value `mag_cal_values[6]` below to your code. In my case,
i use it in my flight controller position hold v4 code.

<img width="803" height="487" alt="image" src="https://github.com/user-attachments/assets/1740c92c-79db-4ac6-b75d-c98878a73e28" />

