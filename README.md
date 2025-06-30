# GY87 Sensor Library

A unified Arduino/C++ driver for the GY-87 IMU module, featuring:
- **MPU6050** (3-axis accelerometer + 3-axis gyroscope)
- **HMC5883L** (3-axis magnetometer / compass)
- **BMP180** (barometric pressure sensor)

## 📦 Features
- Calibrated readings (acceleration, gyro, compass, barometric pressure)
- Fused orientation output: pitch, roll, yaw
- Altitude estimation based on pressure
- EEPROM-based gyro calibration storage
- Easy-to-use API with `SensorData` structure

## 🧪 Example Usage

```cpp
#include "GY87.h"
GY87 imu;

void setup() {
  imu.begin();
  imu.setMagCalibration(4.59, 5.86, -35.41);
}

void loop() {
  imu.update();
  SensorData data = imu.getData();

  Serial.print("Alt (m): ");
  Serial.print(data.altitude, 2); Serial.print(" | ");

  Serial.print("Yaw/Pitch/Roll (°): ");
  Serial.print(data.yaw, 1); Serial.print(", ");
  Serial.print(data.pitch, 1); Serial.print(", ");
  Serial.println(data.roll, 1);
}
