#include <Wire.h>
#include "GY87.h"

GY87 imu;

void setup() {
  Serial.begin(115200);
  while (!Serial); // USB bağlantısını bekle (bazı kartlarda gerekebilir)

  if (!imu.begin()) {
    Serial.println("GY87 başlatılamadı!");
    while (1);
  }

  imu.setMagCalibration(4.59, 5.86, -35.41); // Manuel ofset ayarı (gerekirse)
  Serial.println("GY87 başlatıldı!");
}

void loop() {
  imu.update();
  SensorData data = imu.getData();

  // Serial.print("Acc (g): ");
  // Serial.print(data.accX, 2); Serial.print(", ");
  // Serial.print(data.accY, 2); Serial.print(", ");
  // Serial.print(data.accZ, 2); Serial.print(" | ");

  // Serial.print("Gyro (°/s): ");
  // Serial.print(data.gyroX, 2); Serial.print(", ");
  // Serial.print(data.gyroY, 2); Serial.print(", ");
  // Serial.print(data.gyroZ, 2); Serial.print(" | ");

  // Serial.print("Mag (uT): ");
  // Serial.print(data.magX, 2); Serial.print(", ");
  // Serial.print(data.magY, 2); Serial.print(", ");
  // Serial.print(data.magZ, 2); Serial.print(" | ");

  Serial.print("Alt (m): ");
  Serial.print(data.altitude, 2); Serial.print(" | ");

  Serial.print("Yaw/Pitch/Roll (°): ");
  Serial.print(data.yaw, 1); Serial.print(", ");
  Serial.print(data.pitch, 1); Serial.print(", ");
  Serial.println(data.roll, 1);

  delay(50);
}