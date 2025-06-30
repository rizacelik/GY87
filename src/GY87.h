#ifndef GY87_H
#define GY87_H

#include "Arduino.h"
#include <Wire.h>
#include <math.h>
#include <EEPROM.h>

// I2C adresleri
#define MPU6050_ADDR   0x68
#define HMC5883L_ADDR  0x1E
#define BMP180_ADDR    0x77

// EEPROM adresleri
#define EEPROM_GYRO_OFFSET 0

struct SensorData {
  float accX, accY, accZ;
  float gyroX, gyroY, gyroZ;
  float magX, magY, magZ;
  float temperature;
  float pressure;
  float altitude;
  float pitch, roll, yaw;
};

class GY87 {
public:
  bool begin(bool autoCalibrate = true);
  void update();
  SensorData getData();

  void setMagCalibration(float x, float y, float z);

private:
  SensorData data;

  // Ofset değerleri
  float ax_offset = 0, ay_offset = 0, az_offset = 0;
  float gx_offset = 0, gy_offset = 0, gz_offset = 0;

  float magOffsetX = 0, magOffsetY = 0, magOffsetZ = 0;

  float comp_pitch = 0.0, comp_roll = 0.0;
  unsigned long lastUpdate = 0;
  const float comp_alpha = 0.98; // %98 gyro, %2 ivme katkısı

  // MPU6050
  void initMPU6050();
  void readMPU6050();
  void readRawMPU6050();
  void readMPURegisters(uint8_t* buffer);
  void calibrateGyro();

  // HMC5883L
  void initHMC5883L();
  void readHMC5883L();

  // BMP180
  void initBMP180();
  void readBMP180();
  float calcAltitude(float pressure);
  int ac1, ac2, ac3, b1, b2, mb, mc, md;
  uint16_t ac4, ac5, ac6;
  long b5;

  float bmpGetTemperature(unsigned int ut);
  long  bmpGetPressure(unsigned long up);
  char  bmpRead(uint8_t reg);
  int   bmpReadInt(uint8_t reg);
  unsigned int bmpReadUT();
  unsigned long bmpReadUP();

  // Genel işlemler
  void computeOrientation();
};

#endif
