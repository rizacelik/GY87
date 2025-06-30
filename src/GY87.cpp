#include "GY87.h"

// ------------------------ Public ------------------------

bool GY87::begin(bool autoCalibrate) {
  Wire.begin();
  delay(100);

  initMPU6050();
  initHMC5883L();
  initBMP180();

  if (autoCalibrate) {
    calibrateGyro();
    EEPROM.put(EEPROM_GYRO_OFFSET, gx_offset);
    EEPROM.put(EEPROM_GYRO_OFFSET + sizeof(float), gy_offset);
    EEPROM.put(EEPROM_GYRO_OFFSET + 2 * sizeof(float), gz_offset);
  } else {
    EEPROM.get(EEPROM_GYRO_OFFSET, gx_offset);
    EEPROM.get(EEPROM_GYRO_OFFSET + sizeof(float), gy_offset);
    EEPROM.get(EEPROM_GYRO_OFFSET + 2 * sizeof(float), gz_offset);
  }

lastUpdate = millis();
  return true;
}

void GY87::update() {
  readMPU6050();
  readHMC5883L();
  readBMP180();
  computeOrientation();
}

SensorData GY87::getData() {
  return data;
}

void GY87::setMagCalibration(float x, float y, float z) {
  magOffsetX = x;
  magOffsetY = y;
  magOffsetZ = z;
}

// ------------------------ MPU6050 ------------------------

void GY87::initMPU6050() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  if (Wire.endTransmission() != 0) {
    Serial.println("MPU6050 başlatma hatası!");
    return;
  }

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1C); Wire.write(0x00); Wire.endTransmission(); // ±2g
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1B); Wire.write(0x08); Wire.endTransmission(); // ±500°/s
}

void GY87::readMPURegisters(uint8_t* buffer) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) {
    Serial.println("MPU6050 veri okuma hatası!");
    return;
  }
  Wire.requestFrom(MPU6050_ADDR, 14);
  for (int i = 0; i < 14; i++) {
    buffer[i] = Wire.read();
  }
}

void GY87::readRawMPU6050() {
  uint8_t raw[14];
  readMPURegisters(raw);

  int16_t gx = (raw[8] << 8) | raw[9];
  int16_t gy = (raw[10] << 8) | raw[11];
  int16_t gz = (raw[12] << 8) | raw[13];

  gx_offset = gx;
  gy_offset = gy;
  gz_offset = gz;
}

void GY87::readMPU6050() {
  uint8_t buffer[14];
  readMPURegisters(buffer);

  int16_t axRaw = (buffer[0] << 8) | buffer[1];
  int16_t ayRaw = (buffer[2] << 8) | buffer[3];
  int16_t azRaw = (buffer[4] << 8) | buffer[5];
  int16_t temp  = (buffer[6] << 8) | buffer[7];
  int16_t gxRaw = (buffer[8] << 8) | buffer[9];
  int16_t gyRaw = (buffer[10] << 8) | buffer[11];
  int16_t gzRaw = (buffer[12] << 8) | buffer[13];

  data.accX = (axRaw - ax_offset) / 16384.0;
  data.accY = (ayRaw - ay_offset) / 16384.0;
  data.accZ = (azRaw - az_offset) / 16384.0;
  data.gyroX = (gxRaw - gx_offset) / 65.5;
  data.gyroY = (gyRaw - gy_offset) / 65.5;
  data.gyroZ = (gzRaw - gz_offset) / 65.5;
  data.temperature = (temp / 340.0) + 36.53;
}

void GY87::calibrateGyro() {
  const int samples = 200;
  long sumX = 0, sumY = 0, sumZ = 0;
  long sum_ax = 0, sum_ay = 0, sum_az = 0;

  Serial.println("Gyro kalibrasyonu başlıyor...");
  for (int i = 0; i < samples; i++) {
    uint8_t buffer[14];
    readMPURegisters(buffer);
    int16_t axRaw = (buffer[0] << 8) | buffer[1];
    int16_t ayRaw = (buffer[2] << 8) | buffer[3];
    int16_t azRaw = (buffer[4] << 8) | buffer[5];
    sum_ax += axRaw;
    sum_ay += ayRaw;
    sum_az += azRaw;
    sumX += (int16_t)(buffer[8] << 8 | buffer[9]);
    sumY += (int16_t)(buffer[10] << 8 | buffer[11]);
    sumZ += (int16_t)(buffer[12] << 8 | buffer[13]);
    delay(5);
  }

  ax_offset = sum_ax / (float)samples;
  ay_offset = sum_ay / (float)samples;
  az_offset = (sum_az / (float)samples) - 16384.0;  // Yerçekimi düzeltmesi

  gx_offset = sumX / (float)samples;
  gy_offset = sumY / (float)samples;
  gz_offset = sumZ / (float)samples;

  Serial.println("Gyro ofsetler kaydedildi.");
}

// ------------------------ HMC5883L ------------------------

void GY87::initHMC5883L() {
  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(0x00); Wire.write(0x70); // 8 sample avg, 15 Hz
  Wire.write(0x01); Wire.write(0x20); // Gain
  Wire.write(0x02); Wire.write(0x00); // Continuous mode
  if (Wire.endTransmission() != 0) {
    Serial.println("HMC5883L başlatma hatası!");
  }
}

void GY87::readHMC5883L() {
  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(0x03);
  Wire.endTransmission();
  Wire.requestFrom(HMC5883L_ADDR, 6);

  int16_t mx = Wire.read() << 8 | Wire.read();
  int16_t mz = Wire.read() << 8 | Wire.read();
  int16_t my = Wire.read() << 8 | Wire.read(); // sıraya dikkat

  data.magX = (mx - magOffsetX) * 0.92;
  data.magY = (my - magOffsetY) * 0.92;
  data.magZ = (mz - magOffsetZ) * 0.92;
}

// ------------------------ BMP180 ------------------------

void GY87::initBMP180() {
  ac1 = bmpReadInt(0xAA);
  ac2 = bmpReadInt(0xAC);
  ac3 = bmpReadInt(0xAE);
  ac4 = bmpReadInt(0xB0);
  ac5 = bmpReadInt(0xB2);
  ac6 = bmpReadInt(0xB4);
  b1 = bmpReadInt(0xB6);
  b2 = bmpReadInt(0xB8);
  mb = bmpReadInt(0xBA);
  mc = bmpReadInt(0xBC);
  md = bmpReadInt(0xBE);
}

void GY87::readBMP180() {
  data.temperature = bmpGetTemperature(bmpReadUT());
  data.pressure = bmpGetPressure(bmpReadUP());
  data.altitude = calcAltitude(data.pressure);
}

float GY87::calcAltitude(float pressure) {
  return (1.0 - pow(pressure / 101325.0, 0.1903)) / 0.0000225577;
}

float GY87::bmpGetTemperature(unsigned int ut) {
  long x1 = ((long)ut - (long)ac6) * ac5 >> 15;
  long x2 = ((long)mc << 11) / (x1 + md);
  b5 = x1 + x2;
  return ((b5 + 8) >> 4) / 10.0;
}

long GY87::bmpGetPressure(unsigned long up) {
  long x1, x2, x3, b3, b6, p;
  unsigned long b4, b7;
  b6 = b5 - 4000;
  x1 = (b2 * ((b6 * b6) >> 12)) >> 11;
  x2 = (ac2 * b6) >> 11;
  x3 = x1 + x2;
  b3 = (((((long)ac1) * 4 + x3) << 0) + 2) >> 2;

  x1 = (ac3 * b6) >> 13;
  x2 = (b1 * ((b6 * b6) >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
  b4 = (ac4 * (unsigned long)(x3 + 32768)) >> 15;
  b7 = ((unsigned long)(up - b3) * (50000 >> 0));

  if (b7 < 0x80000000)
    p = (b7 << 1) / b4;
  else
    p = (b7 / b4) << 1;

  x1 = (p >> 8) * (p >> 8);
  x1 = (x1 * 3038) >> 16;
  x2 = (-7357 * p) >> 16;
  p += (x1 + x2 + 3791) >> 4;

  return p;
}

char GY87::bmpRead(uint8_t reg) {
  Wire.beginTransmission(BMP180_ADDR);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(BMP180_ADDR, 1);
  while (!Wire.available());
  return Wire.read();
}

int GY87::bmpReadInt(uint8_t reg) {
  Wire.beginTransmission(BMP180_ADDR);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(BMP180_ADDR, 2);
  while (Wire.available() < 2);
  uint8_t msb = Wire.read();
  uint8_t lsb = Wire.read();
  return (msb << 8) | lsb;
}

unsigned int GY87::bmpReadUT() {
  Wire.beginTransmission(BMP180_ADDR);
  Wire.write(0xF4);
  Wire.write(0x2E);
  Wire.endTransmission();
  delay(5);
  return bmpReadInt(0xF6);
}

unsigned long GY87::bmpReadUP() {
  Wire.beginTransmission(BMP180_ADDR);
  Wire.write(0xF4);
  Wire.write(0x34);
  Wire.endTransmission();
  delay(8);
  uint8_t msb = bmpRead(0xF6);
  uint8_t lsb = bmpRead(0xF7);
  uint8_t xlsb = bmpRead(0xF8);
  return (((unsigned long)msb << 16) | ((unsigned long)lsb << 8) | xlsb) >> 8;
}

// void GY87::computeOrientation() {
//   data.pitch = atan2(-data.accX, sqrt(data.accY * data.accY + data.accZ * data.accZ));
//   data.roll  = atan2(data.accY, sqrt(data.accX * data.accX + data.accZ * data.accZ));

//   float magX = data.magX * cos(data.pitch) + data.magZ * sin(data.pitch);
//   float magY = data.magX * sin(data.roll) * sin(data.pitch) + data.magY * cos(data.roll) - data.magZ * sin(data.roll) * cos(data.pitch);
//   data.yaw = atan2(magY, magX);

//   data.pitch *= RAD_TO_DEG;
//   data.roll  *= RAD_TO_DEG;
//   data.yaw   *= RAD_TO_DEG;
//   if (data.yaw < 0) data.yaw += 360.0;
// }


void GY87::computeOrientation() {
  // Zaman farkı (saniye cinsinden)
  unsigned long now = millis();
  float dt = (now - lastUpdate) / 1000.0;
  lastUpdate = now;

  // İvme tabanlı açı tahmini
  float acc_pitch = atan2(-data.accX, sqrt(data.accY * data.accY + data.accZ * data.accZ)) * RAD_TO_DEG;
  float acc_roll  = atan2(data.accY, sqrt(data.accX * data.accX + data.accZ * data.accZ)) * RAD_TO_DEG;

  // Gyro entegre
  comp_pitch = comp_alpha * (comp_pitch + data.gyroY * dt) + (1.0 - comp_alpha) * acc_pitch;
  comp_roll  = comp_alpha * (comp_roll + data.gyroX * dt) + (1.0 - comp_alpha) * acc_roll;

  // Yaw için hâlâ pusula kullanılacak
  float pitch_rad = comp_pitch * DEG_TO_RAD;
  float roll_rad  = comp_roll * DEG_TO_RAD;

  float magX = data.magX * cos(pitch_rad) + data.magZ * sin(pitch_rad);
  float magY = data.magX * sin(roll_rad) * sin(pitch_rad) + data.magY * cos(roll_rad) - data.magZ * sin(roll_rad) * cos(pitch_rad);
  float yaw = atan2(magY, magX) * RAD_TO_DEG;
  if (yaw < 0) yaw += 360.0;

  // Sonuçları ata
  data.pitch = comp_pitch;
  data.roll  = comp_roll;
  data.yaw   = yaw;
}