// imu.cpp — Two MPU-6050 on the same I²C bus (0x68 forearm, 0x69 hand).
// Burst read of 14 bytes from ACCEL_XOUT_H, converted to SI units.
// Configuration (J-1.2): DLPF_CFG = 2 (~98 Hz, ~3 ms delay), gyro ±500 °/s,
// accel ±4 g. Gyro bias is averaged over 2 s at start-up (keep still).

#include <Arduino.h>
#include <Wire.h>

#include "contracts.h"

namespace {

constexpr uint8_t REG_SMPLRT_DIV = 0x19;
constexpr uint8_t REG_CONFIG = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
constexpr uint8_t REG_WHO_AM_I = 0x75;

constexpr float G = 9.80665f;
constexpr float ACC_LSB_PER_G = 8192.0f;      // ±4 g
constexpr float GYR_LSB_PER_DPS = 65.5f;      // ±500 °/s
constexpr float DEG2RAD = 0.017453292519943295f;

float gyro_bias[2][3] = {};

bool write_reg(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool read_raw(uint8_t addr, int16_t out[7]) {
  Wire.beginTransmission(addr);
  Wire.write(REG_ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(addr, (uint8_t)14) != 14) return false;
  for (int i = 0; i < 7; ++i) {
    out[i] = (int16_t)((Wire.read() << 8) | Wire.read());
  }
  return true;  // out = ax, ay, az, temp, gx, gy, gz
}

bool read_one(uint8_t addr, int idx, ImuSample* s) {
  int16_t raw[7];
  s->t_us = micros();
  if (!read_raw(addr, raw)) return false;
  for (int k = 0; k < 3; ++k) {
    s->acc[k] = raw[k] / ACC_LSB_PER_G * G;
    s->gyr[k] = raw[4 + k] / GYR_LSB_PER_DPS * DEG2RAD - gyro_bias[idx][k];
  }
  return true;
}

bool configure(uint8_t addr) {
  return write_reg(addr, REG_PWR_MGMT_1, 0x01)      // wake up, PLL with gyro X clock
         && write_reg(addr, REG_CONFIG, 0x02)       // DLPF_CFG = 2
         && write_reg(addr, REG_SMPLRT_DIV, 0x00)   // 1 kHz internal rate
         && write_reg(addr, REG_GYRO_CONFIG, 0x08)  // ±500 °/s
         && write_reg(addr, REG_ACCEL_CONFIG, 0x08);  // ±4 g
}

}  // namespace

bool imu_init() {
  Wire.begin(pins::I2C_SDA, pins::I2C_SCL, pins::I2C_FREQ_HZ);
  delay(50);
  if (!configure(pins::IMU1_ADDR) || !configure(pins::IMU2_ADDR)) return false;
  delay(100);

  // Gyro bias: 2 s average, sensor must be still.
  const int n = 1000;
  double sum[2][3] = {};
  ImuSample s1, s2;
  for (int i = 0; i < n; ++i) {
    if (!read_one(pins::IMU1_ADDR, 0, &s1) || !read_one(pins::IMU2_ADDR, 1, &s2)) return false;
    for (int k = 0; k < 3; ++k) {
      sum[0][k] += s1.gyr[k];
      sum[1][k] += s2.gyr[k];
    }
    delayMicroseconds(LOOP_PERIOD_US);
  }
  for (int j = 0; j < 2; ++j)
    for (int k = 0; k < 3; ++k) gyro_bias[j][k] = (float)(sum[j][k] / n);
  return true;
}

bool imu_read(ImuSample* s1, ImuSample* s2) {
  return read_one(pins::IMU1_ADDR, 0, s1) && read_one(pins::IMU2_ADDR, 1, s2);
}
