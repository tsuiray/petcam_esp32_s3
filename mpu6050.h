#pragma once

#include <Arduino.h>
#include <Wire.h>

struct Mpu6050Sample {
  float accel_x; /* m/s^2 */
  float accel_y;
  float accel_z;
  float gyro_x;  /* rad/s */
  float gyro_y;
  float gyro_z;
  float temp_c;
};

bool mpu6050Begin(TwoWire &wire = Wire, uint8_t address = 0x68);
bool mpu6050Read(Mpu6050Sample &sample);
bool mpu6050IsReady();
