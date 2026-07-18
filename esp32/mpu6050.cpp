#include "mpu6050.h"
#include "board_config.h"

#include <math.h>

namespace {

constexpr uint8_t kRegSmplrtDiv = 0x19;
constexpr uint8_t kRegConfig = 0x1A;
constexpr uint8_t kRegGyroConfig = 0x1B;
constexpr uint8_t kRegAccelConfig = 0x1C;
constexpr uint8_t kRegAccelXoutH = 0x3B;
constexpr uint8_t kRegPwrMgmt1 = 0x6B;
constexpr uint8_t kRegWhoAmI = 0x75;

constexpr float kAccelLsbPerG = 16384.0f;
constexpr float kGyroLsbPerDps = 131.0f;
constexpr float kGravity = 9.80665f;
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
constexpr float kDegToRad = static_cast<float>(M_PI / 180.0);

TwoWire *s_wire = nullptr;
uint8_t s_addr = 0x68;
bool s_ready = false;

bool writeReg(uint8_t reg, uint8_t value) {
  s_wire->beginTransmission(s_addr);
  s_wire->write(reg);
  s_wire->write(value);
  return s_wire->endTransmission() == 0;
}

bool readBytes(uint8_t reg, uint8_t *data, size_t len) {
  s_wire->beginTransmission(s_addr);
  s_wire->write(reg);
  if (s_wire->endTransmission(false) != 0) {
    return false;
  }
  const size_t got = s_wire->requestFrom(static_cast<int>(s_addr), static_cast<int>(len));
  if (got != len) {
    return false;
  }
  for (size_t i = 0; i < len; ++i) {
    data[i] = static_cast<uint8_t>(s_wire->read());
  }
  return true;
}

}  // namespace

bool mpu6050Begin(TwoWire &wire, uint8_t address) {
  s_wire = &wire;
  s_addr = address;
  s_ready = false;

  uint8_t whoami = 0;
  if (!readBytes(kRegWhoAmI, &whoami, 1)) {
    return false;
  }
  if (whoami != 0x68) {
    // Some clones still work; continue after warning via Serial.
    Serial.printf("MPU6050 unexpected WHO_AM_I=0x%02X\n", whoami);
  }

  if (!writeReg(kRegPwrMgmt1, 0x01)) {
    return false;
  }
  delay(100);

  writeReg(kRegConfig, 0x03);
  writeReg(kRegSmplrtDiv, 9);
  writeReg(kRegGyroConfig, 0x00);   // ±250 dps
  writeReg(kRegAccelConfig, 0x00);  // ±2g

  s_ready = true;
  return true;
}

bool mpu6050IsReady() { return s_ready; }

bool mpu6050Read(Mpu6050Sample &sample) {
  if (!s_ready || s_wire == nullptr) {
    return false;
  }

  uint8_t raw[14];
  if (!readBytes(kRegAccelXoutH, raw, sizeof(raw))) {
    return false;
  }

  const int16_t ax = static_cast<int16_t>((raw[0] << 8) | raw[1]);
  const int16_t ay = static_cast<int16_t>((raw[2] << 8) | raw[3]);
  const int16_t az = static_cast<int16_t>((raw[4] << 8) | raw[5]);
  const int16_t temp = static_cast<int16_t>((raw[6] << 8) | raw[7]);
  const int16_t gx = static_cast<int16_t>((raw[8] << 8) | raw[9]);
  const int16_t gy = static_cast<int16_t>((raw[10] << 8) | raw[11]);
  const int16_t gz = static_cast<int16_t>((raw[12] << 8) | raw[13]);

  sample.accel_x = (static_cast<float>(ax) / kAccelLsbPerG) * kGravity;
  sample.accel_y = (static_cast<float>(ay) / kAccelLsbPerG) * kGravity;
  sample.accel_z = (static_cast<float>(az) / kAccelLsbPerG) * kGravity;
  sample.gyro_x = (static_cast<float>(gx) / kGyroLsbPerDps) * kDegToRad;
  sample.gyro_y = (static_cast<float>(gy) / kGyroLsbPerDps) * kDegToRad;
  sample.gyro_z = (static_cast<float>(gz) / kGyroLsbPerDps) * kDegToRad;
  sample.temp_c = (static_cast<float>(temp) / 340.0f) + 36.53f;
  return true;
}
