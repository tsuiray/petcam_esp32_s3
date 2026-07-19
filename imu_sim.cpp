#include "imu_sim.h"

#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace {

constexpr float kGravity = 9.80665f;
constexpr float kAccelLin = 0.60f; /* m/s^2 — visible on Orin create_map */
constexpr float kTurnRate = 0.80f; /* rad/s in-place yaw */
constexpr float kYawTol = 0.03f;   /* rad */

/*
 * L-shaped apartment footprint ≈ 490 sq ft (≈ 45.5 m²).
 *
 *   (0,7)----(4.5,7)
 *     |         |
 *     |         |(4.5,4)----(8,4)
 *     |                      |
 *   (0,0)------------------(8,0)
 *
 * Motion model for create_map: NO long cruise at constant velocity
 * (that yields a_x=a_y=0 and looks like "only gravity"). Each edge uses a
 * triangular speed profile so forward accel is non-zero almost the whole time.
 */
constexpr float kVertices[][2] = {
    {0.0f, 0.0f},
    {8.0f, 0.0f},
    {8.0f, 4.0f},
    {4.5f, 4.0f},
    {4.5f, 7.0f},
    {0.0f, 7.0f},
};
constexpr int kNumVertices = 6;

enum class Phase : uint8_t { Drive, Turn };

float s_dt = 0.02f;
float s_x = 0.0f;
float s_y = 0.0f;
float s_yaw = 0.0f;
float s_speed = 0.0f;
int s_edge = 0;
Phase s_phase = Phase::Drive;
float s_target_yaw = 0.0f;
float s_edge_length = 0.0f;
float s_traveled = 0.0f;
float s_ax = 0.0f;
float s_wz = 0.0f;
uint32_t s_tick = 0;

float wrapPi(float a) {
  while (a > static_cast<float>(M_PI)) {
    a -= 2.0f * static_cast<float>(M_PI);
  }
  while (a < -static_cast<float>(M_PI)) {
    a += 2.0f * static_cast<float>(M_PI);
  }
  return a;
}

float dist(float x0, float y0, float x1, float y1) {
  const float dx = x1 - x0;
  const float dy = y1 - y0;
  return sqrtf(dx * dx + dy * dy);
}

void loadEdge(int edge) {
  const int i0 = edge % kNumVertices;
  const int i1 = (edge + 1) % kNumVertices;
  s_edge_length = dist(kVertices[i0][0], kVertices[i0][1], kVertices[i1][0],
                       kVertices[i1][1]);
  s_traveled = 0.0f;
  s_target_yaw = atan2f(kVertices[i1][1] - kVertices[i0][1],
                        kVertices[i1][0] - kVertices[i0][0]);
  s_x = kVertices[i0][0];
  s_y = kVertices[i0][1];
  s_speed = 0.0f;
  s_ax = 0.0f;
  s_phase = Phase::Drive;
}

/**
 * Triangular speed along edge: accel first half of distance, decel second half.
 * Integrates a → v → s so we never stick at v=0 with a>0 unused.
 */
void updateDrive() {
  const float half = 0.5f * s_edge_length;
  if (half < 0.05f) {
    s_speed = 0.0f;
    s_ax = 0.0f;
    s_traveled = s_edge_length;
    return;
  }

  const float a_mag = kAccelLin;
  /* Cap speed so short edges still finish cleanly. */
  const float v_peak = sqrtf(a_mag * half);

  if (s_traveled < half) {
    s_ax = a_mag;
  } else {
    s_ax = -a_mag;
  }

  s_speed += s_ax * s_dt;
  if (s_speed < 0.0f) {
    s_speed = 0.0f;
  }
  if (s_speed > v_peak) {
    s_speed = v_peak;
  }

  s_x += cosf(s_yaw) * s_speed * s_dt;
  s_y += sinf(s_yaw) * s_speed * s_dt;
  s_traveled += s_speed * s_dt;

  if (s_traveled >= s_edge_length - 0.001f ||
      (s_traveled >= half && s_speed <= 0.001f)) {
    const int i1 = (s_edge + 1) % kNumVertices;
    s_x = kVertices[i1][0];
    s_y = kVertices[i1][1];
    s_traveled = s_edge_length;
    s_speed = 0.0f;
    s_ax = 0.0f;

    const int i2 = (s_edge + 2) % kNumVertices;
    s_target_yaw = atan2f(kVertices[i2][1] - kVertices[i1][1],
                          kVertices[i2][0] - kVertices[i1][0]);
    s_phase = Phase::Turn;
  }
}

void updateTurn() {
  s_ax = 0.0f;
  s_speed = 0.0f;
  const float err = wrapPi(s_target_yaw - s_yaw);
  if (fabsf(err) <= kYawTol) {
    s_yaw = s_target_yaw;
    s_wz = 0.0f;
    s_edge = (s_edge + 1) % kNumVertices;
    loadEdge(s_edge);
    s_yaw = s_target_yaw;
  } else {
    s_wz = (err > 0.0f) ? kTurnRate : -kTurnRate;
    s_yaw = wrapPi(s_yaw + s_wz * s_dt);
    if ((err > 0.0f && wrapPi(s_target_yaw - s_yaw) < 0.0f) ||
        (err < 0.0f && wrapPi(s_target_yaw - s_yaw) > 0.0f)) {
      s_yaw = s_target_yaw;
    }
  }
}

}  // namespace

float imuSimFootprintAreaM2() {
  float sum = 0.0f;
  for (int i = 0; i < kNumVertices; ++i) {
    const int j = (i + 1) % kNumVertices;
    sum += kVertices[i][0] * kVertices[j][1] - kVertices[j][0] * kVertices[i][1];
  }
  return fabsf(sum) * 0.5f;
}

float imuSimGetYaw() { return s_yaw; }

float imuSimGetSpeed() { return s_speed; }

int imuSimGetEdge() { return s_edge; }

void imuSimBegin(float dt_seconds) {
  s_dt = (dt_seconds > 0.001f) ? dt_seconds : 0.02f;
  s_edge = 0;
  s_yaw = 0.0f;
  s_tick = 0;
  s_wz = 0.0f;
  loadEdge(0);
  s_yaw = s_target_yaw;
  s_phase = Phase::Drive;
}

bool imuSimRead(Mpu6050Sample &sample) {
  s_wz = 0.0f;

  if (s_phase == Phase::Drive) {
    updateDrive();
  } else {
    updateTurn();
  }

  ++s_tick;

  /* Body frame: +X forward, +Z up. Gravity always on Z when level. */
  sample.accel_x = s_ax;
  sample.accel_y = 0.0f;
  sample.accel_z = kGravity;
  sample.gyro_x = 0.0f;
  sample.gyro_y = 0.0f;
  sample.gyro_z = s_wz;
  /* Fingerprint so Orin/Serial can confirm SIM (not a sitting-still REAL IMU). */
  sample.temp_c = 55.0f + 0.01f * static_cast<float>(s_edge);
  return true;
}
