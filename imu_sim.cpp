#include "imu_sim.h"

#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace {

constexpr float kGravity = 9.80665f;
constexpr float kCruiseSpeed = 0.30f;      /* m/s along edges */
constexpr float kAccelLin = 0.40f;         /* m/s^2 for start/stop bursts */
constexpr float kTurnRate = 0.70f;         /* rad/s in-place yaw */
constexpr float kPosTol = 0.02f;           /* m */
constexpr float kYawTol = 0.03f;           /* rad */

/*
 * L-shaped apartment footprint ≈ 490 sq ft (≈ 45.5 m²).
 *
 * Layout (meters), CCW outer walls — living + bedroom wing:
 *
 *   (0,7)----(4.5,7)
 *     |         |
 *     |         |(4.5,4)----(8,4)
 *     |                      |
 *   (0,0)------------------(8,0)
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

enum class Phase : uint8_t {
  Accelerate,
  Cruise,
  Decelerate,
  Turn,
};

float s_dt = 0.02f;
float s_x = 0.0f;
float s_y = 0.0f;
float s_yaw = 0.0f;
float s_speed = 0.0f;
int s_edge = 0;
Phase s_phase = Phase::Accelerate;
float s_target_yaw = 0.0f;
float s_edge_length = 0.0f;
float s_traveled = 0.0f;

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
  const float x0 = kVertices[i0][0];
  const float y0 = kVertices[i0][1];
  const float x1 = kVertices[i1][0];
  const float y1 = kVertices[i1][1];
  s_edge_length = dist(x0, y0, x1, y1);
  s_traveled = 0.0f;
  s_target_yaw = atan2f(y1 - y0, x1 - x0);
  s_x = x0;
  s_y = y0;
  s_speed = 0.0f;
  s_phase = Phase::Accelerate;
}

}  // namespace

float imuSimFootprintAreaM2() {
  /* shoelace on the L polygon */
  float sum = 0.0f;
  for (int i = 0; i < kNumVertices; ++i) {
    const int j = (i + 1) % kNumVertices;
    sum += kVertices[i][0] * kVertices[j][1] - kVertices[j][0] * kVertices[i][1];
  }
  return fabsf(sum) * 0.5f;
}

void imuSimBegin(float dt_seconds) {
  s_dt = (dt_seconds > 0.001f) ? dt_seconds : 0.02f;
  s_edge = 0;
  s_yaw = 0.0f;
  loadEdge(0);
  /* Face first edge before moving. */
  s_yaw = s_target_yaw;
  s_phase = Phase::Accelerate;
}

bool imuSimRead(Mpu6050Sample &sample) {
  float ax_body = 0.0f;
  float ay_body = 0.0f;
  float wz = 0.0f;

  switch (s_phase) {
    case Phase::Accelerate: {
      s_speed += kAccelLin * s_dt;
      if (s_speed >= kCruiseSpeed) {
        s_speed = kCruiseSpeed;
        s_phase = Phase::Cruise;
        ax_body = 0.0f;
      } else {
        ax_body = kAccelLin;
      }
      break;
    }
    case Phase::Cruise: {
      const float remaining = s_edge_length - s_traveled;
      const float stop_dist = (kCruiseSpeed * kCruiseSpeed) / (2.0f * kAccelLin);
      if (remaining <= stop_dist) {
        s_phase = Phase::Decelerate;
      }
      ax_body = 0.0f;
      break;
    }
    case Phase::Decelerate: {
      s_speed -= kAccelLin * s_dt;
      ax_body = -kAccelLin;
      if (s_speed <= 0.0f || (s_edge_length - s_traveled) <= kPosTol) {
        s_speed = 0.0f;
        ax_body = 0.0f;
        /* Snap to vertex and prepare turn toward next edge. */
        const int i1 = (s_edge + 1) % kNumVertices;
        s_x = kVertices[i1][0];
        s_y = kVertices[i1][1];
        s_traveled = s_edge_length;
        const int i2 = (s_edge + 2) % kNumVertices;
        s_target_yaw =
            atan2f(kVertices[i2][1] - kVertices[i1][1],
                   kVertices[i2][0] - kVertices[i1][0]);
        s_phase = Phase::Turn;
      }
      break;
    }
    case Phase::Turn: {
      const float err = wrapPi(s_target_yaw - s_yaw);
      if (fabsf(err) <= kYawTol) {
        s_yaw = s_target_yaw;
        wz = 0.0f;
        s_edge = (s_edge + 1) % kNumVertices;
        loadEdge(s_edge);
        s_yaw = s_target_yaw;
      } else {
        const float step = (err > 0.0f) ? kTurnRate : -kTurnRate;
        wz = step;
        s_yaw = wrapPi(s_yaw + step * s_dt);
        /* Avoid overshoot */
        if ((err > 0.0f && wrapPi(s_target_yaw - s_yaw) < 0.0f) ||
            (err < 0.0f && wrapPi(s_target_yaw - s_yaw) > 0.0f)) {
          s_yaw = s_target_yaw;
        }
      }
      break;
    }
  }

  if (s_phase == Phase::Accelerate || s_phase == Phase::Cruise ||
      s_phase == Phase::Decelerate) {
    const float dx = cosf(s_yaw) * s_speed * s_dt;
    const float dy = sinf(s_yaw) * s_speed * s_dt;
    s_x += dx;
    s_y += dy;
    s_traveled += s_speed * s_dt;
    if (s_traveled > s_edge_length) {
      s_traveled = s_edge_length;
    }
  }

  /* Body-frame IMU: gravity on +Z when level; forward accel on +X. */
  sample.accel_x = ax_body;
  sample.accel_y = ay_body;
  sample.accel_z = kGravity;
  sample.gyro_x = 0.0f;
  sample.gyro_y = 0.0f;
  sample.gyro_z = wz;
  sample.temp_c = 25.0f;
  return true;
}
