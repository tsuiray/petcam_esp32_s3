#include "imu_sim.h"

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace {

/*
 * L-shaped ~490 sq ft (45.5 m²) apartment — same vertices every lap.
 *
 *   (0,7)----(4.5,7)
 *     |         |
 *     |         |(4.5,4)----(8,4)
 *     |                      |
 *   (0,0)------------------(8,0)
 */
constexpr float kVertices[][2] = {
    {0.0f, 0.0f}, {8.0f, 0.0f}, {8.0f, 4.0f},
    {4.5f, 4.0f}, {4.5f, 7.0f}, {0.0f, 7.0f},
};
constexpr int kNumVertices = 6;

/* Max samples for one full lap (edges + turns) at ~50 Hz. */
constexpr int kMaxLapSamples = 8000;

struct LapSample {
  float ax;  /* world-frame m/s^2 */
  float ay;
  float gz;  /* rad/s */
  float yaw; /* rad, attitude at this sample */
  float speed;
  uint8_t edge;
};

LapSample s_lap[kMaxLapSamples];
int s_lap_len = 0;
int s_idx = 0;
uint32_t s_lap_count = 0;
float s_dt = 0.02f;

float wrapPi(float a) {
  while (a > static_cast<float>(M_PI)) {
    a -= 2.0f * static_cast<float>(M_PI);
  }
  while (a < -static_cast<float>(M_PI)) {
    a += 2.0f * static_cast<float>(M_PI);
  }
  return a;
}

float dist2(float x0, float y0, float x1, float y1) {
  const float dx = x1 - x0;
  const float dy = y1 - y0;
  return sqrtf(dx * dx + dy * dy);
}

bool pushSample(float ax, float ay, float gz, float yaw, float speed,
                uint8_t edge) {
  if (s_lap_len >= kMaxLapSamples) {
    return false;
  }
  s_lap[s_lap_len].ax = ax;
  s_lap[s_lap_len].ay = ay;
  s_lap[s_lap_len].gz = gz;
  s_lap[s_lap_len].yaw = yaw;
  s_lap[s_lap_len].speed = speed;
  s_lap[s_lap_len].edge = edge;
  ++s_lap_len;
  return true;
}

/**
 * Append Euler-consistent accel/decel along a straight edge.
 * With constant +a for N steps then -a for N steps:
 *   distance = N^2 * a * dt^2  →  a = L / (N^2 * dt^2)
 *   final velocity = 0
 * so naive integrators that use the same Euler rule close cleanly.
 */
void appendEdge(int edge_idx, float x0, float y0, float x1, float y1,
                float yaw) {
  const float L = dist2(x0, y0, x1, y1);
  if (L < 1e-3f) {
    return;
  }
  const float ux = (x1 - x0) / L;
  const float uy = (y1 - y0) / L;

  /* Choose N so peak speed stays moderate (~0.4–0.8 m/s). */
  int N = static_cast<int>(L / (0.5f * s_dt)); /* rough */
  if (N < 20) {
    N = 20;
  }
  if (N > 400) {
    N = 400;
  }

  const float a_mag = L / (static_cast<float>(N * N) * s_dt * s_dt);
  float v = 0.0f;

  for (int i = 0; i < N; ++i) {
    const float ax = a_mag * ux;
    const float ay = a_mag * uy;
    v += a_mag * s_dt;
    if (!pushSample(ax, ay, 0.0f, yaw, v, static_cast<uint8_t>(edge_idx))) {
      return;
    }
  }
  for (int i = 0; i < N; ++i) {
    const float ax = -a_mag * ux;
    const float ay = -a_mag * uy;
    v -= a_mag * s_dt;
    if (v < 0.0f) {
      v = 0.0f;
    }
    if (!pushSample(ax, ay, 0.0f, yaw, v, static_cast<uint8_t>(edge_idx))) {
      return;
    }
  }
  /* Force exact zero velocity marker samples (a=0) so integrators settle. */
  for (int i = 0; i < 3; ++i) {
    pushSample(0.0f, 0.0f, 0.0f, yaw, 0.0f, static_cast<uint8_t>(edge_idx));
  }
}

void appendTurn(float yaw0, float yaw1, uint8_t edge_idx) {
  float dyaw = wrapPi(yaw1 - yaw0);
  if (fabsf(dyaw) < 1e-4f) {
    return;
  }

  /* ~90 deg/s turn rate target. */
  const float turn_rate = 0.9f; /* rad/s magnitude */
  const float T = fabsf(dyaw) / turn_rate;
  int N = static_cast<int>(T / s_dt + 0.5f);
  if (N < 10) {
    N = 10;
  }
  if (N > 300) {
    N = 300;
  }

  const float gz = dyaw / (static_cast<float>(N) * s_dt);
  for (int i = 1; i <= N; ++i) {
    const float yaw = wrapPi(yaw0 + gz * static_cast<float>(i) * s_dt);
    if (!pushSample(0.0f, 0.0f, gz, yaw, 0.0f, edge_idx)) {
      return;
    }
  }
  /* Settle after turn. */
  for (int i = 0; i < 3; ++i) {
    pushSample(0.0f, 0.0f, 0.0f, yaw1, 0.0f, edge_idx);
  }
}

void buildLapBuffer() {
  s_lap_len = 0;

  float yaw = atan2f(kVertices[1][1] - kVertices[0][1],
                     kVertices[1][0] - kVertices[0][0]);

  for (int e = 0; e < kNumVertices; ++e) {
    const int i0 = e;
    const int i1 = (e + 1) % kNumVertices;
    const int i2 = (e + 2) % kNumVertices;

    const float x0 = kVertices[i0][0];
    const float y0 = kVertices[i0][1];
    const float x1 = kVertices[i1][0];
    const float y1 = kVertices[i1][1];

    yaw = atan2f(y1 - y0, x1 - x0);
    appendEdge(e, x0, y0, x1, y1, yaw);

    const float yaw_next = atan2f(kVertices[i2][1] - y1, kVertices[i2][0] - x1);
    appendTurn(yaw, yaw_next, static_cast<uint8_t>(e));
    yaw = yaw_next;
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

float imuSimGetYaw() {
  if (s_lap_len <= 0) {
    return 0.0f;
  }
  return s_lap[s_idx].yaw;
}

float imuSimGetSpeed() {
  if (s_lap_len <= 0) {
    return 0.0f;
  }
  return s_lap[s_idx].speed;
}

int imuSimGetEdge() {
  if (s_lap_len <= 0) {
    return 0;
  }
  return s_lap[s_idx].edge;
}

uint32_t imuSimGetLap() { return s_lap_count; }

uint32_t imuSimGetSampleIndex() { return static_cast<uint32_t>(s_idx); }

void imuSimBegin(float dt_seconds) {
  s_dt = (dt_seconds > 0.001f) ? dt_seconds : 0.02f;
  s_idx = 0;
  s_lap_count = 0;
  buildLapBuffer();
}

bool imuSimRead(Mpu6050Sample &sample) {
  if (s_lap_len <= 0) {
    memset(&sample, 0, sizeof(sample));
    sample.accel_z = 0.0f;
    sample.temp_c = 55.0f;
    return false;
  }

  const LapSample &s = s_lap[s_idx];

  /* World-frame accel for create_map (NOT body-frame, NOT gravity). */
  sample.accel_x = s.ax;
  sample.accel_y = s.ay;
  sample.accel_z = 0.0f;
  sample.gyro_x = 0.0f;
  sample.gyro_y = 0.0f;
  sample.gyro_z = s.gz;
  sample.temp_c = 55.0f + 0.01f * static_cast<float>(s.edge);

  ++s_idx;
  if (s_idx >= s_lap_len) {
    s_idx = 0;
    ++s_lap_count;
  }
  return true;
}
