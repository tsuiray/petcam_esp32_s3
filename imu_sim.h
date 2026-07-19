#pragma once

#include "mpu6050.h"

/**
 * Deterministic SIM for Orin create_map.
 *
 * Precomputes one closed L-home lap of world-frame accel + yaw-rate samples.
 * Replaying the same buffer every lap so integrated trajectories overlap.
 *
 * linear_acceleration.x/y = world-frame accel (z=0, no gravity) so a naive
 * integrator (vx+=ax*dt; x+=vx*dt) redraws the same polygon each loop.
 */
void imuSimBegin(float dt_seconds);
bool imuSimRead(Mpu6050Sample &sample);

float imuSimFootprintAreaM2();
float imuSimGetYaw();
float imuSimGetSpeed();
int imuSimGetEdge();
uint32_t imuSimGetLap();
uint32_t imuSimGetSampleIndex();
