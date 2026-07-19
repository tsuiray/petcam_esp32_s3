#pragma once

#include "mpu6050.h"

/**
 * Simulation IMU: drives a virtual robot around an L-shaped ~500 sq ft
 * home-like polygon and synthesizes MPU6050-style 6-axis samples.
 *
 * Uses a triangular speed profile on each edge so linear_acceleration.x
 * stays non-zero while driving (constant-velocity cruise would look like
 * "only gravity" on Orin create_map).
 *
 * Switch mode in board_config.h via IMU_DATA_MODE.
 */
void imuSimBegin(float dt_seconds);
bool imuSimRead(Mpu6050Sample &sample);

float imuSimFootprintAreaM2();
float imuSimGetYaw();
float imuSimGetSpeed();
int imuSimGetEdge();
