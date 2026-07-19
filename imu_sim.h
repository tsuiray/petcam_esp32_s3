#pragma once

#include "mpu6050.h"

/**
 * Simulation IMU: drives a virtual robot around an L-shaped ~500 sq ft
 * home-like polygon and synthesizes MPU6050-style 6-axis samples.
 *
 * Switch mode in board_config.h via IMU_DATA_MODE.
 */
void imuSimBegin(float dt_seconds);
bool imuSimRead(Mpu6050Sample &sample);

/** Approximate floor area of the simulated footprint (m^2). */
float imuSimFootprintAreaM2();
