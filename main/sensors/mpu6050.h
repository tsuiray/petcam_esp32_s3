#pragma once

#include "esp_err.h"
#include <stdbool.h>

typedef struct {
    float accel_x; /* m/s^2 */
    float accel_y;
    float accel_z;
    float gyro_x;  /* rad/s */
    float gyro_y;
    float gyro_z;
    float temp_c;
} mpu6050_sample_t;

esp_err_t mpu6050_init(void);
esp_err_t mpu6050_read(mpu6050_sample_t *sample);
bool mpu6050_is_ready(void);
