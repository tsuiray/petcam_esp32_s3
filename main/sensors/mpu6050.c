#include "sensors/mpu6050.h"
#include "board_config.h"

#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <math.h>

static const char *TAG = "mpu6050";

#define MPU6050_REG_SMPLRT_DIV   0x19
#define MPU6050_REG_CONFIG       0x1A
#define MPU6050_REG_GYRO_CONFIG  0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_PWR_MGMT_1   0x6B
#define MPU6050_REG_WHO_AM_I     0x75
#define MPU6050_REG_ACCEL_XOUT_H 0x3B

/* ±2g and ±250 deg/s */
#define ACCEL_LSB_PER_G          16384.0f
#define GYRO_LSB_PER_DPS         131.0f
#define GRAVITY_MPS2             9.80665f
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define DEG_TO_RAD               ((float)(M_PI / 180.0))


static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_dev;
static bool s_ready;

static esp_err_t mpu6050_write_u8(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = {reg, value};
    return i2c_master_transmit(s_dev, buf, sizeof(buf), 100);
}

static esp_err_t mpu6050_read_bytes(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(s_dev, &reg, 1, data, len, 100);
}

esp_err_t mpu6050_init(void)
{
    s_ready = false;

    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = BOARD_I2C_PORT,
        .sda_io_num = BOARD_I2C_SDA_GPIO,
        .scl_io_num = BOARD_I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t err = i2c_new_master_bus(&bus_cfg, &s_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_new_master_bus failed: %s", esp_err_to_name(err));
        return err;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BOARD_MPU6050_ADDR,
        .scl_speed_hz = BOARD_I2C_FREQ_HZ,
    };

    err = i2c_master_bus_add_device(s_bus, &dev_cfg, &s_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_master_bus_add_device failed: %s", esp_err_to_name(err));
        return err;
    }

    uint8_t whoami = 0;
    err = mpu6050_read_bytes(MPU6050_REG_WHO_AM_I, &whoami, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "WHO_AM_I read failed: %s", esp_err_to_name(err));
        return err;
    }
    if (whoami != 0x68) {
        ESP_LOGW(TAG, "Unexpected WHO_AM_I=0x%02X (expected 0x68)", whoami);
    }

    /* Wake up, use X gyro as clock source. */
    err = mpu6050_write_u8(MPU6050_REG_PWR_MGMT_1, 0x01);
    if (err != ESP_OK) {
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    /* DLPF ~44 Hz accel / 42 Hz gyro */
    ESP_ERROR_CHECK_WITHOUT_ABORT(mpu6050_write_u8(MPU6050_REG_CONFIG, 0x03));
    /* Sample rate divider: 1 kHz / (1+9) = 100 Hz internal */
    ESP_ERROR_CHECK_WITHOUT_ABORT(mpu6050_write_u8(MPU6050_REG_SMPLRT_DIV, 9));
    /* Gyro ±250 dps, Accel ±2g */
    ESP_ERROR_CHECK_WITHOUT_ABORT(mpu6050_write_u8(MPU6050_REG_GYRO_CONFIG, 0x00));
    ESP_ERROR_CHECK_WITHOUT_ABORT(mpu6050_write_u8(MPU6050_REG_ACCEL_CONFIG, 0x00));

    s_ready = true;
    ESP_LOGI(TAG, "MPU6050 ready on I2C addr 0x%02X (SDA=%d SCL=%d)",
             BOARD_MPU6050_ADDR, BOARD_I2C_SDA_GPIO, BOARD_I2C_SCL_GPIO);
    return ESP_OK;
}

bool mpu6050_is_ready(void)
{
    return s_ready;
}

esp_err_t mpu6050_read(mpu6050_sample_t *sample)
{
    if (!sample || !s_ready) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t raw[14];
    esp_err_t err = mpu6050_read_bytes(MPU6050_REG_ACCEL_XOUT_H, raw, sizeof(raw));
    if (err != ESP_OK) {
        return err;
    }

    int16_t ax = (int16_t)((raw[0] << 8) | raw[1]);
    int16_t ay = (int16_t)((raw[2] << 8) | raw[3]);
    int16_t az = (int16_t)((raw[4] << 8) | raw[5]);
    int16_t temp = (int16_t)((raw[6] << 8) | raw[7]);
    int16_t gx = (int16_t)((raw[8] << 8) | raw[9]);
    int16_t gy = (int16_t)((raw[10] << 8) | raw[11]);
    int16_t gz = (int16_t)((raw[12] << 8) | raw[13]);

    sample->accel_x = ((float)ax / ACCEL_LSB_PER_G) * GRAVITY_MPS2;
    sample->accel_y = ((float)ay / ACCEL_LSB_PER_G) * GRAVITY_MPS2;
    sample->accel_z = ((float)az / ACCEL_LSB_PER_G) * GRAVITY_MPS2;
    sample->gyro_x = ((float)gx / GYRO_LSB_PER_DPS) * DEG_TO_RAD;
    sample->gyro_y = ((float)gy / GYRO_LSB_PER_DPS) * DEG_TO_RAD;
    sample->gyro_z = ((float)gz / GYRO_LSB_PER_DPS) * DEG_TO_RAD;
    sample->temp_c = ((float)temp / 340.0f) + 36.53f;

    return ESP_OK;
}
