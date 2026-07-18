#pragma once

/* MPU6050 I2C wiring — adjust to your schematic. */
#define BOARD_I2C_PORT          0
#define BOARD_I2C_SDA_GPIO      8
#define BOARD_I2C_SCL_GPIO      9
#define BOARD_I2C_FREQ_HZ       400000

/* AD0 low -> 0x68, AD0 high -> 0x69 */
#define BOARD_MPU6050_ADDR      0x68

/* IMU publish rate (Hz). Plan target: 50–100 Hz. */
#define BOARD_IMU_PUBLISH_HZ    50
