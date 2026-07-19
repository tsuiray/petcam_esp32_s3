#pragma once

/* Wi-Fi: both ESP32-S3 and Orin join the same home AP (STA). Not Wi-Fi Direct. */
#ifndef WIFI_SSID
#define WIFI_SSID "NOKIA-3121"
#endif

#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD "Rp7rcAxz5e"
#endif

/* Orin LAN IP on that AP (DHCP reservation recommended). */
#ifndef MICROROS_AGENT_IP
#define MICROROS_AGENT_IP "192.168.18.15"
#endif

#ifndef MICROROS_AGENT_PORT
#define MICROROS_AGENT_PORT 8888
#endif

/* MPU6050 I2C — adjust to your schematic. */
#ifndef BOARD_I2C_SDA
#define BOARD_I2C_SDA 8
#endif

#ifndef BOARD_I2C_SCL
#define BOARD_I2C_SCL 9
#endif

#ifndef BOARD_MPU6050_ADDR
#define BOARD_MPU6050_ADDR 0x68
#endif

/* IMU publish period (ms). 20 ms = 50 Hz. */
#ifndef IMU_PUBLISH_PERIOD_MS
#define IMU_PUBLISH_PERIOD_MS 20
#endif

/* Serial: print published IMU every N samples (1 = every packet). */
#ifndef IMU_SERIAL_LOG_EVERY_N
#define IMU_SERIAL_LOG_EVERY_N 10
#endif

/*
 * IMU data source — switch in code before upload:
 *   IMU_DATA_MODE_SIM  → synthetic path around ~500 sq ft L-home (imu_sim.*)
 *   IMU_DATA_MODE_REAL → MPU6050 hardware
 */
#define IMU_DATA_MODE_REAL 0
#define IMU_DATA_MODE_SIM 1

#ifndef IMU_DATA_MODE
#define IMU_DATA_MODE IMU_DATA_MODE_SIM
#endif
