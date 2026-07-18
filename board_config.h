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
#define MICROROS_AGENT_IP "192.168.1.100"
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
