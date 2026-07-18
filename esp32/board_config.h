#pragma once

/*
 * Board / network defaults for PetCam ESP32-S3.
 *
 * Prefer putting secrets in board_config.local.h (gitignored).
 * Copy board_config.local.h.example → board_config.local.h and edit.
 */
#if __has_include("board_config.local.h")
#include "board_config.local.h"
#endif

/* Wi-Fi: both ESP32-S3 and Orin join the same home AP (STA). Not Wi-Fi Direct. */
#ifndef WIFI_SSID
#define WIFI_SSID "YOUR_HOME_WIFI_SSID"
#endif

#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD "YOUR_HOME_WIFI_PASSWORD"
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

/* Agent ping interval while connected (ms). */
#ifndef AGENT_PING_PERIOD_MS
#define AGENT_PING_PERIOD_MS 500
#endif

/* Wi-Fi reconnect attempt interval (ms). */
#ifndef WIFI_RECONNECT_PERIOD_MS
#define WIFI_RECONNECT_PERIOD_MS 2000
#endif
