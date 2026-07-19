/*
 * PetCam ESP32-S3 — micro-ROS (Humble) IMU over Wi-Fi UDP
 *
 * Both ESP32-S3 and Orin Nano join the same home Wi-Fi AP as STA.
 * Publishes sensor_msgs/Imu on /imu/data to micro-ros-agent on the Orin.
 *
 * Board: Tools → Board → ESP32 Arduino → ESP32S3 Dev Module (or your S3 board)
 * Library: micro_ros_arduino (branch/release matching Humble)
 */

#include <micro_ros_arduino.h>

#include <math.h>
#include <stdio.h>
#include <string.h>

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <sensor_msgs/msg/imu.h>

#include <WiFi.h>
#include <Wire.h>

#include "board_config.h"
#include "mpu6050.h"
#if IMU_DATA_MODE == IMU_DATA_MODE_SIM
#include "imu_sim.h"
#endif

#if !defined(ESP32)
#error This sketch targets ESP32 / ESP32-S3 with Wi-Fi.
#endif

rcl_publisher_t publisher;
sensor_msgs__msg__Imu imu_msg;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

static char frame_id[] = "imu_link";

#ifndef LED_BUILTIN
#define LED_PIN 2
#else
#define LED_PIN LED_BUILTIN
#endif

#define RCCHECK(fn)                                                            \
  {                                                                            \
    rcl_ret_t temp_rc = (fn);                                                  \
    if (temp_rc != RCL_RET_OK) {                                               \
      Serial.printf("RCCHECK fail line %d rc=%d\n", __LINE__, (int)temp_rc);   \
      Serial.flush();                                                          \
      error_loop();                                                            \
    }                                                                          \
  }
#define RCSOFTCHECK(fn)                                                        \
  {                                                                            \
    rcl_ret_t temp_rc = (fn);                                                  \
    if (temp_rc != RCL_RET_OK) {                                               \
      Serial.printf("RCSOFT fail line %d rc=%d\n", __LINE__, (int)temp_rc);    \
    }                                                                          \
  }

void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void fill_imu_message(const Mpu6050Sample &sample) {
  const int64_t stamp_ms = rmw_uros_epoch_millis();
  imu_msg.header.stamp.sec = static_cast<int32_t>(stamp_ms / 1000);
  imu_msg.header.stamp.nanosec =
      static_cast<uint32_t>((stamp_ms % 1000) * 1000000UL);

#if IMU_DATA_MODE == IMU_DATA_MODE_SIM
  /* Yaw-only attitude so create_map can use orientation if it wants. */
  {
    const float yaw = imuSimGetYaw();
    const float half = 0.5f * yaw;
    imu_msg.orientation.x = 0.0;
    imu_msg.orientation.y = 0.0;
    imu_msg.orientation.z = sinf(half);
    imu_msg.orientation.w = cosf(half);
    imu_msg.orientation_covariance[0] = 0.01;
    imu_msg.orientation_covariance[4] = 0.01;
    imu_msg.orientation_covariance[8] = 0.05;
  }
#else
  imu_msg.orientation.x = 0.0;
  imu_msg.orientation.y = 0.0;
  imu_msg.orientation.z = 0.0;
  imu_msg.orientation.w = 1.0;
  imu_msg.orientation_covariance[0] = -1.0;
#endif

  imu_msg.angular_velocity.x = sample.gyro_x;
  imu_msg.angular_velocity.y = sample.gyro_y;
  imu_msg.angular_velocity.z = sample.gyro_z;

  imu_msg.linear_acceleration.x = sample.accel_x;
  imu_msg.linear_acceleration.y = sample.accel_y;
  imu_msg.linear_acceleration.z = sample.accel_z;
}

void log_published_imu(const Mpu6050Sample &sample, uint32_t seq) {
  Serial.printf(
      "UDP/IMU #%lu ax=%.3f ay=%.3f az=%.3f gx=%.3f gy=%.3f gz=%.3f",
      static_cast<unsigned long>(seq), sample.accel_x, sample.accel_y,
      sample.accel_z, sample.gyro_x, sample.gyro_y, sample.gyro_z);
#if IMU_DATA_MODE == IMU_DATA_MODE_SIM
  Serial.printf(" | SIM lap=%lu i=%lu edge=%d v=%.2f yaw=%.1fdeg",
                static_cast<unsigned long>(imuSimGetLap()),
                static_cast<unsigned long>(imuSimGetSampleIndex()),
                imuSimGetEdge(), imuSimGetSpeed(),
                imuSimGetYaw() * 57.2957795f);
#endif
  Serial.println();
}

void publish_one_imu() {
  Mpu6050Sample sample;
#if IMU_DATA_MODE == IMU_DATA_MODE_SIM
  if (!imuSimRead(sample)) {
    Serial.println("imuSimRead failed");
    return;
  }
#else
  if (!mpu6050Read(sample)) {
    Serial.println("mpu6050Read failed");
    return;
  }
#endif

  fill_imu_message(sample);
  RCSOFTCHECK(rcl_publish(&publisher, &imu_msg, NULL));

  static uint32_t s_seq;
  ++s_seq;
  if (s_seq <= 5 || (s_seq % IMU_SERIAL_LOG_EVERY_N) == 0) {
    log_published_imu(sample, s_seq);
  }
}

static const char *wifi_status_str(wl_status_t s) {
  switch (s) {
    case WL_IDLE_STATUS:
      return "IDLE";
    case WL_NO_SSID_AVAIL:
      return "NO_SSID_AVAIL";
    case WL_SCAN_COMPLETED:
      return "SCAN_COMPLETED";
    case WL_CONNECTED:
      return "CONNECTED";
    case WL_CONNECT_FAILED:
      return "CONNECT_FAILED";
    case WL_CONNECTION_LOST:
      return "CONNECTION_LOST";
    case WL_DISCONNECTED:
      return "DISCONNECTED";
    default:
      return "UNKNOWN";
  }
}

/**
 * Connect STA with Serial progress. ESP32-S3 is 2.4 GHz only.
 * Returns true on success. Fails when timeout_ms elapses.
 */
bool connect_wifi_sta(const char *ssid, const char *password,
                      uint32_t timeout_ms) {
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.disconnect(true);
  delay(200);

  Serial.printf("WiFi connecting to \"%s\" (2.4GHz AP required)...\n", ssid);
  WiFi.begin(ssid, password);

  const uint32_t start = millis();
  uint32_t last_log = 0;
  while (WiFi.status() != WL_CONNECTED) {
    const uint32_t now = millis();
    if (now - start > timeout_ms) {
      Serial.printf("WiFi TIMEOUT after %lu ms — status=%s (%d)\n",
                    static_cast<unsigned long>(timeout_ms),
                    wifi_status_str(WiFi.status()), static_cast<int>(WiFi.status()));
      Serial.println(
          "Check: SSID/password, 2.4GHz band (not 5GHz-only), AP near board.");
      return false;
    }
    if (now - last_log >= 1000) {
      last_log = now;
      Serial.printf("  ... waiting WiFi status=%s (%d) t=%lus\n",
                    wifi_status_str(WiFi.status()),
                    static_cast<int>(WiFi.status()),
                    static_cast<unsigned long>((now - start) / 1000));
    }
    delay(100);
  }

  Serial.print("WiFi OK  IP=");
  Serial.print(WiFi.localIP());
  Serial.print("  GW=");
  Serial.print(WiFi.gatewayIP());
  Serial.print("  RSSI=");
  Serial.println(WiFi.RSSI());
  return true;
}

/** Bind micro-ROS UDP transport after WiFi is already up (no infinite wait). */
void bind_microros_wifi_transport(const char *agent_ip, uint16_t agent_port) {
  static micro_ros_agent_locator locator;
  static char ip_buf[32];
  strncpy(ip_buf, agent_ip, sizeof(ip_buf) - 1);
  ip_buf[sizeof(ip_buf) - 1] = '\0';
  locator.address.fromString(ip_buf);
  locator.port = static_cast<int>(agent_port);

  rmw_uros_set_custom_transport(
      false, reinterpret_cast<void *>(&locator), arduino_wifi_transport_open,
      arduino_wifi_transport_close, arduino_wifi_transport_write,
      arduino_wifi_transport_read);

  Serial.printf("micro-ROS UDP transport -> %s:%u\n", agent_ip, agent_port);
}

void wait_for_agent() {
  Serial.printf("Waiting for micro-ROS agent at %s:%d ...\n", MICROROS_AGENT_IP,
                MICROROS_AGENT_PORT);
  while (rmw_uros_ping_agent(1000, 1) != RMW_RET_OK) {
    Serial.printf("Agent not reachable (WiFi %s IP=%s), retrying...\n",
                  WiFi.status() == WL_CONNECTED ? "OK" : "DOWN",
                  WiFi.localIP().toString().c_str());
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi dropped — reconnecting...");
      connect_wifi_sta(WIFI_SSID, WIFI_PASSWORD, 30000);
    }
    delay(1000);
  }
  Serial.println("micro-ROS agent is reachable");
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  Serial.begin(115200);
  delay(1000);
  Serial.println("PetCam ESP32-S3 micro-ROS IMU (Arduino)");

#if IMU_DATA_MODE == IMU_DATA_MODE_SIM
  {
    const float dt = IMU_PUBLISH_PERIOD_MS / 1000.0f;
    imuSimBegin(dt);
    Serial.printf("IMU mode: SIMULATION (L-home ~%.1f m^2 / ~%.0f sq ft)\n",
                  imuSimFootprintAreaM2(), imuSimFootprintAreaM2() * 10.7639f);
    Serial.println(
        "SIM publishes WORLD-frame ax/ay (z=0) for deterministic closed laps");
  }
#else
  Serial.println("IMU mode: REAL (MPU6050)");
  Wire.begin(BOARD_I2C_SDA, BOARD_I2C_SCL);
  if (!mpu6050Begin(Wire, BOARD_MPU6050_ADDR)) {
    Serial.println("MPU6050 init failed — check wiring / address");
  } else {
    Serial.printf("MPU6050 OK (SDA=%d SCL=%d addr=0x%02X)\n", BOARD_I2C_SDA,
                  BOARD_I2C_SCL, BOARD_MPU6050_ADDR);
  }
#endif

  /* Do NOT use set_microros_wifi_transports() — it blocks forever with no logs. */
  while (!connect_wifi_sta(WIFI_SSID, WIFI_PASSWORD, 45000)) {
    Serial.println("Retry WiFi in 3s...");
    delay(3000);
  }
  bind_microros_wifi_transport(MICROROS_AGENT_IP, MICROROS_AGENT_PORT);

  delay(500);
  wait_for_agent();

  allocator = rcl_get_default_allocator();
  Serial.println("Init rclc_support...");
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  if (rmw_uros_sync_session(1000) == RMW_RET_OK) {
    Serial.println("Agent time sync OK");
  } else {
    Serial.println("Agent time sync failed; stamps may be relative");
  }

  Serial.println("Init node...");
  RCCHECK(rclc_node_init_default(&node, "petcam_esp32_imu", "", &support));

  Serial.println("Init publisher /imu/data (sensor_msgs/Imu)...");
  RCCHECK(rclc_publisher_init_best_effort(
      &publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
      "imu/data"));
  Serial.println("Publisher OK");

  memset(&imu_msg, 0, sizeof(imu_msg));
  imu_msg.header.frame_id.data = frame_id;
  imu_msg.header.frame_id.size = strlen(frame_id);
  imu_msg.header.frame_id.capacity = sizeof(frame_id);
  imu_msg.orientation_covariance[0] = -1.0;

  Serial.printf("Publishing sensor_msgs/Imu on /imu/data every %d ms\n",
                IMU_PUBLISH_PERIOD_MS);
  Serial.printf("Serial IMU log: first 5 packets, then every %d\n",
                IMU_SERIAL_LOG_EVERY_N);
  Serial.println("Entering loop()");
}

void loop() {
  static uint32_t s_last_ms;
  static uint32_t s_hb;
  const uint32_t now = millis();

  if (now - s_last_ms >= static_cast<uint32_t>(IMU_PUBLISH_PERIOD_MS)) {
    s_last_ms = now;
    publish_one_imu();
  }

  if (now - s_hb >= 5000) {
    s_hb = now;
    Serial.printf("loop heartbeat WiFi=%s IP=%s\n",
                  WiFi.status() == WL_CONNECTED ? "OK" : "DOWN",
                  WiFi.localIP().toString().c_str());
  }
}
