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

#include <stdio.h>
#include <string.h>

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <sensor_msgs/msg/imu.h>

#include <WiFi.h>
#include <Wire.h>

#include "board_config.h"
#include "mpu6050.h"

#if !defined(ESP32)
#error This sketch targets ESP32 / ESP32-S3 with Wi-Fi.
#endif

rcl_publisher_t publisher;
sensor_msgs__msg__Imu imu_msg;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;

static char frame_id[] = "imu_link";

#ifndef LED_BUILTIN
#define LED_PIN 2
#else
#define LED_PIN LED_BUILTIN
#endif

#define RCCHECK(fn)                 \
  {                                 \
    rcl_ret_t temp_rc = (fn);       \
    if (temp_rc != RCL_RET_OK) {    \
      error_loop();                 \
    }                               \
  }
#define RCSOFTCHECK(fn)             \
  {                                 \
    rcl_ret_t temp_rc = (fn);       \
    (void)temp_rc;                  \
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

  imu_msg.orientation.x = 0.0;
  imu_msg.orientation.y = 0.0;
  imu_msg.orientation.z = 0.0;
  imu_msg.orientation.w = 1.0;
  imu_msg.orientation_covariance[0] = -1.0;

  imu_msg.angular_velocity.x = sample.gyro_x;
  imu_msg.angular_velocity.y = sample.gyro_y;
  imu_msg.angular_velocity.z = sample.gyro_z;

  imu_msg.linear_acceleration.x = sample.accel_x;
  imu_msg.linear_acceleration.y = sample.accel_y;
  imu_msg.linear_acceleration.z = sample.accel_z;
}

void timer_callback(rcl_timer_t *timer_handle, int64_t /*last_call_time*/) {
  if (timer_handle == NULL) {
    return;
  }

  Mpu6050Sample sample;
  if (!mpu6050Read(sample)) {
    return;
  }

  fill_imu_message(sample);
  RCSOFTCHECK(rcl_publish(&publisher, &imu_msg, NULL));
}

void wait_for_agent() {
  Serial.printf("Waiting for micro-ROS agent at %s:%d ...\n", MICROROS_AGENT_IP,
                MICROROS_AGENT_PORT);
  while (rmw_uros_ping_agent(1000, 1) != RMW_RET_OK) {
    Serial.println("Agent not reachable, retrying...");
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

  Wire.begin(BOARD_I2C_SDA, BOARD_I2C_SCL);
  if (!mpu6050Begin(Wire, BOARD_MPU6050_ADDR)) {
    Serial.println("MPU6050 init failed — check wiring / address");
  } else {
    Serial.printf("MPU6050 OK (SDA=%d SCL=%d addr=0x%02X)\n", BOARD_I2C_SDA,
                  BOARD_I2C_SCL, BOARD_MPU6050_ADDR);
  }

  Serial.printf("Connecting Wi-Fi SSID=%s ...\n", WIFI_SSID);
  set_microros_wifi_transports(WIFI_SSID, WIFI_PASSWORD, MICROROS_AGENT_IP,
                               MICROROS_AGENT_PORT);
  Serial.print("Wi-Fi IP: ");
  Serial.println(WiFi.localIP());

  delay(2000);
  wait_for_agent();

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  if (rmw_uros_sync_session(1000) == RMW_RET_OK) {
    Serial.println("Agent time sync OK");
  } else {
    Serial.println("Agent time sync failed; stamps may be relative");
  }

  RCCHECK(rclc_node_init_default(&node, "petcam_esp32_imu", "", &support));

  RCCHECK(rclc_publisher_init_best_effort(
      &publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
      "imu/data"));

  RCCHECK(rclc_timer_init_default(&timer, &support,
                                  RCL_MS_TO_NS(IMU_PUBLISH_PERIOD_MS),
                                  timer_callback));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  memset(&imu_msg, 0, sizeof(imu_msg));
  imu_msg.header.frame_id.data = frame_id;
  imu_msg.header.frame_id.size = strlen(frame_id);
  imu_msg.header.frame_id.capacity = sizeof(frame_id);
  imu_msg.orientation_covariance[0] = -1.0;

  Serial.printf("Publishing sensor_msgs/Imu on /imu/data every %d ms\n",
                IMU_PUBLISH_PERIOD_MS);
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
