/*
 * PetCam ESP32-S3 — micro-ROS (Humble) IMU over Wi-Fi UDP
 *
 * Both ESP32-S3 and Orin Nano join the same home Wi-Fi AP as STA.
 * Publishes sensor_msgs/Imu on /imu/data to micro-ros-agent on the Orin.
 *
 * Board: Tools → Board → ESP32 Arduino → ESP32S3 Dev Module (or your S3 board)
 * Library: micro_ros_arduino (branch/release matching Humble)
 *
 * Open this folder as the sketch: File → Open → .../esp32/esp32.ino
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

enum class AgentState : uint8_t { WaitingAgent = 0, Connected = 1 };

rcl_publisher_t publisher;
sensor_msgs__msg__Imu imu_msg;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;

static char frame_id[] = "imu_link";
static AgentState agent_state = AgentState::WaitingAgent;
static bool entities_ready = false;
static bool wifi_transport_configured = false;

#ifndef LED_BUILTIN
#define LED_PIN 2
#else
#define LED_PIN LED_BUILTIN
#endif

#define RCSOFTCHECK(fn)                                                        \
  do {                                                                         \
    rcl_ret_t temp_rc = (fn);                                                  \
    (void)temp_rc;                                                             \
  } while (0)

#define EXECUTE_EVERY_N_MS(MS, X)                                              \
  do {                                                                         \
    static uint32_t last_exec_ms = 0;                                           \
    if ((millis() - last_exec_ms) >= static_cast<uint32_t>(MS)) {              \
      last_exec_ms = millis();                                                 \
      X;                                                                       \
    }                                                                          \
  } while (0)

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
  if (timer_handle == NULL || !entities_ready) {
    return;
  }

  Mpu6050Sample sample;
  if (!mpu6050Read(sample)) {
    return;
  }

  fill_imu_message(sample);
  RCSOFTCHECK(rcl_publish(&publisher, &imu_msg, NULL));
}

bool ensure_wifi_connected() {
  if (WiFi.status() == WL_CONNECTED) {
    return true;
  }

  Serial.println("Wi-Fi disconnected — reconnecting...");
  if (!wifi_transport_configured) {
    set_microros_wifi_transports(WIFI_SSID, WIFI_PASSWORD, MICROROS_AGENT_IP,
                                 MICROROS_AGENT_PORT);
    wifi_transport_configured = true;
  } else {
    WiFi.disconnect();
    delay(100);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  }

  const uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - start) < 15000U) {
    delay(250);
    Serial.print('.');
  }
  Serial.println();

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Wi-Fi reconnect failed");
    return false;
  }

  Serial.print("Wi-Fi IP: ");
  Serial.println(WiFi.localIP());
  return true;
}

void destroy_entities() {
  if (!entities_ready) {
    return;
  }

  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  if (rmw_context != NULL) {
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
  }

  RCSOFTCHECK(rclc_executor_fini(&executor));
  RCSOFTCHECK(rcl_timer_fini(&timer));
  RCSOFTCHECK(rcl_publisher_fini(&publisher, &node));
  RCSOFTCHECK(rcl_node_fini(&node));
  RCSOFTCHECK(rclc_support_fini(&support));

  entities_ready = false;
  Serial.println("micro-ROS entities destroyed; waiting for agent...");
}

bool create_entities() {
  allocator = rcl_get_default_allocator();
  uint8_t step = 0;

  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
    return false;
  }
  step = 1;

  if (rmw_uros_sync_session(1000) == RMW_RET_OK) {
    Serial.println("Agent time sync OK");
  } else {
    Serial.println("Agent time sync failed; stamps may be relative");
  }

  if (rclc_node_init_default(&node, "petcam_esp32_imu", "", &support) !=
      RCL_RET_OK) {
    goto fail;
  }
  step = 2;

  if (rclc_publisher_init_best_effort(
          &publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
          "imu/data") != RCL_RET_OK) {
    goto fail;
  }
  step = 3;

  if (rclc_timer_init_default(&timer, &support,
                              RCL_MS_TO_NS(IMU_PUBLISH_PERIOD_MS),
                              timer_callback) != RCL_RET_OK) {
    goto fail;
  }
  step = 4;

  if (rclc_executor_init(&executor, &support.context, 1, &allocator) !=
      RCL_RET_OK) {
    goto fail;
  }
  step = 5;

  if (rclc_executor_add_timer(&executor, &timer) != RCL_RET_OK) {
    goto fail;
  }

  memset(&imu_msg, 0, sizeof(imu_msg));
  imu_msg.header.frame_id.data = frame_id;
  imu_msg.header.frame_id.size = strlen(frame_id);
  imu_msg.header.frame_id.capacity = sizeof(frame_id);
  imu_msg.orientation_covariance[0] = -1.0;

  entities_ready = true;
  Serial.printf("Publishing sensor_msgs/Imu on /imu/data every %d ms\n",
                IMU_PUBLISH_PERIOD_MS);
  return true;

fail:
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  if (rmw_context != NULL) {
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
  }
  if (step >= 5) {
    RCSOFTCHECK(rclc_executor_fini(&executor));
  }
  if (step >= 4) {
    RCSOFTCHECK(rcl_timer_fini(&timer));
  }
  if (step >= 3) {
    RCSOFTCHECK(rcl_publisher_fini(&publisher, &node));
  }
  if (step >= 2) {
    RCSOFTCHECK(rcl_node_fini(&node));
  }
  if (step >= 1) {
    RCSOFTCHECK(rclc_support_fini(&support));
  }
  entities_ready = false;
  return false;
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  Serial.begin(115200);
  delay(1000);
  Serial.println("PetCam ESP32-S3 micro-ROS IMU (Arduino)");

#if defined(WIFI_SSID) && defined(WIFI_PASSWORD)
  if (strcmp(WIFI_SSID, "YOUR_HOME_WIFI_SSID") == 0) {
    Serial.println(
        "WARNING: Wi-Fi still uses placeholders. Copy "
        "board_config.local.h.example → board_config.local.h and edit.");
  }
#endif

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
  wifi_transport_configured = true;
  Serial.print("Wi-Fi IP: ");
  Serial.println(WiFi.localIP());

  agent_state = AgentState::WaitingAgent;
  Serial.printf("Waiting for micro-ROS agent at %s:%d ...\n", MICROROS_AGENT_IP,
                MICROROS_AGENT_PORT);
}

void loop() {
  EXECUTE_EVERY_N_MS(WIFI_RECONNECT_PERIOD_MS, {
    if (WiFi.status() != WL_CONNECTED) {
      if (entities_ready) {
        destroy_entities();
        agent_state = AgentState::WaitingAgent;
      }
      (void)ensure_wifi_connected();
    }
  });

  switch (agent_state) {
  case AgentState::WaitingAgent: {
    digitalWrite(LED_PIN, (millis() / 500) % 2);
    EXECUTE_EVERY_N_MS(AGENT_PING_PERIOD_MS, {
      if (WiFi.status() == WL_CONNECTED &&
          rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
        Serial.println("micro-ROS agent is reachable");
        if (create_entities()) {
          agent_state = AgentState::Connected;
          digitalWrite(LED_PIN, HIGH);
        } else {
          Serial.println("Entity create failed; will retry");
          destroy_entities();
        }
      }
    });
    break;
  }
  case AgentState::Connected: {
    EXECUTE_EVERY_N_MS(AGENT_PING_PERIOD_MS, {
      if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
        Serial.println("Agent lost — tearing down entities");
        destroy_entities();
        agent_state = AgentState::WaitingAgent;
      }
    });
    if (agent_state == AgentState::Connected && entities_ready) {
      RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
    }
    break;
  }
  }
}
