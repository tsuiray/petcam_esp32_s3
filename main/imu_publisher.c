#include "imu_publisher.h"
#include "board_config.h"
#include "sensors/mpu6050.h"

#include <rcl/error_handling.h>
#include <rmw_microros/rmw_microros.h>
#include <sensor_msgs/msg/imu.h>
#include <stdio.h>
#include <string.h>

#define RCSOFTCHECK(fn)                                                        \
    {                                                                          \
        rcl_ret_t temp_rc = (fn);                                              \
        if ((temp_rc != RCL_RET_OK)) {                                         \
            printf("Failed status on line %d: %d. Continuing.\n", __LINE__,    \
                   (int)temp_rc);                                              \
        }                                                                      \
    }

static rcl_publisher_t s_imu_pub;
static rcl_timer_t s_imu_timer;
static sensor_msgs__msg__Imu s_imu_msg;
static char s_frame_id[] = "imu_link";

static void imu_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)last_call_time;
    if (timer == NULL) {
        return;
    }

    mpu6050_sample_t sample;
    if (mpu6050_read(&sample) != ESP_OK) {
        return;
    }

    int64_t stamp_ms = rmw_uros_epoch_millis();
    s_imu_msg.header.stamp.sec = (int32_t)(stamp_ms / 1000);
    s_imu_msg.header.stamp.nanosec = (uint32_t)((stamp_ms % 1000) * 1000000);

    /* No fused orientation from MPU6050 alone. */
    s_imu_msg.orientation.x = 0.0;
    s_imu_msg.orientation.y = 0.0;
    s_imu_msg.orientation.z = 0.0;
    s_imu_msg.orientation.w = 1.0;
    s_imu_msg.orientation_covariance[0] = -1.0;

    s_imu_msg.angular_velocity.x = sample.gyro_x;
    s_imu_msg.angular_velocity.y = sample.gyro_y;
    s_imu_msg.angular_velocity.z = sample.gyro_z;

    s_imu_msg.linear_acceleration.x = sample.accel_x;
    s_imu_msg.linear_acceleration.y = sample.accel_y;
    s_imu_msg.linear_acceleration.z = sample.accel_z;

    RCSOFTCHECK(rcl_publish(&s_imu_pub, &s_imu_msg, NULL));
}

rcl_ret_t imu_publisher_init(rcl_node_t *node,
                             rclc_support_t *support,
                             rclc_executor_t *executor)
{
    memset(&s_imu_msg, 0, sizeof(s_imu_msg));
    s_imu_msg.header.frame_id.data = s_frame_id;
    s_imu_msg.header.frame_id.size = strlen(s_frame_id);
    s_imu_msg.header.frame_id.capacity = sizeof(s_frame_id);

    for (size_t i = 0; i < 9; ++i) {
        s_imu_msg.angular_velocity_covariance[i] = 0.0;
        s_imu_msg.linear_acceleration_covariance[i] = 0.0;
        s_imu_msg.orientation_covariance[i] = 0.0;
    }
    s_imu_msg.orientation_covariance[0] = -1.0;

    rcl_ret_t rc = rclc_publisher_init_best_effort(
        &s_imu_pub,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu/data");
    if (rc != RCL_RET_OK) {
        return rc;
    }

    const unsigned int period_ms = 1000U / BOARD_IMU_PUBLISH_HZ;
    rc = rclc_timer_init_default(
        &s_imu_timer,
        support,
        RCL_MS_TO_NS(period_ms),
        imu_timer_callback);
    if (rc != RCL_RET_OK) {
        return rc;
    }

    return rclc_executor_add_timer(executor, &s_imu_timer);
}
