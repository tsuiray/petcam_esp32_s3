#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "board_config.h"
#include "imu_publisher.h"
#include "sensors/mpu6050.h"

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <uros_network_interfaces.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

static const char *TAG = "petcam_uros";

#define RCCHECK(fn)                                                            \
    {                                                                          \
        rcl_ret_t temp_rc = (fn);                                              \
        if ((temp_rc != RCL_RET_OK)) {                                         \
            printf("Failed status on line %d: %d. Aborting.\n", __LINE__,      \
                   (int)temp_rc);                                              \
            vTaskDelete(NULL);                                                 \
        }                                                                      \
    }

static void wait_for_agent(void)
{
#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    ESP_LOGI(TAG, "Waiting for micro-ROS agent at %s:%s ...",
             CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT);
    while (rmw_uros_ping_agent(1000, 1) != RMW_RET_OK) {
        ESP_LOGW(TAG, "Agent not reachable, retrying...");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGI(TAG, "micro-ROS agent is reachable");
#endif
}

static void micro_ros_task(void *arg)
{
    (void)arg;

    wait_for_agent();

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP,
                                             CONFIG_MICRO_ROS_AGENT_PORT,
                                             rmw_options));
#endif

    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    /* Sync time with agent when possible (improves IMU header stamps). */
#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    if (rmw_uros_sync_session(1000) == RMW_RET_OK) {
        ESP_LOGI(TAG, "Agent time sync OK");
    } else {
        ESP_LOGW(TAG, "Agent time sync failed; stamps may be relative");
    }
#endif

    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "petcam_esp32_imu", "", &support));

    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

    RCCHECK(imu_publisher_init(&node, &support, &executor));
    ESP_LOGI(TAG, "Publishing sensor_msgs/Imu on /imu/data at %d Hz", BOARD_IMU_PUBLISH_HZ);

    while (1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        usleep(1000);
    }

    RCCHECK(rcl_node_fini(&node));
    vTaskDelete(NULL);
}

void app_main(void)
{
    esp_err_t imu_err = mpu6050_init();
    if (imu_err != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 init failed (%s); continuing without valid IMU data",
                 esp_err_to_name(imu_err));
    }

#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif

    /* Pin micro-ROS on APP_CPU so PRO_CPU can service Wi-Fi. */
    xTaskCreate(micro_ros_task,
                "uros_task",
                CONFIG_MICRO_ROS_APP_STACK,
                NULL,
                CONFIG_MICRO_ROS_APP_TASK_PRIO,
                NULL);
}
