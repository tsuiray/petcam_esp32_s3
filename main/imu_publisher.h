#pragma once

#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcl/rcl.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Create /imu/data publisher and a periodic timer on the given node/support.
 * Adds the timer to executor (expects capacity for at least 1 handle).
 */
rcl_ret_t imu_publisher_init(rcl_node_t *node,
                             rclc_support_t *support,
                             rclc_executor_t *executor);

#ifdef __cplusplus
}
#endif
