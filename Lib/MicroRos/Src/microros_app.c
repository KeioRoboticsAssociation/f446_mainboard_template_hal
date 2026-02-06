#include "microros_app.h"

#include "microros_core.h"
#define MICROROS_RCL_LOG_PREFIX "app"
#include "microros_rcl_check.h"
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/u_int32.h>

#include <stdio.h>

static rcl_publisher_t count_publisher;
static rcl_timer_t counter_timer;
static std_msgs__msg__UInt32 counter_msg;
static uint32_t counter = 0;

static void counter_timer_callback(rcl_timer_t* timer_, int64_t last_call_time)
{
  counter_msg.data = counter++;
  MICROROS_RCL_WARN(rcl_publish(&count_publisher, &counter_msg, NULL));
}

rcl_ret_t microros_app_setup(microros_core_t* core)
{
  if (core == NULL) {
    return RCL_RET_INVALID_ARGUMENT;
  }

  count_publisher = rcl_get_zero_initialized_publisher();
  counter_timer = rcl_get_zero_initialized_timer();

  MICROROS_RCL_TRY(rclc_publisher_init_best_effort(
      &count_publisher, &core->node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32), "/f446/counter"));

  std_msgs__msg__UInt32__init(&counter_msg);

  MICROROS_RCL_TRY(rclc_timer_init_default(
      &counter_timer, &core->support, RCL_MS_TO_NS(1000U), counter_timer_callback));

  MICROROS_RCL_TRY(rclc_executor_add_timer(&core->executor, &counter_timer));

  return RCL_RET_OK;
}

void microros_app_loop(microros_core_t* core)
{
  // Optional user hook for non-timer based periodic work.

}
