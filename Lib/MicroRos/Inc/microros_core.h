#pragma once

#include "stm32f4xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct microros_core {
  UART_HandleTypeDef* transport_uart;
  rcl_allocator_t allocator;
  rclc_support_t support;
  rcl_node_t node;
  rclc_executor_t executor;
  bool rcl_initialized;
  bool printf_pub_ready;
  uint32_t last_ping_ms;
  uint8_t ping_failures;
} microros_core_t;

void microros_core_setup(UART_HandleTypeDef* transport_uart);
void microros_core_loop(void);

microros_core_t* microros_core_get(void);

#ifdef __cplusplus
}
#endif
