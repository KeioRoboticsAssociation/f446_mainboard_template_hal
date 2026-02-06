#include "microros_core.h"

#include "microros_app.h"
#include "main.h"
#include "microros_config.h"
#include "microros_printf.h"
#include "microros_transport_uart.h"

#include <rcl/error_handling.h>
#include <rmw_microros/ping.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/string.h>

#include <stdio.h>
#include <string.h>

#ifndef MICROROS_NODE_NAME
#define MICROROS_NODE_NAME "f446_node"
#endif

#ifndef MICROROS_EXECUTOR_HANDLES
#define MICROROS_EXECUTOR_HANDLES 4
#endif

// Reconnect tuning.
#ifndef MICROROS_AGENT_PING_TIMEOUT_MS
#define MICROROS_AGENT_PING_TIMEOUT_MS 50
#endif

#ifndef MICROROS_AGENT_PING_INTERVAL_MS
#define MICROROS_AGENT_PING_INTERVAL_MS 200
#endif

#ifndef MICROROS_AGENT_MAX_CONSECUTIVE_PING_FAILURES
#define MICROROS_AGENT_MAX_CONSECUTIVE_PING_FAILURES 3
#endif

#ifndef MICROROS_AGENT_BOOT_WAIT_DELAY_MS
#define MICROROS_AGENT_BOOT_WAIT_DELAY_MS 50
#endif

#ifndef MICROROS_PRINTF_TOPIC
#define MICROROS_PRINTF_TOPIC "/f446/printf"
#endif

#ifndef MICROROS_PRINTF_MSG_CAPACITY
#define MICROROS_PRINTF_MSG_CAPACITY 256U
#endif

#ifndef MICROROS_PRINTF_MAX_LINES_PER_LOOP
#define MICROROS_PRINTF_MAX_LINES_PER_LOOP 4U
#endif

static microros_core_t g_core;

static rcl_publisher_t g_printf_publisher;
static std_msgs__msg__String g_printf_msg;
static char g_printf_msg_buf[MICROROS_PRINTF_MSG_CAPACITY];

microros_core_t* microros_core_get(void)
{
  return &g_core;
}

static void microros_core_log_line_to_ring(const char* s)
{
  if (s == NULL) {
    return;
  }
  while (*s != '\0') {
    microros_printf_putc(*s++);
  }
  microros_printf_putc('\n');
}

static void microros_core_print_rcl_error(const char* context, int32_t rcl_ret)
{
  const char* rcl_err = rcl_get_error_string().str;
  if (rcl_err == NULL) {
    rcl_err = "";
  }

  char line[192];
  (void)snprintf(
      line,
      sizeof(line),
      "micro-ROS error: ret=%ld ctx=%s err=%s",
      (long)rcl_ret,
      (context != NULL) ? context : "",
      rcl_err);
  rcl_reset_error();

  bool was_enabled = microros_printf_is_enabled();
  microros_printf_set_enabled(true);
  microros_core_log_line_to_ring(line);
  microros_printf_set_enabled(was_enabled);
}

static bool microros_core_printf_pub_init(void)
{
  g_printf_publisher = rcl_get_zero_initialized_publisher();
  g_core.printf_pub_ready = false;

  rcl_ret_t ret = rclc_publisher_init_best_effort(
      &g_printf_publisher,
      &g_core.node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      MICROROS_PRINTF_TOPIC);
  if (ret != RCL_RET_OK) {
    microros_core_print_rcl_error("printf publisher init", (int32_t)ret);
    return false;
  }

  g_printf_msg.data.data = g_printf_msg_buf;
  g_printf_msg.data.size = 0;
  g_printf_msg.data.capacity = MICROROS_PRINTF_MSG_CAPACITY;
  g_printf_msg.data.data[0] = '\0';
  g_core.printf_pub_ready = true;
  return true;
}

static bool microros_core_rcl_init(void)
{
  g_core.allocator = rcl_get_default_allocator();
  rcl_ret_t ret = rclc_support_init(&g_core.support, 0, NULL, &g_core.allocator);
  if (ret != RCL_RET_OK) {
    microros_core_print_rcl_error("support init", (int32_t)ret);
    return false;
  }

  ret = rclc_node_init_default(&g_core.node, MICROROS_NODE_NAME, "", &g_core.support);
  if (ret != RCL_RET_OK) {
    microros_core_print_rcl_error("node init", (int32_t)ret);
    return false;
  }

  ret = rclc_executor_init(&g_core.executor, &g_core.support.context, MICROROS_EXECUTOR_HANDLES, &g_core.allocator);
  if (ret != RCL_RET_OK) {
    microros_core_print_rcl_error("executor init", (int32_t)ret);
    return false;
  }

  if (!microros_core_printf_pub_init()) {
    return false;
  }

  return true;
}

static void microros_core_mark_connected(void)
{
  g_core.rcl_initialized = true;
  g_core.last_ping_ms = HAL_GetTick();
  g_core.ping_failures = 0;
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
}

static void microros_core_mark_disconnected(void)
{
  g_core.rcl_initialized = false;
  g_core.ping_failures = 0;
  g_core.printf_pub_ready = false;
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
}

static void microros_core_printf_flush(void)
{
  if (!g_core.rcl_initialized || !g_core.printf_pub_ready) {
    return;
  }

  for (uint32_t i = 0; i < MICROROS_PRINTF_MAX_LINES_PER_LOOP; i++) {
    size_t n = microros_printf_pop_line(g_printf_msg.data.data, g_printf_msg.data.capacity);
    if (n == 0U) {
      break;
    }
    g_printf_msg.data.size = n;
    rcl_ret_t pub_ret = rcl_publish(&g_printf_publisher, &g_printf_msg, NULL);
    (void)pub_ret;
  }
}

static void microros_core_reset_now(void)
{
  __disable_irq();
  NVIC_SystemReset();
  while (1) {
  }
}

void microros_core_setup(UART_HandleTypeDef* transport_uart)
{
  // パブリッシャ/サブスクライバの有無に関係なく「常に必要」な初期化。
  // ここでやること：
  // - UART設定（ボーレート上書き）
  // - Micro XRCE-DDS 用のカスタムトランスポート設定
  // - rcl/rclc の support/node/executor（サポート/ノード/エグゼキュータ）初期化
  if (HAL_UART_DeInit(transport_uart) != HAL_OK) {
    Error_Handler();
  }
  transport_uart->Init.BaudRate = MICROROS_UART_BAUDRATE;
  if (HAL_UART_Init(transport_uart) != HAL_OK) {
    Error_Handler();
  }

  g_core.transport_uart = transport_uart;
  g_core.support = (rclc_support_t){0};
  g_core.node = rcl_get_zero_initialized_node();
  g_core.executor = rclc_executor_get_zero_initialized_executor();
  g_printf_publisher = rcl_get_zero_initialized_publisher();
  microros_core_mark_disconnected();

  microros_transport_uart_init(transport_uart);
  microros_printf_set_enabled(false);

  // Wait for agent availability so that a board boot *before* the agent doesn't get stuck in Error_Handler().
  while (rmw_uros_ping_agent(MICROROS_AGENT_PING_TIMEOUT_MS, 1) != RMW_RET_OK) {
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    HAL_Delay(MICROROS_AGENT_BOOT_WAIT_DELAY_MS);
  }
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  if (!microros_core_rcl_init()) {
    microros_core_reset_now();
  }

  rcl_ret_t app_ret = microros_app_setup(&g_core);
  if (app_ret != RCL_RET_OK) {
    microros_core_print_rcl_error("app setup", (int32_t)app_ret);
    microros_core_mark_connected();
    microros_printf_set_enabled(true);
    microros_core_printf_flush();
    HAL_Delay(50);
    microros_core_printf_flush();
    microros_core_reset_now();
  }
  microros_core_mark_connected();
  microros_printf_set_enabled(true);
}

void microros_core_loop(void)
{
  uint32_t now = HAL_GetTick();

  if (!g_core.rcl_initialized) {
    return;
  }

  (void)rclc_executor_spin_some(&g_core.executor, RCL_MS_TO_NS(2));

  // Monitor agent liveness periodically. If it goes away, reset the MCU.
  if ((now - g_core.last_ping_ms) >= MICROROS_AGENT_PING_INTERVAL_MS) {
    g_core.last_ping_ms = now;
    if (rmw_uros_ping_agent(MICROROS_AGENT_PING_TIMEOUT_MS, 1) != RMW_RET_OK) {
      if (g_core.ping_failures < 0xFFu) {
        g_core.ping_failures++;
      }
    } else {
      g_core.ping_failures = 0;
    }

    if (g_core.ping_failures >= MICROROS_AGENT_MAX_CONSECUTIVE_PING_FAILURES) {
      microros_core_log_line_to_ring("micro-ROS error: agent lost -> reset");
      microros_core_printf_flush();
      microros_core_reset_now();
    }
  }

  microros_core_printf_flush();
}
