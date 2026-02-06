#pragma once

#include <rcl/rcl.h>

#include <stdio.h>

// Minimal rcl/rclc return-code helpers.
// - MICROROS_RCL_TRY(): log + early return (for functions returning rcl_ret_t)
// - MICROROS_RCL_WARN(): log only (for callbacks/loops)

#ifndef MICROROS_RCL_LOG_PREFIX
#define MICROROS_RCL_LOG_PREFIX "microros"
#endif

#define MICROROS_RCL_TRY(expr)                                                                                        \
  do {                                                                                                                \
    rcl_ret_t _rc = (expr);                                                                                            \
    if (_rc != RCL_RET_OK) {                                                                                           \
      printf(MICROROS_RCL_LOG_PREFIX ": %s:%d %s failed rc=%ld\n", __func__, __LINE__, #expr, (long)_rc);             \
      return _rc;                                                                                                      \
    }                                                                                                                 \
  } while (0)

#define MICROROS_RCL_WARN(expr)                                                                                       \
  do {                                                                                                                \
    rcl_ret_t _rc = (expr);                                                                                            \
    if (_rc != RCL_RET_OK) {                                                                                           \
      printf(MICROROS_RCL_LOG_PREFIX ": %s:%d %s failed rc=%ld\n", __func__, __LINE__, #expr, (long)_rc);             \
    }                                                                                                                 \
  } while (0)
