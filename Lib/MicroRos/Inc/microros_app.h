#pragma once

#include "stm32f4xx_hal.h"

#include <rcl/rcl.h>

#ifdef __cplusplus
extern "C" {
#endif

struct microros_core;
typedef struct microros_core microros_core_t;

rcl_ret_t microros_app_setup(microros_core_t* core);
void microros_app_loop(microros_core_t* core);

#ifdef __cplusplus
}
#endif
