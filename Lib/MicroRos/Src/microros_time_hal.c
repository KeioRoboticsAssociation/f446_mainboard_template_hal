#include "stm32f4xx_hal.h"

#include <time.h>

int clock_gettime(int clock_id, struct timespec* tp)
{
  // micro-ROS/newlib が clock_gettime() を要求するため実装する。
  // この最小実装では HAL_GetTick() ベースの粗い時刻だけ返す。
  // clock_id は使わない（未使用引数の警告回避）。
  (void)clock_id;
  if (tp == NULL) {
    return -1;
  }

  uint32_t ms = HAL_GetTick();
  tp->tv_sec = (time_t)(ms / 1000U);
  tp->tv_nsec = (long)((ms % 1000U) * 1000000UL);
  return 0;
}

