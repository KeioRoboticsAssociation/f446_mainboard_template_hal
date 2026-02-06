#include "microros_printf.h"

#include <stdint.h>

#include "stm32f4xx.h"

#ifndef MICROROS_PRINTF_RING_SIZE
#define MICROROS_PRINTF_RING_SIZE 2048U
#endif

static volatile bool g_enabled = true;

static volatile uint32_t g_head = 0;
static volatile uint32_t g_tail = 0;
static char g_buf[MICROROS_PRINTF_RING_SIZE];

static inline uint32_t ring_next(uint32_t v)
{
  return (v + 1U) % MICROROS_PRINTF_RING_SIZE;
}

static inline void crit_enter(uint32_t* primask_out)
{
  *primask_out = __get_PRIMASK();
  __disable_irq();
}

static inline void crit_exit(uint32_t primask)
{
  if (primask == 0U) {
    __enable_irq();
  }
}

void microros_printf_set_enabled(bool enabled)
{
  uint32_t primask;
  crit_enter(&primask);
  g_enabled = enabled;
  crit_exit(primask);
}

bool microros_printf_is_enabled(void)
{
  return g_enabled;
}

void microros_printf_putc(char c)
{
  if (!g_enabled) {
    return;
  }

  uint32_t primask;
  crit_enter(&primask);

  uint32_t next = ring_next(g_head);
  if (next == g_tail) {
    // Overflow: drop.
    crit_exit(primask);
    return;
  }

  g_buf[g_head] = c;
  g_head = next;

  crit_exit(primask);
}

size_t microros_printf_pop_line(char* out, size_t out_size)
{
  if (out == NULL || out_size == 0U) {
    return 0;
  }

  if (out_size == 1U) {
    out[0] = '\0';
    return 0;
  }

  uint32_t primask;
  crit_enter(&primask);

  size_t n = 0;
  while (g_tail != g_head && n < (out_size - 1U)) {
    char c = g_buf[g_tail];
    g_tail = ring_next(g_tail);
    out[n++] = c;
    if (c == '\n') {
      break;
    }
  }

  crit_exit(primask);

  out[n] = '\0';
  return n;
}

