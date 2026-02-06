#pragma once

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// Capture printf/stdout stream into an internal ring buffer.
// Publishing to ROS topics is handled separately (e.g. from microros_core_loop()).

void microros_printf_set_enabled(bool enabled);
bool microros_printf_is_enabled(void);

// Push one character into the buffer (drop on overflow).
void microros_printf_putc(char c);

// Pop up to (out_size - 1) characters, stopping at '\n' if present.
// Returns number of bytes written (not including terminating '\0').
size_t microros_printf_pop_line(char* out, size_t out_size);

#ifdef __cplusplus
}
#endif
