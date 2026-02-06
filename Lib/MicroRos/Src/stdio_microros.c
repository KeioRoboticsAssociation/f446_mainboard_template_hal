#include "microros_printf.h"

int __io_putchar(int ch)
{
  microros_printf_putc((char)ch);
  return ch;
}

