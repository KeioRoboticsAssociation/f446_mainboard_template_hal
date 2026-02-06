#include "microros_transport_uart.h"
#include "microros_config.h"

#include <rmw_microros/rmw_microros.h>
#include <uxr/client/profile/transport/custom/custom_transport.h>

static UART_HandleTypeDef* g_transport_uart = NULL;

static volatile uint16_t g_rx_head = 0;
static volatile uint16_t g_rx_tail = 0;
static uint8_t g_rx_ring[MICROROS_UART_RX_RING_SIZE];
static uint8_t g_rx_dma_buf[MICROROS_UART_RX_DMA_BUFFER_SIZE];
static volatile uint16_t g_rx_dma_last_pos = 0;

static volatile bool g_tx_dma_done = true;
static volatile bool g_tx_dma_error = false;

static inline uint16_t ring_next(uint16_t v)
{
  return (uint16_t)((v + 1U) % MICROROS_UART_RX_RING_SIZE);
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

static inline bool rx_ring_push(uint8_t b)
{
  uint16_t next = ring_next(g_rx_head);
  if (next == g_rx_tail) {
    return false;
  }
  g_rx_ring[g_rx_head] = b;
  g_rx_head = next;
  return true;
}

static inline bool rx_ring_pop(uint8_t* out)
{
  if (g_rx_tail == g_rx_head) {
    return false;
  }
  *out = g_rx_ring[g_rx_tail];
  g_rx_tail = ring_next(g_rx_tail);
  return true;
}

static void rx_dma_copy_into_ring(uint16_t start, uint16_t end)
{
  for (uint16_t i = start; i < end; i++) {
    (void)rx_ring_push(g_rx_dma_buf[i]);
  }
}

static void microros_uart_rx_dma_start(UART_HandleTypeDef* huart)
{
  g_rx_dma_last_pos = 0;
  (void)HAL_UARTEx_ReceiveToIdle_DMA(huart, g_rx_dma_buf, (uint16_t)MICROROS_UART_RX_DMA_BUFFER_SIZE);
}

static bool f446_uart_open(struct uxrCustomTransport* transport)
{
  // シグネチャ上必要だがUARTでは使わない（未使用引数の警告回避）。
  (void)transport;
  return true;
}

static bool f446_uart_close(struct uxrCustomTransport* transport)
{
  // シグネチャ上必要だがUARTでは使わない（未使用引数の警告回避）。
  (void)transport;
  return true;
}

static size_t f446_uart_write(
    struct uxrCustomTransport* transport,
    const uint8_t* buf,
    size_t len,
    uint8_t* err)
{
  UART_HandleTypeDef* huart = (UART_HandleTypeDef*)transport->args;
  size_t sent = 0;

  while (sent < len) {
    uint16_t chunk = (uint16_t)((len - sent) > 0xFFFFU ? 0xFFFFU : (len - sent));
    g_tx_dma_done = false;
    g_tx_dma_error = false;

    uint32_t launch_start = HAL_GetTick();
    HAL_StatusTypeDef launch_status;
    do {
      launch_status = HAL_UART_Transmit_DMA(huart, (uint8_t*)&buf[sent], chunk);
      if (launch_status == HAL_BUSY) {
        __WFI();
      }
    } while (launch_status == HAL_BUSY && (HAL_GetTick() - launch_start) < MICROROS_UART_TX_DMA_TIMEOUT_MS);

    if (launch_status != HAL_OK) {
      if (err != NULL) {
        *err = 1;
      }
      return sent;
    }

    uint32_t start = HAL_GetTick();
    while (!g_tx_dma_done && !g_tx_dma_error) {
      if ((HAL_GetTick() - start) >= MICROROS_UART_TX_DMA_TIMEOUT_MS) {
        (void)HAL_UART_AbortTransmit(huart);
        if (err != NULL) {
          *err = 1;
        }
        return sent;
      }
      __WFI();
    }

    if (g_tx_dma_error) {
      if (err != NULL) {
        *err = 1;
      }
      return sent;
    }

    sent += chunk;
  }

  if (err != NULL) {
    *err = 0;
  }
  return sent;
}

static size_t f446_uart_read(
    struct uxrCustomTransport* transport,
    uint8_t* buf,
    size_t len,
    int timeout_ms,
    uint8_t* err)
{
  (void)transport;
  size_t received = 0;

  // Some XRCE paths pass timeout_ms < 0 to indicate "wait for data".
  // Don't block indefinitely here, otherwise the main loop can stall.
  if (timeout_ms < 0) {
    timeout_ms = MICROROS_UART_READ_INFINITE_TIMEOUT_MS;
  }

  uint32_t start = HAL_GetTick();
  uint32_t last_rx = start;
  while (received < len) {
    uint32_t now = HAL_GetTick();
    uint32_t elapsed = now - start;
    if (elapsed >= (uint32_t)timeout_ms) {
      break;
    }

    uint8_t b = 0;
    bool got = false;
    uint32_t primask;
    crit_enter(&primask);
    got = rx_ring_pop(&b);
    crit_exit(primask);

    if (got) {
      buf[received] = b;
      received++;
      last_rx = now;
      continue;
    }

    if (received > 0U && (now - last_rx) >= MICROROS_UART_READ_INTERBYTE_TIMEOUT_MS) {
      break;
    }

    // Sleep until next interrupt (RX/SysTick/etc.) to avoid busy looping.
    __WFI();
  }

  if (err != NULL) {
    *err = 0;
  }
  return received;
}

void microros_transport_uart_init(UART_HandleTypeDef* huart)
{
  g_transport_uart = huart;
  g_rx_head = 0;
  g_rx_tail = 0;
  g_tx_dma_done = true;
  g_tx_dma_error = false;
  g_rx_dma_last_pos = 0;

  // Start DMA circular RX with IDLE line event callback.
  microros_uart_rx_dma_start(huart);

  rmw_uros_set_custom_transport(
      true,
      (void*)huart,
      f446_uart_open,
      f446_uart_close,
      f446_uart_write,
      f446_uart_read);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
{
  if (huart == g_transport_uart) {
    g_tx_dma_done = true;
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t size)
{
  if (huart != g_transport_uart) {
    return;
  }

  uint16_t curr = size;
  if (curr > MICROROS_UART_RX_DMA_BUFFER_SIZE) {
    curr = MICROROS_UART_RX_DMA_BUFFER_SIZE;
  }

  uint16_t prev = g_rx_dma_last_pos;
  if (curr > prev) {
    rx_dma_copy_into_ring(prev, curr);
  } else if (curr < prev) {
    rx_dma_copy_into_ring(prev, MICROROS_UART_RX_DMA_BUFFER_SIZE);
    rx_dma_copy_into_ring(0U, curr);
  }

  g_rx_dma_last_pos = (curr == MICROROS_UART_RX_DMA_BUFFER_SIZE) ? 0U : curr;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart)
{
  if (huart != g_transport_uart) {
    return;
  }

  g_tx_dma_error = true;

  __HAL_UART_CLEAR_OREFLAG(huart);
  __HAL_UART_CLEAR_NEFLAG(huart);
  __HAL_UART_CLEAR_FEFLAG(huart);
  __HAL_UART_CLEAR_PEFLAG(huart);

  microros_uart_rx_dma_start(huart);
}
