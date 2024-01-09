//
// Created by zhpwa on 2024/1/8.
//

#include "bsp_uart.h"
#include "stdarg.h"
#include "stddef.h"
#include "stdio.h"

static uint8_t UART1_MESSAGE_BUFFER[MESSAGE_BUFFER_SIZE] = {0};
static StaticMessageBuffer_t UART1_SMB_STRUCT = {0};
MessageBufferHandle_t UART1_MB_HANDLE = {0};

static uint8_t UART6_MESSAGE_BUFFER[MESSAGE_BUFFER_SIZE] = {0};
static StaticMessageBuffer_t UART6_SMB_STRUCT = {0};
MessageBufferHandle_t UART6_MB_HANDLE = {0};

int bsp_uart_init(void) {
  UART1_MB_HANDLE = xMessageBufferCreateStatic(
      MESSAGE_BUFFER_SIZE, UART1_MESSAGE_BUFFER, &UART1_SMB_STRUCT);
  UART6_MB_HANDLE = xMessageBufferCreateStatic(
      MESSAGE_BUFFER_SIZE, UART6_MESSAGE_BUFFER, &UART6_SMB_STRUCT);

  if (!UART1_MB_HANDLE || !UART6_MB_HANDLE) {
    return BSP_UART_OPT_FAIL;
  }
  return BSP_UART_OPT_SUCCESS;
}

int bsp_u1_printf(const char *fmt, ...) {
  va_list args;
  static char buffer[MESSAGE_BUFFER_SIZE / 4]; // 临时缓冲区
  int written; // vsnprintf 返回写入的字符数

  // 初始化变量参数列表
  va_start(args, fmt);

  // 将数据格式化输出到缓冲区, 保留一个字符的空间用于 '\0'
  written = vsnprintf(buffer, MESSAGE_BUFFER_SIZE / 4, fmt, args);

  // 清理变量参数列表
  va_end(args);

  if (written > 0) {
    // 如果有数据被写入到buffer中
    if (xMessageBufferSend(UART1_MB_HANDLE, buffer, written, 1) != written) {
      return -1;
    }
    // 也许你还需要处理FIFO已满或者其他错误的情况
  }
  return written;
}

// static UART_FIFO_t UART1_FIFO = {0};
// static UART_FIFO_t UART6_FIFO = {0};
//
// uint32_t bsp_get_uart_fifo_len(void) { return CONFIG_UART_FIFO_LENTH; }
//
// UART_FIFO_t *bsp_get_uart_fifo_p(int uart_num) {
//   switch (uart_num) {
//   case 1:
//     return &UART1_FIFO;
//   case 6:
//     return &UART6_FIFO;
//   default:
//     return NULL;
//   }
// }
//
// uint32_t bsp_get_uart_fifo_head(UART_FIFO_t *FIFO) { return FIFO->head; }
// uint32_t bsp_get_uart_fifo_tail(UART_FIFO_t *FIFO) { return FIFO->tail; }
// uint32_t bsp_get_uart_fifo_cnt(UART_FIFO_t *FIFO) { return FIFO->count; }
//
// int bsp_uart_fifo_write(UART_FIFO_t *FIFO, char src) {
//   if (FIFO->count >= CONFIG_UART_FIFO_LENTH) {
//     return BSP_UART_FIFO_FULL;
//   }
//
//   FIFO->buffer[FIFO->tail] = src;
//   FIFO->tail++;
//   FIFO->count++;
//   return BSP_UART_FIFO_WRITE_SUCCESS;
// }
//
// int bsp_uart_fifo_write_mult(UART_FIFO_t *FIFO, const char *data, int
// len) {
//   if (!FIFO || !data)
//     return BSP_UART_FIFO_NULL_PTR; // Check for NULL pointers
//
//   int bytesWritten = 0;
//
//   // Loop over each byte of data
//   for (int i = 0; i < len; i++) {
//     // Check if the FIFO is full
//     if (FIFO->count == CONFIG_UART_FIFO_LENTH) {
//       break; // FIFO is full, stop writing
//     }
//
//     // Write the data byte into the FIFO
//     FIFO->buffer[FIFO->tail] = data[i];
//
//     // Move the tail pointer forward
//     FIFO->tail = (FIFO->tail + 1) % CONFIG_UART_FIFO_LENTH;
//
//     // Increment the count of items in the FIFO
//     FIFO->count++;
//
//     // Increment the number of bytes written
//     bytesWritten++;
//   }
//
//   return bytesWritten; // Return the number of bytes written
// }