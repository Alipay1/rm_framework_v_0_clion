//
// Created by zhpwa on 2024/1/8.
//

#ifndef C_BOARD_STANDARD_ROBOT_BSP_UART_H
#define C_BOARD_STANDARD_ROBOT_BSP_UART_H

#include "FreeRTOS.h"
#include "message_buffer.h"
#include "stdint.h"

#define MESSAGE_BUFFER_SIZE 1024

#define UART1_TX_CPT_TASK_NOTIFY_BIT (1 << 0)
#define UART6_TX_CPT_TASK_NOTIFY_BIT (1 << 6)

extern MessageBufferHandle_t UART1_MB_HANDLE;
extern MessageBufferHandle_t UART6_MB_HANDLE;

typedef enum {
  BSP_UART_OPT_SUCCESS = 0, // option success
  BSP_UART_OPT_FAIL = -1,   // option fail
  BSP_UART_FIFO_NULL_PTR = -2

} BSP_UART_r;

int bsp_uart_init(void);

#endif // C_BOARD_STANDARD_ROBOT_BSP_UART_H
