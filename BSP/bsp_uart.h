//
// Created by zhpwa on 2024/1/8.
//

#ifndef C_BOARD_STANDARD_ROBOT_BSP_UART_H
#define C_BOARD_STANDARD_ROBOT_BSP_UART_H

#include "FreeRTOS.h"
#include "message_buffer.h"

#include "stm32f4xx_hal.h"

#define MESSAGE_BUFFER_SIZE 1024

extern MessageBufferHandle_t UART1_MB_HANDLE;
extern MessageBufferHandle_t UART6_MB_HANDLE;
typedef enum {
  BSP_UART1 = 1, // option success
  BSP_UART6 = 6,   // option fail
} BSP_UART_e;
typedef enum {
  BSP_UART_OPT_SUCCESS = 0, // option success
  BSP_UART_OPT_FAIL = -1,   // option fail
  BSP_UART_FIFO_NULL_PTR = -2

} BSP_UART_r;

HAL_StatusTypeDef bsp_uart1_start_idle_dma_rx (void);
HAL_StatusTypeDef bsp_uart6_start_idle_dma_rx (void);
int bsp_uart_init (void);
int bsp_printf (BSP_UART_e UARTx, const char *fmt, ...);
char *bsp_get_uart1_rx_buf (void);
char *bsp_get_uart6_rx_buf (void);

#endif // C_BOARD_STANDARD_ROBOT_BSP_UART_H
