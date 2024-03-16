//
// Created by zhpwa on 2024/1/8.
//

#include "bsp_uart.h"
#include "stdarg.h"
#include "stddef.h"
#include "stdio.h"
#include "usart.h"

extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;

static uint8_t UART1_MESSAGE_BUFFER[MESSAGE_BUFFER_SIZE] = {0};
static StaticMessageBuffer_t UART1_SMB_STRUCT = {0};
MessageBufferHandle_t UART1_MB_HANDLE = {0};
static char bsp_uart1_rx_buf[MESSAGE_BUFFER_SIZE] = {0};

static uint8_t UART6_MESSAGE_BUFFER[MESSAGE_BUFFER_SIZE] = {0};
static StaticMessageBuffer_t UART6_SMB_STRUCT = {0};
MessageBufferHandle_t UART6_MB_HANDLE = {0};
static char bsp_uart6_rx_buf[MESSAGE_BUFFER_SIZE] = {0};

HAL_StatusTypeDef bsp_uart1_start_idle_dma_rx (void)
{
  HAL_StatusTypeDef ret = HAL_ERROR;
  ret = HAL_UARTEx_ReceiveToIdle_DMA (&huart1, (uint8_t *) bsp_uart1_rx_buf, MESSAGE_BUFFER_SIZE);
  __HAL_DMA_DISABLE_IT (&hdma_usart1_rx, DMA_IT_HT);
  return ret;
}

HAL_StatusTypeDef bsp_uart6_start_idle_dma_rx (void)
{
  HAL_StatusTypeDef ret = HAL_ERROR;
  ret = HAL_UARTEx_ReceiveToIdle_DMA (&huart6, (uint8_t *) bsp_uart6_rx_buf, MESSAGE_BUFFER_SIZE);
  __HAL_DMA_DISABLE_IT (&hdma_usart6_rx, DMA_IT_HT);
  return ret;
}

int bsp_uart_init (void)
{
  /*start idle dma receiving & disable some interrupt*/
  HAL_StatusTypeDef uart1_ret = HAL_ERROR;
  HAL_StatusTypeDef uart6_ret = HAL_ERROR;
  uart1_ret = bsp_uart1_start_idle_dma_rx ();
  uart6_ret = bsp_uart6_start_idle_dma_rx ();

  /*register the tx message buffer*/
  /*!MUST CALL BEFORE ANY TX OPERATIONS (exp. bsp_printf)*/
  UART1_MB_HANDLE = xMessageBufferCreateStatic(
	  MESSAGE_BUFFER_SIZE, UART1_MESSAGE_BUFFER, &UART1_SMB_STRUCT);
  UART6_MB_HANDLE = xMessageBufferCreateStatic(
	  MESSAGE_BUFFER_SIZE, UART6_MESSAGE_BUFFER, &UART6_SMB_STRUCT);

  /*if handles are empty or HAL_STATUS wrong then return fail*/
  if (!UART1_MB_HANDLE || !UART6_MB_HANDLE ||
	  uart1_ret != HAL_OK || uart6_ret != HAL_OK)
	{
	  return BSP_UART_OPT_FAIL;
	}
  return BSP_UART_OPT_SUCCESS;
}

int bsp_printf (BSP_UART_e UARTx, const char *fmt, ...)
{
  va_list args;
  static char buffer[MESSAGE_BUFFER_SIZE / 4]; // 临时缓冲区
  int written; // vsnprintf 返回写入的字符数
  MessageBufferHandle_t *private_message_buf_handle = NULL;
  UART_HandleTypeDef huart = huart1;
  switch (UARTx)
	{
	  default:
	  case BSP_CDC://todo end this

	  case BSP_UART1: private_message_buf_handle = &UART1_MB_HANDLE;
	  huart = huart1;
	  break;

	  case BSP_UART6: private_message_buf_handle = &UART6_MB_HANDLE;
	  huart = huart6;
	  break;
	}

  va_start(args, fmt);
  written = vsnprintf (buffer, MESSAGE_BUFFER_SIZE / 4, fmt, args);
  va_end(args);

  if (written > 0)
	{
	  while (xMessageBufferSend(*private_message_buf_handle, buffer, written, 0) != written);

//	  HAL_UART_Transmit (&huart, buffer, written, 0xFFFF);
	}
  return written;
}

char *bsp_get_uart1_rx_buf (void)
{
  return bsp_uart1_rx_buf;
}
char *bsp_get_uart6_rx_buf (void)
{
  return bsp_uart6_rx_buf;
}

void usart1_tx_dma_init (void)
{

  // enable the DMA transfer for the receiver and tramsmit request
  // 使能DMA串口接收和发送
  SET_BIT (huart1.Instance->CR3, USART_CR3_DMAR);
  SET_BIT (huart1.Instance->CR3, USART_CR3_DMAT);

  // disable DMA
  // 失效DMA
  __HAL_DMA_DISABLE (&hdma_usart1_tx);

  while (hdma_usart1_tx.Instance->CR & DMA_SxCR_EN)
	{
	  __HAL_DMA_DISABLE (&hdma_usart1_tx);
	}

  hdma_usart1_tx.Instance->PAR = (uint32_t) &(USART1->DR);
  hdma_usart1_tx.Instance->M0AR = (uint32_t) (NULL);
  hdma_usart1_tx.Instance->NDTR = 0;
}
void usart1_tx_dma_enable (uint8_t *data, uint16_t len)
{
  // disable DMA
  // 失效DMA
  __HAL_DMA_DISABLE (&hdma_usart1_tx);

  while (hdma_usart1_tx.Instance->CR & DMA_SxCR_EN)
	{
	  __HAL_DMA_DISABLE (&hdma_usart1_tx);
	}

  __HAL_DMA_CLEAR_FLAG (&hdma_usart1_tx, DMA_HISR_TCIF7);

  hdma_usart1_tx.Instance->M0AR = (uint32_t) (data);
  __HAL_DMA_SET_COUNTER (&hdma_usart1_tx, len);

  __HAL_DMA_ENABLE (&hdma_usart1_tx);
}
