//
// Created by zhpwa on 2024/1/9.
//

#include "bsp_isr.h"
#include "bsp.h"

#include "FreeRTOS.h"
#include "semphr.h"

#include "can.h"
#include "cmsis_os.h"
#include "crc.h"
#include "dma.h"
#include "gpio.h"
#include "main.h"
#include "rng.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"

#include "app_pid.h"

extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart6_rx;

void HAL_UART_TxCpltCallback (UART_HandleTypeDef *huart)
{
  UBaseType_t uxSavedInterruptStatus;
  uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();

  if (huart->Instance == USART1)
	{
	  extern osThreadId_t UART1TaskHandle;    // from freertos.c
	  vTaskNotifyGiveFromISR (UART1TaskHandle, NULL);
//	  bsp_led_toggle (LED_RED);

	  goto End_Of_HAL_UART_TxCpltCallback;
	}
  if (huart->Instance == USART6)
	{
	  extern osThreadId_t UART6TaskHandle;    // from freertos.c
	  vTaskNotifyGiveFromISR (UART6TaskHandle, NULL);
//	  bsp_led_toggle (LED_RED);
	  goto End_Of_HAL_UART_TxCpltCallback;
	}

  End_Of_HAL_UART_TxCpltCallback:
  taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
}

void HAL_UARTEx_RxEventCallback (UART_HandleTypeDef *huart, uint16_t Size)
{
  UBaseType_t uxSavedInterruptStatus;
  uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();

  if (huart->Instance == USART1)
	{
	  extern osThreadId_t UART1RxTaskHandle;    // from freertos.c

	  vTaskNotifyGiveFromISR (UART1RxTaskHandle, NULL);
	  
	  pid_sscanf (bsp_get_uart1_rx_buf ());
	  bsp_led_toggle (LED_GREEN);

	  bsp_uart1_start_idle_dma_rx ();
	  goto End_Of_HAL_UARTEx_RxEventCallback;
	}
  if (huart->Instance == USART6)
	{
	  extern osThreadId_t UART6RxTaskHandle;    // from freertos.c
	  vTaskNotifyGiveFromISR (UART6RxTaskHandle, NULL);

	  pid_sscanf (bsp_get_uart6_rx_buf ());
	  bsp_led_toggle (LED_GREEN);

	  bsp_uart6_start_idle_dma_rx ();
	  goto End_Of_HAL_UARTEx_RxEventCallback;
	}

  End_Of_HAL_UARTEx_RxEventCallback:
  taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
}

void HAL_CAN_RxFifo0MsgPendingCallback (CAN_HandleTypeDef *hcan)
{
  static uint8_t rx_data[8];
  CAN_RxHeaderTypeDef rx_header;

  HAL_CAN_GetRxMessage (hcan, CAN_RX_FIFO0, &rx_header, rx_data);

  if (hcan->Instance == CAN1)
	{
	  switch (rx_header.StdId)
		{

		  case CAN_3508_M1_ID:
		  case CAN_3508_M2_ID:
		  case CAN_3508_M3_ID:
		  case CAN_3508_M4_ID:
			{
			  int i = rx_header.StdId - CAN_3508_M1_ID;
			  bsp_can_get_motor_measure (get_measure_pointer (i),
										 rx_data);
			  bsp_can_updateTotalAngle (i);

//			  bsp_led_toggle (LED_BLUE);
			  break;
			}

		  default:
			{
			  break;
			}
		}
	}

  if (hcan->Instance == CAN2)
	{
	  switch (rx_header.StdId)
		{
		  case CAN_YAW_MOTOR_ID:
		  case CAN_PIT_MOTOR_ID:
		  case 0x207:
		  case 0x208:
//		  case CAN_TRIGGER_MOTOR_ID:
			{
			  int i = rx_header.StdId - CAN_3508_M1_ID;
			  bsp_can_get_motor_measure (get_measure_pointer (i),
										 rx_data);
			  bsp_can_updateTotalAngle (i);

//			  bsp_led_toggle (LED_BLUE);
			  break;
			}

		  default:
			{
			  break;
			}
		}
	}
}