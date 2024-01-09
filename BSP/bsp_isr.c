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

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  UBaseType_t uxSavedInterruptStatus;
  uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();

  if (huart->Instance == USART1) {
    extern osMutexId_t UART1_FIFO_muHandle; // from freertos.c
    extern osThreadId_t UART1TaskHandle;    // from freertos.c
    vTaskNotifyGiveFromISR(UART1TaskHandle, NULL);
    bsp_led_toggle(LED_RED);

    goto End_Of_HAL_UART_TxCpltCallback;
  }
  if (huart->Instance == USART6) {

    goto End_Of_HAL_UART_TxCpltCallback;
  }

End_Of_HAL_UART_TxCpltCallback:
  taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  UBaseType_t uxSavedInterruptStatus;
  uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();

  if (huart->Instance == USART3) {
    // todo add RC_DECODE
    goto End_Of_HAL_UARTEx_RxEventCallback;
  }
  if (huart->Instance == USART1) {

    goto End_Of_HAL_UARTEx_RxEventCallback;
  }
  if (huart->Instance == USART6) {

    goto End_Of_HAL_UARTEx_RxEventCallback;
  }

End_Of_HAL_UARTEx_RxEventCallback:
  taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
}