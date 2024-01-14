/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "can.h"
#include "crc.h"
#include "dma.h"
#include "gpio.h"
#include "rng.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"

#include "stdio.h"
#include <stdarg.h>

#include "bsp.h"

#include "app_pid.h"
#include "app_step_resp.h"

#include "event_groups.h"
#include "semphr.h"
#include "ins_task.h"
#include "app_rc.h"
#include "app_nav.h"

#pragma clang diagnostic push
#pragma ide diagnostic ignored "UnusedParameter"
#pragma ide diagnostic ignored "EndlessLoop"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticSemaphore_t osStaticMutexDef_t;
typedef StaticEventGroup_t osStaticEventGroupDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USE_STEP_RESPONSE 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
void print (char *fmt, ...)
{
  uint8_t buf[128];

  uint16_t len;
  va_list ap;
  va_start(ap, fmt);
  len = vsnprintf ((char *) buf, 128, fmt, ap);
  va_end(ap);

  // while (flag_u1 == 1) {
  // }
  // HAL_UART_Transmit_DMA(&huart1, buf, len);

  HAL_UART_Transmit (&huart1, buf, len, 1000);
  // HAL_UART_Transmit_DMA(&huart1, buf, len);
}
/* USER CODE END Variables */
/* Definitions for INSTask */
osThreadId_t INSTaskHandle;
uint32_t INSTaskBuffer[1024];
osStaticThreadDef_t INSTaskControlBlock;
const osThreadAttr_t INSTask_attributes = {
	.name = "INSTask",
	.cb_mem = &INSTaskControlBlock,
	.cb_size = sizeof (INSTaskControlBlock),
	.stack_mem = &INSTaskBuffer[0],
	.stack_size = sizeof (INSTaskBuffer),
	.priority = (osPriority_t) osPriorityLow,
};
/* Definitions for LEDTask */
osThreadId_t LEDTaskHandle;
uint32_t LEDTaslBuffer[128];
osStaticThreadDef_t LEDTaslControlBlock;
const osThreadAttr_t LEDTask_attributes = {
	.name = "LEDTask",
	.cb_mem = &LEDTaslControlBlock,
	.cb_size = sizeof (LEDTaslControlBlock),
	.stack_mem = &LEDTaslBuffer[0],
	.stack_size = sizeof (LEDTaslBuffer),
	.priority = (osPriority_t) osPriorityLow,
};
/* Definitions for BUZTask */
osThreadId_t BUZTaskHandle;
uint32_t BUZTaskBuffer[128];
osStaticThreadDef_t BUZTaskControlBlock;
const osThreadAttr_t BUZTask_attributes = {
	.name = "BUZTask",
	.cb_mem = &BUZTaskControlBlock,
	.cb_size = sizeof (BUZTaskControlBlock),
	.stack_mem = &BUZTaskBuffer[0],
	.stack_size = sizeof (BUZTaskBuffer),
	.priority = (osPriority_t) osPriorityLow,
};
/* Definitions for CANTask */
osThreadId_t CANTaskHandle;
uint32_t CANTaskBuffer[4096];
osStaticThreadDef_t CANTaskControlBlock;
const osThreadAttr_t CANTask_attributes = {
	.name = "CANTask",
	.cb_mem = &CANTaskControlBlock,
	.cb_size = sizeof (CANTaskControlBlock),
	.stack_mem = &CANTaskBuffer[0],
	.stack_size = sizeof (CANTaskBuffer),
	.priority = (osPriority_t) osPriorityLow,
};
/* Definitions for UART1Task */
osThreadId_t UART1TaskHandle;
uint32_t UART1TaskBuffer[1024];
osStaticThreadDef_t UART1TaskControlBlock;
const osThreadAttr_t UART1Task_attributes = {
	.name = "UART1Task",
	.cb_mem = &UART1TaskControlBlock,
	.cb_size = sizeof (UART1TaskControlBlock),
	.stack_mem = &UART1TaskBuffer[0],
	.stack_size = sizeof (UART1TaskBuffer),
	.priority = (osPriority_t) osPriorityLow,
};
/* Definitions for UART6Task */
osThreadId_t UART6TaskHandle;
uint32_t UART6TaskBuffer[1024];
osStaticThreadDef_t UART6TaskControlBlock;
const osThreadAttr_t UART6Task_attributes = {
	.name = "UART6Task",
	.cb_mem = &UART6TaskControlBlock,
	.cb_size = sizeof (UART6TaskControlBlock),
	.stack_mem = &UART6TaskBuffer[0],
	.stack_size = sizeof (UART6TaskBuffer),
	.priority = (osPriority_t) osPriorityLow,
};
/* Definitions for SERVOTask */
osThreadId_t SERVOTaskHandle;
uint32_t SERVOTaskBuffer[128];
osStaticThreadDef_t SERVOTaskControlBlock;
const osThreadAttr_t SERVOTask_attributes = {
	.name = "SERVOTask",
	.cb_mem = &SERVOTaskControlBlock,
	.cb_size = sizeof (SERVOTaskControlBlock),
	.stack_mem = &SERVOTaskBuffer[0],
	.stack_size = sizeof (SERVOTaskBuffer),
	.priority = (osPriority_t) osPriorityLow,
};
/* Definitions for STEP_RESPONSETa */
osThreadId_t STEP_RESPONSETaHandle;
const osThreadAttr_t STEP_RESPONSETa_attributes = {
	.name = "STEP_RESPONSETa",
	.stack_size = 256 * 4,
	.priority = (osPriority_t) osPriorityLow,
};
/* Definitions for UART6RxTask */
osThreadId_t UART6RxTaskHandle;
uint32_t UART6RxTaskBuffer[512];
osStaticThreadDef_t UART6RxTaskControlBlock;
const osThreadAttr_t UART6RxTask_attributes = {
	.name = "UART6RxTask",
	.cb_mem = &UART6RxTaskControlBlock,
	.cb_size = sizeof (UART6RxTaskControlBlock),
	.stack_mem = &UART6RxTaskBuffer[0],
	.stack_size = sizeof (UART6RxTaskBuffer),
	.priority = (osPriority_t) osPriorityLow,
};
/* Definitions for UART1RxTask */
osThreadId_t UART1RxTaskHandle;
uint32_t UART1RxTaskBuffer[512];
osStaticThreadDef_t UART1RxTaskControlBlock;
const osThreadAttr_t UART1RxTask_attributes = {
	.name = "UART1RxTask",
	.cb_mem = &UART1RxTaskControlBlock,
	.cb_size = sizeof (UART1RxTaskControlBlock),
	.stack_mem = &UART1RxTaskBuffer[0],
	.stack_size = sizeof (UART1RxTaskBuffer),
	.priority = (osPriority_t) osPriorityLow,
};
/* Definitions for MOTOR_TEMPTask */
osThreadId_t MOTOR_TEMPTaskHandle;
const osThreadAttr_t MOTOR_TEMPTask_attributes = {
	.name = "MOTOR_TEMPTask",
	.stack_size = 256 * 4,
	.priority = (osPriority_t) osPriorityLow,
};
/* Definitions for RunTimeStats */
osThreadId_t RunTimeStatsHandle;
const osThreadAttr_t RunTimeStats_attributes = {
	.name = "RunTimeStats",
	.stack_size = 512 * 4,
	.priority = (osPriority_t) osPriorityLow,
};
/* Definitions for LED_q */
osMessageQueueId_t LED_qHandle;
const osMessageQueueAttr_t LED_q_attributes = {
	.name = "LED_q"
};
/* Definitions for BUZ_q */
osMessageQueueId_t BUZ_qHandle;
const osMessageQueueAttr_t BUZ_q_attributes = {
	.name = "BUZ_q"
};
/* Definitions for SERVO_q */
osMessageQueueId_t SERVO_qHandle;
const osMessageQueueAttr_t SERVO_q_attributes = {
	.name = "SERVO_q"
};
/* Definitions for UART1_FIFO_mu */
osMutexId_t UART1_FIFO_muHandle;
osStaticMutexDef_t UART1_FIFO_muControlBlock;
const osMutexAttr_t UART1_FIFO_mu_attributes = {
	.name = "UART1_FIFO_mu",
	.cb_mem = &UART1_FIFO_muControlBlock,
	.cb_size = sizeof (UART1_FIFO_muControlBlock),
};
/* Definitions for UART6_FIFO_mu */
osMutexId_t UART6_FIFO_muHandle;
osStaticMutexDef_t UART6_FIFO_muControlBlock;
const osMutexAttr_t UART6_FIFO_mu_attributes = {
	.name = "UART6_FIFO_mu",
	.cb_mem = &UART6_FIFO_muControlBlock,
	.cb_size = sizeof (UART6_FIFO_muControlBlock),
};
/* Definitions for UART_FIFO_e */
osEventFlagsId_t UART_FIFO_eHandle;
osStaticEventGroupDef_t UART_FIFO_eControlBlock;
const osEventFlagsAttr_t UART_FIFO_e_attributes = {
	.name = "UART_FIFO_e",
	.cb_mem = &UART_FIFO_eControlBlock,
	.cb_size = sizeof (UART_FIFO_eControlBlock),
};
/* Definitions for STEP_RESPON_GLOBAL_VARIABLE */
osEventFlagsId_t STEP_RESPON_GLOBAL_VARIABLEHandle;
osStaticEventGroupDef_t STEP_RESPON_GLOBAL_VARIABLEControlBlock;
const osEventFlagsAttr_t STEP_RESPON_GLOBAL_VARIABLE_attributes = {
	.name = "STEP_RESPON_GLOBAL_VARIABLE",
	.cb_mem = &STEP_RESPON_GLOBAL_VARIABLEControlBlock,
	.cb_size = sizeof (STEP_RESPON_GLOBAL_VARIABLEControlBlock),
};
/* Definitions for Key_e */
osEventFlagsId_t Key_eHandle;
const osEventFlagsAttr_t Key_e_attributes = {
	.name = "Key_e"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartINSTask (void *argument);
void StartLEDTask (void *argument);
void StartBUZTask (void *argument);
void StartCANTask (void *argument);
void StartUART1Task (void *argument);
void StartUART6Task (void *argument);
void StartSERVOTask (void *argument);
void StartSTEP_RESPONSETask (void *argument);
void StartUART6RxTask (void *argument);
void StartUART1RxTask (void *argument);
void StartMOTOR_TEMPTask (void *argument);
void StartTask12 (void *argument);

void MX_FREERTOS_Init (void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void configureTimerForRunTimeStats (void);
unsigned long getRunTimeCounterValue (void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
extern uint32_t g_osRuntimeCounter;/*defined in main.c(CCMRAM)*/
__weak void configureTimerForRunTimeStats (void)
{
  HAL_TIM_Base_Start_IT (&htim7);
  g_osRuntimeCounter = 0;
}

__weak unsigned long getRunTimeCounterValue (void)
{ return g_osRuntimeCounter; }
/* USER CODE END 1 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init (void)
{
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of UART1_FIFO_mu */
  UART1_FIFO_muHandle = osMutexNew (&UART1_FIFO_mu_attributes);

  /* creation of UART6_FIFO_mu */
  UART6_FIFO_muHandle = osMutexNew (&UART6_FIFO_mu_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of LED_q */
  LED_qHandle = osMessageQueueNew (16, sizeof (uint16_t), &LED_q_attributes);

  /* creation of BUZ_q */
  BUZ_qHandle = osMessageQueueNew (16, sizeof (uint16_t), &BUZ_q_attributes);

  /* creation of SERVO_q */
  SERVO_qHandle = osMessageQueueNew (16, sizeof (uint16_t), &SERVO_q_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of INSTask */
  INSTaskHandle = osThreadNew (StartINSTask, NULL, &INSTask_attributes);

  /* creation of LEDTask */
  LEDTaskHandle = osThreadNew (StartLEDTask, NULL, &LEDTask_attributes);

  /* creation of BUZTask */
  BUZTaskHandle = osThreadNew (StartBUZTask, NULL, &BUZTask_attributes);

  /* creation of CANTask */
  CANTaskHandle = osThreadNew (StartCANTask, NULL, &CANTask_attributes);

  /* creation of UART1Task */
  UART1TaskHandle = osThreadNew (StartUART1Task, NULL, &UART1Task_attributes);

  /* creation of UART6Task */
  UART6TaskHandle = osThreadNew (StartUART6Task, NULL, &UART6Task_attributes);

  /* creation of SERVOTask */
  SERVOTaskHandle = osThreadNew (StartSERVOTask, NULL, &SERVOTask_attributes);

  /* creation of STEP_RESPONSETa */
  STEP_RESPONSETaHandle = osThreadNew (StartSTEP_RESPONSETask, NULL, &STEP_RESPONSETa_attributes);

  /* creation of UART6RxTask */
  UART6RxTaskHandle = osThreadNew (StartUART6RxTask, NULL, &UART6RxTask_attributes);

  /* creation of UART1RxTask */
  UART1RxTaskHandle = osThreadNew (StartUART1RxTask, NULL, &UART1RxTask_attributes);

  /* creation of MOTOR_TEMPTask */
  MOTOR_TEMPTaskHandle = osThreadNew (StartMOTOR_TEMPTask, NULL, &MOTOR_TEMPTask_attributes);

  /* creation of RunTimeStats */
  RunTimeStatsHandle = osThreadNew (StartTask12, NULL, &RunTimeStats_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* creation of UART_FIFO_e */
  UART_FIFO_eHandle = osEventFlagsNew (&UART_FIFO_e_attributes);

  /* creation of STEP_RESPON_GLOBAL_VARIABLE */
  STEP_RESPON_GLOBAL_VARIABLEHandle = osEventFlagsNew (&STEP_RESPON_GLOBAL_VARIABLE_attributes);

  /* creation of Key_e */
  Key_eHandle = osEventFlagsNew (&Key_e_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartINSTask */
/**
 * @brief  Function implementing the INSTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartINSTask */
void StartINSTask (void *argument)
{
  /* USER CODE BEGIN StartINSTask */

  /* Infinite loop */
  for (;;)
	{
	  INS_Task ();
	  osDelay (1);
	}
  /* USER CODE END StartINSTask */
}

/* USER CODE BEGIN Header_StartLEDTask */
/**
 * @brief Function implementing the LEDTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartLEDTask */
void StartLEDTask (void *argument)
{
  /* USER CODE BEGIN StartLEDTask */
  /* Infinite loop */
  for (;;)
	{


//	  bsp_led_blink (LED_RED);
	  bsp_led_blink (LED_GREEN);
//	  bsp_led_blink (LED_BLUE);
//	  bsp_led_toggle (LED_GREEN);
	  osDelay (pdMS_TO_TICKS(1000));
	}
  /* USER CODE END StartLEDTask */
}

/* USER CODE BEGIN Header_StartBUZTask */
/**
 * @brief Function implementing the BUZTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartBUZTask */
void StartBUZTask (void *argument)
{
  /* USER CODE BEGIN StartBUZTask */
  bsp_buz_set_major (BSP_BUZ_MAJOR_B);
  /* Infinite loop */
  for (;;)
	{
//	  bsp_buz_set_pitch (BSP_BUZ_TONE_MI);
//	  osDelay (666);
//	  bsp_buz_set_pitch (BSP_BUZ_TONE_RE);
//	  osDelay (333);
//	  bsp_buz_set_pitch (BSP_BUZ_TONE_DO);
//	  osDelay (666);
//
//	  bsp_buz_set_pitch (BSP_BUZ_TONE_RE);
//	  osDelay (166);
//	  bsp_buz_set_pitch (BSP_BUZ_TONE_MI);
//	  osDelay (333);
//
//	  bsp_buz_set_pitch (BSP_BUZ_TONE_FA);
//	  osDelay (333);
//	  bsp_buz_set_pitch (BSP_BUZ_TONE_MI);
//	  osDelay (166);
//	  bsp_buz_set_pitch (BSP_BUZ_TONE_RE);
//	  osDelay (333 * 2);

//	  bsp_buz_set_pitch (BSP_BUZ_TONE_RE);

//	  bsp_buz_start ();
//	  bsp_buz_apply_frequency (77);
//	  osDelay (400);
//	  bsp_buz_apply_frequency (83);
//	  osDelay (550);
	  bsp_buz_mute ();
	  osDelay (1000);

	}
  /* USER CODE END StartBUZTask */
}

/* USER CODE BEGIN Header_StartCANTask */
/**
 * @brief Function implementing the CANTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartCANTask */
void StartCANTask (void *argument)
{
  /* USER CODE BEGIN StartCANTask */

  TickType_t xLastWakeUpTime = xTaskGetTickCount ();
  char buf[128] = {0};
  int str_len = 0;

  PID_Setup ();

  PID *wheel0 = pid_get_struct_pointer (0, NORMAL_MOTOR);
  PID *wheel1 = pid_get_struct_pointer (1, NORMAL_MOTOR);
  PID *wheel2 = pid_get_struct_pointer (2, NORMAL_MOTOR);
  PID *wheel3 = pid_get_struct_pointer (3, NORMAL_MOTOR);
  motor_measure_t *motor0 = get_measure_pointer (0);
  motor_measure_t *motor1 = get_measure_pointer (1);
  motor_measure_t *motor2 = get_measure_pointer (2);
  motor_measure_t *motor3 = get_measure_pointer (3);

  extern RC_ctrl_t rc_ctrl;

  RC_ctrl_t *rc = get_remote_control_point ();

  PID *servo0_pos = pid_get_struct_pointer (4 + 4, NORMAL_MOTOR);/*4 + 4 indicates motor5(4) -> PID -> position(+4)*/
  PID *servo0_spd = pid_get_struct_pointer (4, NORMAL_MOTOR);/*4 + 4 indicates motor5(4) -> PID -> position(+4)*/
  motor_measure_t *motor4 = get_measure_pointer (4);

  PID *servo1_pos = pid_get_struct_pointer (5 + 4, NORMAL_MOTOR);/*4 + 4 indicates motor5(4) -> PID -> position(+4)*/
  PID *servo1_spd = pid_get_struct_pointer (5, NORMAL_MOTOR);/*4 + 4 indicates motor5(4) -> PID -> position(+4)*/
  motor_measure_t *motor5 = get_measure_pointer (5);

  PID *servo2_pos = pid_get_struct_pointer (6 + 4, NORMAL_MOTOR);/*4 + 4 indicates motor5(4) -> PID -> position(+4)*/
  PID *servo2_spd = pid_get_struct_pointer (6, NORMAL_MOTOR);/*4 + 4 indicates motor5(4) -> PID -> position(+4)*/
  motor_measure_t *motor6 = get_measure_pointer (6);

  PID *servo3_pos = pid_get_struct_pointer (7 + 4, NORMAL_MOTOR);/*4 + 4 indicates motor5(4) -> PID -> position(+4)*/
  PID *servo3_spd = pid_get_struct_pointer (7, NORMAL_MOTOR);/*4 + 4 indicates motor5(4) -> PID -> position(+4)*/
  motor_measure_t *motor7 = get_measure_pointer (7);

  app_nav_t *nav = get_navigation_p ();

  /* Infinite loop */
  for (;;)
	{

	  servo0_pos->ideal = rc_ctrl.rc.ch[0] * 0.1F;
	  servo1_pos->ideal = rc_ctrl.rc.ch[1] * 0.05F;
//	  servo2_pos->ideal = get_bsp_pid_step_response_target ();
//	  servo3_pos->ideal = get_bsp_pid_step_response_target ();


	  if (rc_ctrl.rc.s[0] == 3)
		{
		  wheel0->ideal = -6000;
		  wheel1->ideal = 6000;
		}
	  else
		{
		  wheel0->ideal = 0;
		  wheel1->ideal = 0;
		}
//	  wheel2->ideal = get_navi_struct_p ()->vector.velocity;
//	  wheel3->ideal = get_navi_struct_p ()->vector.velocity;


	  PID_Calculate_seper (servo0_spd, servo0_pos);
	  PID_Calculate_seper (servo1_spd, servo1_pos);

	  PID_Calculate_single (wheel0);
	  PID_Calculate_single (wheel1);
	  PID_Calculate_single (wheel2);
	  PID_Calculate_single (wheel3);

//	  PID_Calculate_seper (&pid_servo0.spd, &pid_servo0.pos);
//	  PID_Calculate_seper (&pid_servo1.spd, &pid_servo1.pos);
//	  PID_Calculate_seper (&pid_servo2.spd, &pid_servo2.pos);
//	  PID_Calculate_seper (&pid_servo3.spd, &pid_servo3.pos);

//	  bsp_printf (BSP_UART6, "%f,%f,%f\r\n", INS.YawTotalAngle, INS.Pitch, INS.Roll);
//	  bsp_printf (BSP_UART6, "psc:%d\r\narr:%d\r\n", htim4.Instance->PSC, htim4.Instance->ARR);
//	  bsp_printf (BSP_UART6, "tone:%d\r\n", bsp_buz_set_pitch (BSP_BUZ_TONE_DO));
//	  bsp_printf (BSP_UART6, "addr:%p\r\n", wheel0);
//	  bsp_printf (BSP_UART6, "total_ecd:%d\r\n", motor0_pos->total_ecd);
//	  bsp_printf (BSP_UART6, "run:%d\r\nkp:%f\r\nki:%f\r\nkd:%f\r\n", wheel0->active ? 1 : 0,
//				  wheel0->Kp, wheel0->Ki, wheel0->Kd);
//	  bsp_printf (BSP_UART6, "%f,%f,%f,%f,%f,%f,%d\r\n",
//				  servo0_pos->ideal,
//				  servo0_pos->actual,
//				  servo0_pos->output,
//				  servo0_spd->ideal,
//				  servo0_spd->actual,
//				  servo0_spd->output,
//				  motor0->total_ecd);
//	  print ("%f,%f,%f,%f,%f,%f,%d\r\n",
//			 servo2_pos->ideal,
//			 servo2_pos->actual,
//			 servo2_pos->output,
//			 servo2_spd->ideal,
//			 servo2_spd->actual,
//			 servo2_spd->output,
//			 motor2->total_ecd);
//	  print ("%f,%f,%f,%f\r\n",
//			 servo0_spd->output,
//			 servo1_spd->output,
//			 servo2_spd->output,
//			 servo3_spd->output);
//	  bsp_printf (BSP_UART1, "%f,%f,%f,%f\r\n",
//				  servo0_pos->actual,
//				  servo0_pos->ideal,
//				  servo1_pos->actual,
//				  servo1_pos->ideal);
//	  print ("%f,%f,%f,%f,%f,%f\r\n",
//			 servo1_spd->actual,
//			 servo1_spd->ideal,
//			 servo1_spd->output,
//			 servo1_pos->actual,
//			 servo1_pos->ideal,
//			 servo1_pos->output);
	  print ("%f,%f,%f,%f,%f,%f\r\n",
			 servo0_spd->actual,
			 servo0_spd->ideal,
			 servo0_spd->output,
			 servo0_pos->actual,
			 servo0_pos->ideal,
			 servo0_pos->output);
//	  bsp_printf (BSP_UART6, "%d,%d\r\n",
//				  rc_ctrl.rc.ch[0],
//				  rc_ctrl.rc.ch[1]);
//	  print ("%f,%f,%f,%f\r\n",
//			 pid_servo0.pos.actual,
//			 pid_servo0.pos.actual,
//			 pid_servo0.pos.actual,
//			 pid_servo0.pos.actual);
//	  print ("%f,%f,%f,%f\r\n",
//			 pid_servo0.spd.output,
//			 pid_servo1.spd.output,
//			 pid_servo2.spd.output,
//			 pid_servo3.spd.output);
//	  print ("%f,%f\r\n",
//			 wheel2->ideal,
//			 wheel2->actual);
//	  print ("%f,%f,%f,%f\r\n",
//			 wheel0->actual,
//			 wheel1->actual,
//			 wheel2->actual,
//			 wheel3->actual);
//	  print ("%f,%f,%f,%f\r\n",
//			 wheel0->output,
//			 wheel1->output,
//			 wheel2->output,
//			 wheel3->output);
//	  bsp_printf (BSP_UART1, "%f,%f,%f,%f\r\n",
//				  motor0->ecd,
//				  motor1->ecd,
//				  motor2->ecd,
//				  motor3->ecd);
//	  print ("%d,%d,%d,%d\r\n",
//			 motor4->ecd,
//			 motor5->ecd,
//			 motor6->ecd,
//			 motor7->ecd);



	  CAN_SendMessage (CAN_CHANNEL_1, MOTOR_1234,
					   (int16_t) wheel0->output,
					   (int16_t) wheel1->output,
					   (int16_t) wheel2->output,
					   0);
//	  CAN_SendMessage (CAN_CHANNEL_2,
//					   MOTOR_5678,
//					   (int16_t) servo0_spd->output,
//					   (int16_t) servo1_spd->output,
//					   0,
//					   0);
	  CAN_SendMessage (CAN_CHANNEL_1,
					   MOTOR_5678,
					   (int16_t) servo1_spd->output,
					   (int16_t) servo0_spd->output,
					   0,
					   0);

	  vTaskDelayUntil (&xLastWakeUpTime, 1);
	}
  /* USER CODE END StartCANTask */
}

/* USER CODE BEGIN Header_StartUART1Task */
/**
 * @brief Function implementing the UART1Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartUART1Task */
void StartUART1Task (void *argument)
{
  /* USER CODE BEGIN StartUART1Task */

  /*private variables*/
  char rx_data[MESSAGE_BUFFER_SIZE] = {0};
  uint32_t rx_size = 0;

  /* Infinite loop */
  for (;;)
	{
	  /*wait for data coming from message buffer*/
	  rx_size = xMessageBufferReceive(UART1_MB_HANDLE,
									  rx_data,
									  MESSAGE_BUFFER_SIZE,
									  portMAX_DELAY);

	  /*if received data*/
	  if (rx_size)
		{

		  /*take the mutex to prevent other IO operations*/
		  if (xSemaphoreTake(UART1_FIFO_muHandle, portMAX_DELAY) == pdTRUE)
			{
			  HAL_UART_Transmit_DMA (&huart1, (uint8_t *) rx_data, rx_size);
			}

		  /*wait for TaskNotify from bsp_isr.c HAL_UART_TxCpltCallback of UART1 to acknowledge dma transfer done*/
		  if (xTaskNotifyWait (0,
							   UINT32_MAX,
							   NULL,
							   portMAX_DELAY) == pdPASS)
			{
			  xSemaphoreGive(UART1_FIFO_muHandle);
			}
		}
	  //    bsp_led_toggle(LED_GREEN);  //redundant led toggle for quick running state check
	}
  /* USER CODE END StartUART1Task */
}

/* USER CODE BEGIN Header_StartUART6Task */
/**
 * @brief Function implementing the UART6Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartUART6Task */
void StartUART6Task (void *argument)
{
  /* USER CODE BEGIN StartUART6Task */
  char rx_data[MESSAGE_BUFFER_SIZE] = {0};
  uint32_t rx_size = 0;

  /* Infinite loop */
  for (;;)
	{
	  /*wait for data coming from message buffer*/
	  rx_size = xMessageBufferReceive(UART6_MB_HANDLE,
									  rx_data,
									  MESSAGE_BUFFER_SIZE,
									  portMAX_DELAY);

	  /*if received data*/
	  if (rx_size)
		{

		  /*take the mutex to prevent other IO operations*/
		  if (xSemaphoreTake(UART6_FIFO_muHandle, portMAX_DELAY) == pdTRUE)
			{
			  HAL_UART_Transmit_DMA (&huart6, (uint8_t *) rx_data, rx_size);
			}

		  /*wait for TaskNotify from bsp_isr.c HAL_UART_TxCpltCallback of UART1 to acknowledge dma transfer done*/
		  if (xTaskNotifyWait (0,
							   UINT32_MAX,
							   NULL,
							   portMAX_DELAY) == pdPASS)
			{
			  xSemaphoreGive(UART6_FIFO_muHandle);
			}
		}
	  //    bsp_led_toggle(LED_GREEN);  //redundant led toggle for quick running state check
	}
  /* USER CODE END StartUART6Task */
}

/* USER CODE BEGIN Header_StartSERVOTask */
/**
 * @brief Function implementing the SERVOTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartSERVOTask */
void StartSERVOTask (void *argument)
{
  /* USER CODE BEGIN StartSERVOTask */

  /* Infinite loop */
  for (;;)
	{
	  osDelay (1);
	}
  /* USER CODE END StartSERVOTask */
}

/* USER CODE BEGIN Header_StartSTEP_RESPONSETask */
/**
* @brief Function implementing the STEP_RESPONSETa thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSTEP_RESPONSETask */
void StartSTEP_RESPONSETask (void *argument)
{
  /* USER CODE BEGIN StartSTEP_RESPONSETask */
  TickType_t xLastWakeUpTime = xTaskGetTickCount ();

#if USE_STEP_RESPONSE == 0 /*defined in freertos.c*/
  /*if not in step response mode then delete self*/
  vTaskDelete (NULL);
#endif

  /* Infinite loop */
  for (;;)
	{

	  set_bsp_pid_step_response_target (100);
	  vTaskDelayUntil (&xLastWakeUpTime, 1000);
	  set_bsp_pid_step_response_target (-100);
	  vTaskDelayUntil (&xLastWakeUpTime, 1000);
	}
  /* USER CODE END StartSTEP_RESPONSETask */
}

/* USER CODE BEGIN Header_StartUART6RxTask */
/**
* @brief Function implementing the UART6RxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUART6RxTask */
void StartUART6RxTask (void *argument)
{
  /* USER CODE BEGIN StartUART6RxTask */
  /* Infinite loop */
  for (;;)
	{
	  if (xTaskNotifyWait (0,
						   UINT32_MAX,
						   NULL,
						   portMAX_DELAY) == pdPASS)
		{
//		  pid_sscanf (bsp_get_uart6_rx_buf ());
//		  bsp_led_toggle (LED_GREEN);
		}
	}
  /* USER CODE END StartUART6RxTask */
}

/* USER CODE BEGIN Header_StartUART1RxTask */
/**
* @brief Function implementing the UART1RxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUART1RxTask */
void StartUART1RxTask (void *argument)
{
  /* USER CODE BEGIN StartUART1RxTask */
  /* Infinite loop */
  for (;;)
	{
	  if (xTaskNotifyWait (0,
						   UINT32_MAX,
						   NULL,
						   portMAX_DELAY) == pdPASS)
		{
//		  pid_sscanf (bsp_get_uart6_rx_buf ());
		}
	}
  /* USER CODE END StartUART1RxTask */
}

/* USER CODE BEGIN Header_StartMOTOR_TEMPTask */
/**
* @brief Function implementing the MOTOR_TEMPTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMOTOR_TEMPTask */
void StartMOTOR_TEMPTask (void *argument)
{
  /* USER CODE BEGIN StartMOTOR_TEMPTask */
  motor_measure_t *motor = get_measure_pointer (0);
  uint8_t temperature[BSP_CAN_TOTAL_MORTOR_COUNT] = {0};
  uint8_t temp = 0;
  /* Infinite loop */
  for (;;)
	{
	  /*reset temp to 0*/
	  temp = 0;

	  /*collect temperature together*/
	  for (int i = 0; i < BSP_CAN_TOTAL_MORTOR_COUNT;)
		{
		  temperature[i] = (motor + i)->temperate;
		  i++;
		}

	  /*find max temperature*/
	  for (int i = 0; i < BSP_CAN_TOTAL_MORTOR_COUNT;)
		{
		  if (temperature[i] > temp)
			{
			  temp = temperature[i];
			}
		  i++;
		}

	  /*configMOTOR_ALARM_TEMPERATURE defined in main.h*/
	  if (temp >= configMOTOR_ALARM_TEMPERATURE)
		{
		  //todo call somthing safer to alarm
		  __disable_irq ();
		  while (1);
		}

	  osDelay (100);
	}
  /* USER CODE END StartMOTOR_TEMPTask */
}

/* USER CODE BEGIN Header_StartTask12 */
/**
* @brief Function implementing the RunTimeStats thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask12 */
void StartTask12 (void *argument)
{
  /* USER CODE BEGIN StartTask12 */
  static char InfoBuffer[512] = {0};
  /* Infinite loop */
  for (;;)
	{
	  vTaskGetRunTimeStats ((char *) &InfoBuffer);
//	  bsp_printf (BSP_UART1, "\r\n任务       运行计数         使用率\r\n");
//	  bsp_printf (BSP_UART1, "\r\n%s\r\n", InfoBuffer);
	  osDelay (1000);
	}
  /* USER CODE END StartTask12 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

#pragma clang diagnostic pop
/* USER CODE END Application */

