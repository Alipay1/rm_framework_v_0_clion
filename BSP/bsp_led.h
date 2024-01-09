//
// Created by zhpwa on 2024/1/8.
//

#ifndef C_BOARD_STANDARD_ROBOT_BSP_LED_H
#define C_BOARD_STANDARD_ROBOT_BSP_LED_H

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "main.h"
#include "task.h"

#include "tim.h"

typedef enum {
  LED_BLUE = TIM_CHANNEL_1,
  LED_GREEN = TIM_CHANNEL_2,
  LED_RED = TIM_CHANNEL_3
} LED_COLOR_e;

void bsp_led_init(void);
void bsp_led_blink(LED_COLOR_e color);
void bsp_led_toggle(LED_COLOR_e color);

#endif // C_BOARD_STANDARD_ROBOT_BSP_LED_H
