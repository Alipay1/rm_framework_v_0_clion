//
// Created by zhpwa on 2024/1/8.
//

#include "bsp_led.h"

static uint8_t state_b = 0, state_g = 0, state_r = 0;

void bsp_led_init(void) {
  HAL_TIM_Base_Start(&htim5);
  HAL_TIM_PWM_Start(&htim5, LED_BLUE);
  HAL_TIM_PWM_Start(&htim5, LED_GREEN);
  HAL_TIM_PWM_Start(&htim5, LED_RED);
}

void bsp_led_blink(LED_COLOR_e color) {
  __HAL_TIM_SET_COMPARE(&htim5, color, 255);
  osDelay(100);
  __HAL_TIM_SET_COMPARE(&htim5, color, 0);
  osDelay(100);
}

void bsp_led_toggle(LED_COLOR_e color) {
  uint8_t *state = NULL;
  switch (color) {
  case LED_BLUE:
    state = &state_b;
    break;
  case LED_GREEN:
    state = &state_g;
    break;
  default:
  case LED_RED:
    state = &state_r;
    break;
  }

  if (*state == 0) {
    __HAL_TIM_SET_COMPARE(&htim5, color, 255);
    *state = 1;
  } else if (*state == 1) {
    __HAL_TIM_SET_COMPARE(&htim5, color, 0);
    *state = 0;
  }
}