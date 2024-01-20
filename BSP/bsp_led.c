//
// Created by zhpwa on 2024/1/8.
//

#include "bsp_led.h"

typedef enum {
  LED_STATE_OFF = 0,
  LED_STATE_ON = 1
} private_bsp_led_state_e;

static uint8_t
	state_b = LED_STATE_OFF,
	state_g = LED_STATE_OFF,
	state_r = LED_STATE_OFF;

void bsp_led_init (void)
{
  HAL_TIM_Base_Start (&htim5);
  HAL_TIM_PWM_Start (&htim5, LED_BLUE);
  HAL_TIM_PWM_Start (&htim5, LED_GREEN);
  HAL_TIM_PWM_Start (&htim5, LED_RED);
}

void bsp_led_blink_os (LED_COLOR_e color)
{
  __HAL_TIM_SET_COMPARE(&htim5, color, 255);
  osDelay (100);
  __HAL_TIM_SET_COMPARE(&htim5, color, 0);
  osDelay (100);
}

/**
 * @brief set specific color brightness by 0~255
 *
 * @brief will set the color channel state to active
 *
 *
 *
 * */
void bsp_led_write (LED_COLOR_e color, uint8_t brightness)
{
  uint8_t *state = NULL;
  switch (color)
	{
	  case LED_BLUE:state = &state_b;
	  break;
	  case LED_GREEN:state = &state_g;
	  break;
	  default:
	  case LED_RED:state = &state_r;
	  break;
	}

  if (*state == LED_STATE_OFF)
	{
	  *state = LED_STATE_ON;
	}

  __HAL_TIM_SET_COMPARE(&htim5, color, brightness);

}

void bsp_led_toggle (LED_COLOR_e color)
{
  uint8_t *state = NULL;
  switch (color)
	{
	  case LED_BLUE:state = &state_b;
	  break;
	  case LED_GREEN:state = &state_g;
	  break;
	  default:
	  case LED_RED:state = &state_r;
	  break;
	}

  if (*state == 0)
	{
	  __HAL_TIM_SET_COMPARE(&htim5, color, 255);
	  *state = LED_STATE_ON;
	}
  else if (*state == 1)
	{
	  __HAL_TIM_SET_COMPARE(&htim5, color, 0);
	  *state = LED_STATE_OFF;
	}
}