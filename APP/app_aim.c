#pragma clang diagnostic push
#pragma ide diagnostic ignored "Simplify"
/**
  ******************************************************************************
  * @file           : app_aim.c
  * @author         : zhpwa
  * @brief          : None
  * @attention      : None
  * @date           : 2024/3/16
  ******************************************************************************
  */


#include <stdbool.h>
#include "app_aim.h"
#include "usart.h"
#include "ins_task.h"
#include "bsp_check.h"

struct McuData mcu_data = {
	.curr_yaw = 0,
	.curr_pitch = 0,
	.state = 0,
	.mark = 0,
	.anti_top = 0,
	.enemy_color = 0,
	.delta_x = 0,
	.delta_y = 0
};

struct SendData send_data = {
	.start_flag = 's',
	.yaw = 0,
	.pitch = 0,
	.shoot_delay = 0,
	.end_flag = 'e'
};

static bool app_aim_init_flag = false;

/**
 * @brief 如果发生结构体尺寸等问题将进入此处
 */
_Noreturn void app_aim_init_fail (void)
{
  __disable_irq ();
  while (1);
}

/**
 * @brief must call before this app ever used
 * @attention 没有找到适用于arm-gcc的编译器检查宏，简陋代替
 */
void app_aim_init (void)
{
  if (sizeof (struct McuData) == 20 && sizeof (struct SendData) == 8)
	{
	  app_aim_init_flag = true;
	}
  else
	{
	  app_aim_init_fail ();
	}

}

void app_aim_send (void)
{
  send_data.pitch = (int16_t) (INS.Pitch * 0.01F * 32767.0F);/// float类型的实际角度(以度为单位)/100*(32768-1)
  send_data.yaw = (int16_t) (INS.YawTotalAngle * 0.01F * 32767.0F);/// float类型的实际角度(以度为单位)/100*(32768-1)
  send_data.shoot_delay = 0;

  send_data.start_flag = 's';
  send_data.end_flag = 'e';

  if (app_aim_init_flag)
	{
	  HAL_UART_Transmit (&APP_AIM_UART_HANDLE,
						 (const uint8_t *) &send_data,
						 sizeof (struct SendData),
						 1000);
	}
  else
	{
	  app_aim_init_fail ();
	}
}

void app_aim_decode (int rx_size)
{
  char *data = bsp_get_uart6_rx_buf ();

  if (rx_size != (sizeof (struct McuData) + 1))
	{
	  return;
	}

  for (int i = 0; i < sizeof (struct McuData); i++)
	{
	  ((char *) (&mcu_data))[i] = data[i];
	}
}

#pragma clang diagnostic pop