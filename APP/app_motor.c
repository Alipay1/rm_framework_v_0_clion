/**
  ******************************************************************************
  * @file           : app_motor.c
  * @author         : zhpwa
  * @brief          : None
  * @attention      : None
  * @date           : 2024/3/12
  ******************************************************************************
  */


#include "app_motor.h"

__weak void app_motor_offline_callback ()
{

}

/**
 * @brief 电机监控结构体数组
 */
app_motor_t motor_mon[8] = {0};

void app_motor_online ()
{

}

bool app_motor_online_detect (app_motor_t *mon)
{
  uint32_t tick_to_wait =
#if APP_MOTOR_USE_DEFAULT_WAIT_TICK == 1
	  APP_MOTOR_DEFAULT_WAIT_TICK;
#else
  mon->online.wait_tick;
#endif

  if (get_app_motor_tick - mon->online.last_tick > tick_to_wait)
	{
	  mon->online.online = false;
	}

  mon->online.last_tick = get_app_motor_tick;
  return mon->online.online;
}
