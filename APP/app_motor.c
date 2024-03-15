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
#include "app_pid.h"
#include "bsp.h"

/**
 * @brief 转换索引
 * @details 发现一个小问题，从电机号到pid序号不可直接引用，要将4和5调换，
 * 			推测是因为yaw pit与dji顺序相反导致的，今后代码或装车必须考虑
 * 			这点，否则需要转换索引再使用电机号或pid序号
 * @param motor_num 电机序号
 * @return 用于pid索引的序号
 */
static uint8_t app_motor_index_tran (uint8_t motor_num)
{
  uint8_t temp = 0;
  switch (motor_num)
	{
	  case 5:
		{
		  return 4;
		}
	  case 4:
		{
		  return 5;
		}
	  default: return motor_num;
	}
}

__weak void app_motor_offline_callback (uint8_t motor_num)
{
  uint8_t temp = app_motor_index_tran (motor_num);

  PID *motor_pid = pid_get_ptr (temp);//获取速度环pid结构体
  PID *raw_motor_pid = pid_get_raw_ptr (temp);//获取速度环pid原始结构体

  *motor_pid = *raw_motor_pid;
  motor_pid->active = false;

  if (temp > 3 && temp < 8)//如果是6020
	{
	  PID *motor_pid_pos = pid_get_ptr (temp + 4);//获取速度环pid结构体
	  PID *raw_motor_pid_pos = pid_get_raw_ptr (temp + 4);//获取速度环pid原始结构体

	  *motor_pid_pos = *raw_motor_pid_pos;
	  motor_pid_pos->active = false;
	}

}

/**
 * @brief 电机监控结构体数组
 */
app_motor_t motor_mon[MOTOR_MON_NUM] = {0};

/**
 * @brief 获取电机监测结构体指针
 * @param motor_num 要获取的目标的序号
 * @return 电机监测结构体指针
 */
app_motor_t *get_motor_mon_ptr (uint8_t motor_num)
{
  if (motor_num >= MOTOR_MON_NUM)
	{
	  return NULL;
	}
  return motor_mon + motor_num;
}

/**
 * @brief 在CAN接收中断中使用，每当接收到电机报文代表电机在线，将电机置为激活状态，并且更新last_tick
 * @param motor_num
 */
void app_motor_online (uint8_t motor_num)
{
//  if (motor_num >= MOTOR_MON_NUM)
//	{
//	  return;
//	}

  uint8_t temp = app_motor_index_tran (motor_num);
  PID *motor_pid = pid_get_ptr (temp);//获取速度环pid结构体

//  if (motor_mon[motor_num].online.online == false)
//	{
//	  motor_pid->active = true;
//	}
  if ((temp + 4) < 12 && (temp + 4) > 7)//如果是6020
	{
	  PID *motor_pid_pos = pid_get_ptr (temp + 4);//获取速度环pid结构体
	  motor_pid_pos->active = true;
	}

  motor_pid->active = true;
  motor_mon[motor_num].online.online = true;
  motor_mon[motor_num].online.last_tick = get_app_motor_tick;
}

/**
 * @brief 电机活跃状态的后台检测程序，需要定时调用
 * @param mon
 * @return 电机活跃状态
 */
bool app_motor_online_detect (void)
{
  static uint32_t tick_to_wait =
#if APP_MOTOR_USE_DEFAULT_WAIT_TICK == 1
	  APP_MOTOR_DEFAULT_WAIT_TICK;
#else
  mon->online.wait_tick;
#endif

  for (uint8_t i = 0; i < MOTOR_MON_NUM; i++)
	{
	  if (get_app_motor_tick - (motor_mon + i)->online.last_tick > tick_to_wait)
		{
		  if ((motor_mon + i)->online.online == true)
			{
			  (motor_mon + i)->online.online = false;
			  app_motor_offline_callback (i);
			}
		}
	}

  return motor_mon->online.online;
}
