/**
  ******************************************************************************
  * @file           : app_motor.h
  * @author         : zhpwa
  * @brief          : None
  * @attention      : None
  * @date           : 2024/3/12
  ******************************************************************************
  */


#ifndef _APP_MOTOR_H_
#define _APP_MOTOR_H_

#include "stm32f4xx_hal.h"

#include "stdbool.h"

#define APP_MOTOR_USE_DEFAULT_WAIT_TICK 1
#define APP_MOTOR_DEFAULT_WAIT_TICK 100U

/**
 * @brief app_motor结构体
 * @var last_tick 123
 */
typedef struct {
  struct {
	uint32_t last_tick;
	uint32_t wait_tick;
	bool online;
  }
	  online;

  struct {
	uint32_t overheat_count;//
	uint32_t temp_lim;//温度上限
	bool overheat;
  }
	  ovh;
} app_motor_t;

#define MOTOR_MON_NUM 8U
extern app_motor_t motor_mon[MOTOR_MON_NUM];

#define get_app_motor_tick HAL_GetTick()
//#define get_app_motor_tick xTaskGetTickCount()

void app_motor_offline_callback (uint8_t motor_num);

app_motor_t *get_motor_mon_ptr (uint8_t motor_num);
void app_motor_online (uint8_t motor_num);
bool app_motor_online_detect (void);

#endif //_APP_MOTOR_H_
