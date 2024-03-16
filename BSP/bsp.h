//
// Created by zhpwa on 2024/1/8.
//

#ifndef C_BOARD_STANDARD_ROBOT_BSP_H
#define C_BOARD_STANDARD_ROBOT_BSP_H

#define GIMBAL_CODE 1
#define CHASSIS_CODE 0

#if GIMBAL_CODE == CHASSIS_CODE
#error "!!!Check GIMBAL_CODE & CHASSIS_CODE definition!!!"
#endif

#include "bsp_led.h"
#include "bsp_uart.h"
#include "bsp_can.h"
#include "bsp_buz.h"
#include "bsp_check.h"
#include "bsp_dwt.h"
#include "bsp_rc.h"

#endif // C_BOARD_STANDARD_ROBOT_BSP_H
