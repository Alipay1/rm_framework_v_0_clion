/**
  ******************************************************************************
  * @file           : app_aim.h
  * @author         : zhpwa
  * @brief          : from https://github.com/xinyang-go/SJTU-RM-CV-2019
  * @attention      : 编译环境为arm-gcc，使用packed命令将收发结构体按字节对齐
  * @date           : 2024/3/16
  ******************************************************************************
  */


#ifndef _APP_AIM_H_
#define _APP_AIM_H_

#include "stdint.h"

#define APP_AIM_UART_HANDLE huart6

///@brief End of data frame
#define EO_DF ('\n')

/**
 * @brief 数据接收结构体
 * @attention 每个数据帧后使用字符'\n'作为帧尾标志，这意味着结构体长度是(sizeof(struct McuData) + 1)
 */
struct McuData {
  float curr_yaw;      /// 当前云台yaw角度
  float curr_pitch;    /// 当前云台pitch角
  uint8_t state;       /// 当前状态，自瞄-大符-小符
  uint8_t mark;        /// 云台角度标记位
  uint8_t anti_top;    /// 是否为反陀螺模式
  uint8_t enemy_color; /// 敌方颜色
  int delta_x;         /// 能量机关x轴补偿量
  int delta_y;         /// 能量机关y轴补偿量
}__attribute__((packed));

/**
 * @brief 数据发送结构体
 */
struct SendData {
  char start_flag;      /// 帧头标志，字符's'
  int16_t yaw;          /// float类型的实际角度(以度为单位)/100*(32768-1)
  int16_t pitch;        /// float类型的实际角度(以度为单位)/100*(32768-1)
  uint16_t shoot_delay; /// 反陀螺模式下的发射延迟
  char end_flag;        /// 帧尾标识，字符'e'
}__attribute__((packed));

void app_aim_init (void);
void app_aim_send (void);
void app_aim_decode (int rx_size);

#endif //_APP_AIM_H_
