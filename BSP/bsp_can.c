//
// Created by zhpwa on 2024/1/10.
//

#include "bsp_can.h"

static CAN_TxHeaderTypeDef hcan1_tx_header;
static CAN_TxHeaderTypeDef hcan2_tx_header;

static uint8_t hcan1_message[8];
static uint8_t hcan2_message[8];

static motor_measure_t MotorInfo[8];

chassis_transported_controller_data rc_ctcd = {0};

ammo_booster_data ammo_booster_info = {0};

motor_measure_t *get_measure_pointer (uint32_t i)
{
  return MotorInfo + i;
}

chassis_transported_controller_data *get_rc_data_from_chassis_pointer (void)
{
  return &rc_ctcd;
}

void CAN_FilterSetup (void)
{
  CAN_FilterTypeDef setup_handle;

  setup_handle.FilterActivation = ENABLE;
  setup_handle.FilterMode = CAN_FILTERMODE_IDMASK;
  setup_handle.FilterScale = CAN_FILTERSCALE_32BIT;
  setup_handle.FilterIdHigh = 0x0000;
  setup_handle.FilterIdLow = 0x0000;
  setup_handle.FilterMaskIdHigh = 0x0000;
  setup_handle.FilterMaskIdLow = 0x0000;
  setup_handle.FilterBank = 0;
  setup_handle.FilterFIFOAssignment = CAN_RX_FIFO0;
  HAL_CAN_ConfigFilter (&hcan1, &setup_handle);
  HAL_CAN_Start (&hcan1);
  HAL_CAN_ActivateNotification (&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  setup_handle.SlaveStartFilterBank = 14;
  setup_handle.FilterBank = 14;
  HAL_CAN_ConfigFilter (&hcan2, &setup_handle);
  HAL_CAN_Start (&hcan2);
  HAL_CAN_ActivateNotification (&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void get_ammo_booster_info (ammo_booster_data *ptr, uint8_t *data)
{
  (ptr)->ID = data[0];
  (ptr)->shooting_rate = data[1];
  (ptr)->bullet_speed = (data[3] << 8) | data[2];
  (ptr)->muzzle_heat = data[4];
  (ptr)->muzzle_heat_lim = data[5];
  (ptr)->muzzle_cooling_rate = data[6];
  (ptr)->muzzle_speed_lim = data[7];
}

uint16_t dat = 0;

uint32_t
CAN_SendMessage (can_channel_id CAN_Channel, motor_id_range MOTOR_ID_RANGE, int16_t Motor1, int16_t Motor2, int16_t Motor3, int16_t Motor4)
{
  uint32_t TxMailBox = 0;
  CAN_HandleTypeDef *pxhcan = NULL;
  CAN_TxHeaderTypeDef *pxHeader = NULL;
  uint8_t *pxMessage = NULL;

  switch (CAN_Channel)
	{
	  case CAN_CHANNEL_1: pxhcan = &hcan1;
	  pxHeader = &hcan1_tx_header;
	  pxMessage = (uint8_t *) &hcan1_message;
	  break;
	  case CAN_CHANNEL_2: pxhcan = &hcan2;
	  pxHeader = &hcan2_tx_header;
	  pxMessage = (uint8_t *) &hcan2_message;
	  break;
	  default: return 0;
	}

  switch (MOTOR_ID_RANGE)
	{
	  case MOTOR_1234:

		pxHeader->StdId = CAN_CHASSIS_ALL_ID;
	  break;
	  case MOTOR_5678: pxHeader->StdId = CAN_GIMBAL_ALL_ID;
	  break;
	  case ECD_REPORT: pxHeader->StdId = ECD_REPORT_ID;
	  break;

	  default: break;
	}
  pxHeader->IDE = CAN_ID_STD;
  pxHeader->RTR = CAN_RTR_DATA;
  pxHeader->DLC = 0x08;

  pxMessage[0] = (Motor1 >> 8);
  pxMessage[1] = Motor1;
  pxMessage[2] = (Motor2 >> 8);
  pxMessage[3] = Motor2;
  pxMessage[4] = (Motor3 >> 8);
  pxMessage[5] = Motor3;
  pxMessage[6] = (Motor4 >> 8);
  pxMessage[7] = Motor4;

  HAL_CAN_AddTxMessage (pxhcan, pxHeader, pxMessage, &TxMailBox);
  return TxMailBox;
}

chassis_transported_controller_data *get_rc_data_from_chassis (void)
{
  return &rc_ctcd;
}

ammo_booster_data *get_ammo_booster_info_ptr (void)
{
  return &ammo_booster_info;
}
/**
 * @param motor_num motor number
 * @return total_ecd
 *
 * */
int32_t bsp_can_updateTotalAngle (uint32_t motor_num)
{
  get_measure_pointer (motor_num)->speed_rpm;

  int16_t difference = (int16_t)
	  (MotorInfo[motor_num].ecd - MotorInfo[motor_num].last_ecd);

  if (difference > ENCODER_THRESHOLD)
	{
	  // 8191->0
	  difference -= (ENCODER_MAX_VALUE + 1);
	}
  else if (difference < -ENCODER_THRESHOLD)
	{
	  // 0->8191
	  difference += (ENCODER_MAX_VALUE + 1);
	}

  MotorInfo[motor_num].total_ecd += difference;

  return MotorInfo[motor_num].total_ecd;
}