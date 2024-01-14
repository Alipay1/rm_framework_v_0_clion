//
// Created by zhpwa on 2024/1/10.
//

#include "app_pid.h"

#include "stdio.h"
#include "ins_task.h"

#define SERVO_SPD_LIM 50

float abs_f (float value)
{
  if (value >= 0)
	{
	  return value;
	}
  else
	{
	  return 0 - value;
	}
}

//__attribute__ ((section(".ccmram")))
static PID pid_speed_struct[PID_SPEED_STRUCT_NUM] = {0};

int PID_Setup (void)
{
  uint8_t i = 0;
  (pid_speed_struct + i)->motor_number = 0;
  (pid_speed_struct + i)->active = true;
  (pid_speed_struct + i)->ideal = -6000;
  (pid_speed_struct + i)->Kp = 5.0f; /*1*/
  (pid_speed_struct + i)->Ki = 0.02f;
  (pid_speed_struct + i)->Kd = 0.1f;
  (pid_speed_struct + i)->Limit_Iout = 30000;
  (pid_speed_struct + i)->Limit_Out = 30000;
  (pid_speed_struct + i)->Error = 0;
  i++;
  (pid_speed_struct + i)->motor_number = 1;
  (pid_speed_struct + i)->active = true;
  (pid_speed_struct + i)->ideal = 6000;
  (pid_speed_struct + i)->Kp = 5.0f; /*2*/
  (pid_speed_struct + i)->Ki = 0.02f;
  (pid_speed_struct + i)->Kd = 0.1f;
  (pid_speed_struct + i)->Limit_Iout = 30000;
  (pid_speed_struct + i)->Limit_Out = 30000;
  (pid_speed_struct + i)->Error = 0;
  i++;
  (pid_speed_struct + i)->motor_number = 2;
  (pid_speed_struct + i)->active = true;
  (pid_speed_struct + i)->ideal = -4000;
  (pid_speed_struct + i)->Kp = 6.5f; /*3*/
  (pid_speed_struct + i)->Ki = 0.5f;
  (pid_speed_struct + i)->Kd = 0.0f;
  (pid_speed_struct + i)->Limit_Iout = 16384;
  (pid_speed_struct + i)->Limit_Out = 16384;
  (pid_speed_struct + i)->Error = 0;
  i++;
  (pid_speed_struct + i)->motor_number = 3;
  (pid_speed_struct + i)->active = false;
  (pid_speed_struct + i)->ideal = 0;
  (pid_speed_struct + i)->Kp = 0; /*4*/
  (pid_speed_struct + i)->Ki = 0;
  (pid_speed_struct + i)->Kd = 0;
  (pid_speed_struct + i)->Limit_Iout = 30000;
  (pid_speed_struct + i)->Limit_Out = 30000;
  (pid_speed_struct + i)->Error = 0;
  i++;
  (pid_speed_struct + i)->motor_number = 4;
  (pid_speed_struct + i)->active = true;
  (pid_speed_struct + i)->ideal = 0;
  (pid_speed_struct + i)->Kp = 6000.0F; /*5*/
  (pid_speed_struct + i)->Ki = 115.0F;
  (pid_speed_struct + i)->Kd = 0;
  (pid_speed_struct + i)->Limit_Iout = 10000;
  (pid_speed_struct + i)->Limit_Out = 10000;
  (pid_speed_struct + i)->Error = 0;
  (pid_speed_struct + i)->if_angular_velocity_mode = true;
  (pid_speed_struct + i)->eula = APP_PID_YAW;
  i++;
  (pid_speed_struct + i)->motor_number = 5;
  (pid_speed_struct + i)->active = true;
  (pid_speed_struct + i)->ideal = 0;
  (pid_speed_struct + i)->Kp = 1200.0f; /*6*/
  (pid_speed_struct + i)->Ki = 100.0f;
  (pid_speed_struct + i)->Kd = 0;
  (pid_speed_struct + i)->Limit_Iout = 5000;
  (pid_speed_struct + i)->Limit_Out = 5000;
  (pid_speed_struct + i)->Error = 0;
  (pid_speed_struct + i)->if_angular_velocity_mode = true;
  (pid_speed_struct + i)->eula = APP_PID_PIT;
  i++;
  (pid_speed_struct + i)->motor_number = 6;
  (pid_speed_struct + i)->active = false;
  (pid_speed_struct + i)->ideal = 0;
  (pid_speed_struct + i)->Kp = 0; /*7*/
  (pid_speed_struct + i)->Ki = 0;
  (pid_speed_struct + i)->Kd = 0;
  (pid_speed_struct + i)->Limit_Iout = 30000;
  (pid_speed_struct + i)->Limit_Out = 30000;
  (pid_speed_struct + i)->Error = 0;
  i++;
  (pid_speed_struct + i)->motor_number = 7;
  (pid_speed_struct + i)->active = false;
  (pid_speed_struct + i)->ideal = 0;
  (pid_speed_struct + i)->Kp = 0; /*8*/
  (pid_speed_struct + i)->Ki = 0;
  (pid_speed_struct + i)->Kd = 0;
  (pid_speed_struct + i)->Limit_Iout = 30000;
  (pid_speed_struct + i)->Limit_Out = 30000;
  (pid_speed_struct + i)->Error = 0;
  i++;
  (pid_speed_struct + i)->motor_number = 8;
  (pid_speed_struct + i)->active = true;
  (pid_speed_struct + i)->ideal = 0;
  (pid_speed_struct + i)->Kp = 0.1F; /*9 motor5 6020 no.1 position YAW*/
  (pid_speed_struct + i)->Ki = 0.0000F;
  (pid_speed_struct + i)->Kd = 0;
  (pid_speed_struct + i)->Limit_Iout = 10;
  (pid_speed_struct + i)->Limit_Out = SERVO_SPD_LIM;
  (pid_speed_struct + i)->Error = 0;
  (pid_speed_struct + i)->if_angular_velocity_mode = true;
  (pid_speed_struct + i)->eula = APP_PID_YAW;
  i++;
  (pid_speed_struct + i)->motor_number = 9;
  (pid_speed_struct + i)->active = true;
  (pid_speed_struct + i)->ideal = 0;
  (pid_speed_struct + i)->Kp = 0.1F; /*10 PITCH*/
  (pid_speed_struct + i)->Ki = 0.0000F;
  (pid_speed_struct + i)->Kd = 0;
  (pid_speed_struct + i)->Limit_Iout = 10;
  (pid_speed_struct + i)->Limit_Out = SERVO_SPD_LIM;
  (pid_speed_struct + i)->Error = 0;
  (pid_speed_struct + i)->if_angular_velocity_mode = true;
  (pid_speed_struct + i)->eula = APP_PID_PIT;
  i++;
  (pid_speed_struct + i)->motor_number = 10;
  (pid_speed_struct + i)->active = false;
  (pid_speed_struct + i)->ideal = 0;
  (pid_speed_struct + i)->Kp = 0.15F; /*11*/
  (pid_speed_struct + i)->Ki = 0.001F;
  (pid_speed_struct + i)->Kd = 0;
  (pid_speed_struct + i)->Limit_Iout = SERVO_SPD_LIM;
  (pid_speed_struct + i)->Limit_Out = SERVO_SPD_LIM;
  (pid_speed_struct + i)->Error = 0;
  i++;
  (pid_speed_struct + i)->motor_number = 11;
  (pid_speed_struct + i)->active = false;
  (pid_speed_struct + i)->ideal = 0;
  (pid_speed_struct + i)->Kp = 0.15F; /*12*/
  (pid_speed_struct + i)->Ki = 0.001F;
  (pid_speed_struct + i)->Kd = 0;
  (pid_speed_struct + i)->Limit_Iout = SERVO_SPD_LIM;
  (pid_speed_struct + i)->Limit_Out = SERVO_SPD_LIM;
  (pid_speed_struct + i)->Error = 0;
  return 0;
}

PID IMU_T = {
	.ideal = 40.0f,
	.actual = 0,
	.output = 0,
	.integral = 0,
	.err = 0,
	.err_last = 0,
	.err_last_last = 0,
	.Kp = 1600.0f,
	.Ki = 0.2f,
	.Kd = 0.00f,
	.Limit_Iout = 30000,
	.Limit_Out = 30000,
	.Error = 0};

void PID_Out_Limit (PID *pxStruct)
{
  // 限制积分值的范围
  pxStruct->integral = (pxStruct->integral > pxStruct->Limit_Iout) ? pxStruct->Limit_Iout : pxStruct->integral;
  pxStruct->integral = (pxStruct->integral < -pxStruct->Limit_Iout) ? -pxStruct->Limit_Iout : pxStruct->integral;

  // 限制输出值的范围
  pxStruct->output = (pxStruct->output > pxStruct->Limit_Out) ? pxStruct->Limit_Out : pxStruct->output;
  pxStruct->output = (pxStruct->output < -pxStruct->Limit_Out) ? -pxStruct->Limit_Out : pxStruct->output;
}

int PID_Calculate_seper (PID *input_spd, PID *input_pos)
{
  motor_measure_t *motinfo = (motor_measure_t *) get_measure_pointer (input_spd->motor_number);

/*判断是哪种模式*/
  if (input_pos->if_angular_velocity_mode)
	{
	  float temp = 0;
	  switch (input_pos->eula)
		{
		  case APP_PID_YAW: temp = INS.Yaw;
		  break;
		  case APP_PID_PIT: temp = INS.Pitch;
		  break;
		  case APP_PID_ROL: temp = INS.Roll;
		  break;
		  default: break;
		}
	  input_pos->actual = temp;
	}
  else
	{
	  input_pos->actual = (float) motinfo->total_ecd;
	}
/*判断是哪种模式*/

  if (input_pos->active == true)
	{
	  if (input_pos->Error == 0)
		{

		  input_pos->err = input_pos->ideal - input_pos->actual;
		  input_pos->integral += input_pos->err;

		  input_pos->Pout = input_pos->Kp * input_pos->err;
		  input_pos->Iout = input_pos->Ki * input_pos->integral;
		  input_pos->Dout = input_pos->Kd
							* (input_pos->err - 2.0f * input_pos->err_last
							   + input_pos->err_last_last);

		  input_pos->output =
			  input_pos->Pout + input_pos->Iout + input_pos->Dout;
		  input_pos->err_last = input_pos->err;
		  PID_Out_Limit (input_pos);

		  /*give PID speed target value*/
		  //todo 添加自由控制双环开启状态的宏

		  input_spd->ideal = input_pos->output;
		}
	}
  else if (input_pos->active == false)
	{
	  input_spd->ideal = input_pos->ideal;
	}


/*判断是哪种模式*/
  if (input_spd->if_angular_velocity_mode)
	{
	  float temp = 0;
	  switch (input_spd->eula)
		{
		  case APP_PID_YAW: temp = INS.Gyro[2];
		  break;
		  case APP_PID_PIT: temp = INS.Gyro[0];
		  break;
		  case APP_PID_ROL: temp = INS.Gyro[1];
		  break;
		  default: break;
		}
	  input_spd->actual = temp;
	}
  else
	{
	  input_spd->actual = motinfo->speed_rpm;
	}
/*判断是哪种模式*/

  if (input_spd->active == true)
	{
	  if (input_spd->Error == 0)
		{

		  input_spd->err = input_spd->ideal - input_spd->actual;
		  input_spd->integral += input_spd->err;

		  input_spd->Pout = input_spd->Kp * input_spd->err;
		  input_spd->Iout = input_spd->Ki * input_spd->integral;
		  input_spd->Dout = input_spd->Kd
							* (input_spd->err - 2.0f * input_spd->err_last
							   + input_spd->err_last_last);

		  input_spd->output =
			  input_spd->Pout + input_spd->Iout + input_spd->Dout;
		  input_spd->err_last = input_spd->err;
		  PID_Out_Limit (input_spd);
		}
	}
}

int PID_Calculate_single (PID *input_spd)
{
  motor_measure_t *motinfo = (motor_measure_t *) get_measure_pointer (input_spd->motor_number);

  input_spd->actual = motinfo->speed_rpm;

  if (input_spd->active == true)
	{
	  if (input_spd->Error == 0)
		{

		  input_spd->err = input_spd->ideal - input_spd->actual;
		  input_spd->integral += input_spd->err;

		  input_spd->Pout = input_spd->Kp * input_spd->err;
		  input_spd->Iout = input_spd->Ki * input_spd->integral;
		  input_spd->Dout = input_spd->Kd
							* (input_spd->err - 2.0f * input_spd->err_last
							   + input_spd->err_last_last);

		  input_spd->output =
			  input_spd->Pout + input_spd->Iout + input_spd->Dout;
		  input_spd->err_last = input_spd->err;
		  PID_Out_Limit (input_spd);
		}
	}
}

int app_PID_Calculate (void)
{
//  motor_measure_t *motinfo = (motor_measure_t *) get_measure_pointer (0);
//  chassis_transported_controller_data *rc_chsas = get_rc_data_from_chassis_pointer ();
//
//  for (int i = 0; i < 4;)
//	{
//	  (pid_speed_struct + i)->actual = get_measure_pointer (i)->speed_rpm;
//
//	  if ((pid_speed_struct + i)->active == true)
//		{
//		  if ((pid_speed_struct + i)->Error == 0)
//			{
//
//			  (pid_speed_struct + i)->err = (pid_speed_struct + i)->ideal - (pid_speed_struct + i)->actual;
//			  (pid_speed_struct + i)->integral += (pid_speed_struct + i)->err;
//
//			  (pid_speed_struct + i)->Pout = (pid_speed_struct + i)->Kp * (pid_speed_struct + i)->err;
//			  (pid_speed_struct + i)->Iout = (pid_speed_struct + i)->Ki * (pid_speed_struct + i)->integral;
//			  (pid_speed_struct + i)->Dout = (pid_speed_struct + i)->Kd
//											 * ((pid_speed_struct + i)->err - 2.0f * (pid_speed_struct + i)->err_last
//												+ (pid_speed_struct + i)->err_last_last);
//
//			  (pid_speed_struct + i)->output =
//				  (pid_speed_struct + i)->Pout + (pid_speed_struct + i)->Iout + (pid_speed_struct + i)->Dout;
//			  (pid_speed_struct + i)->err_last = (pid_speed_struct + i)->err;
//			  PID_Out_Limit (pid_speed_struct + i);
//			}
//		}
//	  i++;
//	  motinfo++;
//	}
//
//  return 0;
  motor_measure_t *motinfo = (motor_measure_t *) get_measure_pointer (0);
  chassis_transported_controller_data *rc_chsas = get_rc_data_from_chassis_pointer ();
  // HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);
  for (int i = 0; i < 4;)
	{
	  if ((pid_speed_struct + i)->active == true)
		{
		  if ((pid_speed_struct + i)->Error == 0)
			{
			  (pid_speed_struct + i)->actual = motinfo->speed_rpm;
			  (pid_speed_struct + i)->err = (pid_speed_struct + i)->ideal - (pid_speed_struct + i)->actual;
			  (pid_speed_struct + i)->integral += (pid_speed_struct + i)->err;

			  (pid_speed_struct + i)->Pout = (pid_speed_struct + i)->Kp * (pid_speed_struct + i)->err;
			  (pid_speed_struct + i)->Iout = (pid_speed_struct + i)->Ki * (pid_speed_struct + i)->integral;
			  (pid_speed_struct + i)->Dout = (pid_speed_struct + i)->Kd
											 * ((pid_speed_struct + i)->err - 2.0f * (pid_speed_struct + i)->err_last
												+ (pid_speed_struct + i)->err_last_last);

			  (pid_speed_struct + i)->output =
				  (pid_speed_struct + i)->Pout + (pid_speed_struct + i)->Iout + (pid_speed_struct + i)->Dout;
			  (pid_speed_struct + i)->err_last = (pid_speed_struct + i)->err;
			  PID_Out_Limit (pid_speed_struct + i);
			}
		}
	  i++;
	  motinfo++;
	}
  return 0;
}

PID *pid_get_struct_pointer (uint32_t num, uint32_t in_array)
{
  switch (in_array)
	{
	  case NORMAL_MOTOR: return pid_speed_struct + num;
	  break;
	  case PITCH_MOTOR:
		switch (num)
		  {
//			case 0: return &PITCH_V;
//			break;
//			case 1: return &PITCH_A;
//			break;
			default: return NULL;
			break;
		  }
	  case YAW_MOTOR:
		switch (num)
		  {
//			case 0: return &YAW_V;
//			break;
//			case 1: return &YAW_P;
//			break;
			default: return NULL;
			break;
		  }
	  break;

	  default: return NULL;
	  break;
	}
}

int pid_sscanf (char *input)
{
  PID temp = {0};
  int run_or_not = 0;
  int motor_num = 0;
  float Kp = 0,
	  Ki = 0,
	  Kd = 0;
  PID *apply = NULL;

  sscanf (input, "set%d/%d/%f/%f/%f\r\n",
		  &run_or_not,
		  &motor_num,
		  &Kp,
		  &Ki,
		  &Kd);

  /*if motor num over the limit return*/
  if (motor_num > 11 || motor_num < 0)
	{
	  return -1;
	}
  else
	{
	  apply = pid_get_struct_pointer (motor_num, NORMAL_MOTOR);
	}

  /*active the motor*/
  if (run_or_not == 1)
	{
	  apply->active = true;
	}
  else if (run_or_not == 0)
	{
	  apply->active = false;
	  apply->output = 0;
	}

  apply->Kp = Kp;
  apply->Ki = Ki;
  apply->Kd = Kd;

  return 0;
}

//int32_t get_yaw_motor_position (void)
//{
//  return YAW_Motor_Position;
//}
