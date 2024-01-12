//
// Created by zhpwa on 2024/1/10.
//

#include "app_pid.h"

#include "stdio.h"

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

#define CHASSIS_SERVO_SPEED_LIM_I 300
#define CHASSIS_SERVO_SPEED_LIM 300

int PID_Setup (void)
{
  uint8_t i = 0;
  (pid_speed_struct + i)->motor_number = 0;
  (pid_speed_struct + i)->active = true;
  (pid_speed_struct + i)->ideal = 0;
  (pid_speed_struct + i)->Kp = 6.5f; /*1*/
  (pid_speed_struct + i)->Ki = 0.05f;
  (pid_speed_struct + i)->Kd = 0.001f;
  (pid_speed_struct + i)->Limit_Iout = 15000;
  (pid_speed_struct + i)->Limit_Out = 15000;
  (pid_speed_struct + i)->Error = 0;
  i++;
  (pid_speed_struct + i)->motor_number = 1;
  (pid_speed_struct + i)->active = true;
  (pid_speed_struct + i)->ideal = 0;
  (pid_speed_struct + i)->Kp = 6.5f; /*2*/
  (pid_speed_struct + i)->Ki = 0.005f;
  (pid_speed_struct + i)->Kd = 0.001f;
  (pid_speed_struct + i)->Limit_Iout = 30000;
  (pid_speed_struct + i)->Limit_Out = 30000;
  (pid_speed_struct + i)->Error = 0;
  i++;
  (pid_speed_struct + i)->motor_number = 2;
  (pid_speed_struct + i)->active = true;
  (pid_speed_struct + i)->ideal = 0;
  (pid_speed_struct + i)->Kp = 6.5f; /*3*/
  (pid_speed_struct + i)->Ki = 0.5f;
  (pid_speed_struct + i)->Kd = 0.001f;
  (pid_speed_struct + i)->Limit_Iout = 30000;
  (pid_speed_struct + i)->Limit_Out = 30000;
  (pid_speed_struct + i)->Error = 0;
  i++;
  (pid_speed_struct + i)->motor_number = 3;
  (pid_speed_struct + i)->active = false;
  (pid_speed_struct + i)->ideal = 0;
  (pid_speed_struct + i)->Kp = 6.5f; /*4*/
  (pid_speed_struct + i)->Ki = 0.5f;
  (pid_speed_struct + i)->Kd = 0.001f;
  (pid_speed_struct + i)->Limit_Iout = 30000;
  (pid_speed_struct + i)->Limit_Out = 30000;
  (pid_speed_struct + i)->Error = 0;
  i++;
  (pid_speed_struct + i)->motor_number = 4;
  (pid_speed_struct + i)->active = true;
  (pid_speed_struct + i)->ideal = 0;
  (pid_speed_struct + i)->Kp = 170.0F; /*5 6020 no.1*/
  (pid_speed_struct + i)->Ki = 7.0F;
  (pid_speed_struct + i)->Kd = 0.0F;
  (pid_speed_struct + i)->Limit_Iout = 30000;
  (pid_speed_struct + i)->Limit_Out = 30000;
  (pid_speed_struct + i)->Error = 0;
  i++;
  (pid_speed_struct + i)->motor_number = 5;
  (pid_speed_struct + i)->active = true;
  (pid_speed_struct + i)->ideal = 0;
  (pid_speed_struct + i)->Kp = 170.0F; /*6 6020 no.2*/
  (pid_speed_struct + i)->Ki = 7.0F;
  (pid_speed_struct + i)->Kd = 0.0F;
  (pid_speed_struct + i)->Limit_Iout = 30000;
  (pid_speed_struct + i)->Limit_Out = 30000;
  (pid_speed_struct + i)->Error = 0;
  i++;
  (pid_speed_struct + i)->motor_number = 6;
  (pid_speed_struct + i)->active = true;
  (pid_speed_struct + i)->ideal = 0;
  (pid_speed_struct + i)->Kp = 170.0F; /*7 6020 no.3*/
  (pid_speed_struct + i)->Ki = 7.0F;
  (pid_speed_struct + i)->Kd = 0.0F;
  (pid_speed_struct + i)->Limit_Iout = 30000;
  (pid_speed_struct + i)->Limit_Out = 30000;
  (pid_speed_struct + i)->Error = 0;
  i++;
  (pid_speed_struct + i)->motor_number = 7;
  (pid_speed_struct + i)->active = true;
  (pid_speed_struct + i)->ideal = 0;
  (pid_speed_struct + i)->Kp = 170.0F; /*8 6020 no.4*/
  (pid_speed_struct + i)->Ki = 7.0F;
  (pid_speed_struct + i)->Kd = 0.0F;
  (pid_speed_struct + i)->Limit_Iout = 30000;
  (pid_speed_struct + i)->Limit_Out = 30000;
  (pid_speed_struct + i)->Error = 0;
  i++;
  (pid_speed_struct + i)->motor_number = 8;
  (pid_speed_struct + i)->active = true;
  (pid_speed_struct + i)->ideal = 0;
  (pid_speed_struct + i)->Kp = 0.2F; /*9 motor5 6020 no.1 position*/
  (pid_speed_struct + i)->Ki = 0;
  (pid_speed_struct + i)->Kd = 0;
  (pid_speed_struct + i)->Limit_Iout = CHASSIS_SERVO_SPEED_LIM_I;
  (pid_speed_struct + i)->Limit_Out = CHASSIS_SERVO_SPEED_LIM;
  (pid_speed_struct + i)->Error = 0;
  i++;
  (pid_speed_struct + i)->motor_number = 9;
  (pid_speed_struct + i)->active = true;
  (pid_speed_struct + i)->ideal = 0;
  (pid_speed_struct + i)->Kp = 0.2F; /*10 motor5 6020 no.1 position*/
  (pid_speed_struct + i)->Ki = 0;
  (pid_speed_struct + i)->Kd = 0;
  (pid_speed_struct + i)->Limit_Iout = CHASSIS_SERVO_SPEED_LIM_I;
  (pid_speed_struct + i)->Limit_Out = CHASSIS_SERVO_SPEED_LIM;
  (pid_speed_struct + i)->Error = 0;
  i++;
  (pid_speed_struct + i)->motor_number = 10;
  (pid_speed_struct + i)->active = true;
  (pid_speed_struct + i)->ideal = 0;
  (pid_speed_struct + i)->Kp = 0.2F; /*11 motor5 6020 no.1 position*/
  (pid_speed_struct + i)->Ki = 0;
  (pid_speed_struct + i)->Kd = 0;
  (pid_speed_struct + i)->Limit_Iout = CHASSIS_SERVO_SPEED_LIM_I;
  (pid_speed_struct + i)->Limit_Out = CHASSIS_SERVO_SPEED_LIM;
  (pid_speed_struct + i)->Error = 0;
  i++;
  (pid_speed_struct + i)->motor_number = 11;
  (pid_speed_struct + i)->active = true;
  (pid_speed_struct + i)->ideal = 0;
  (pid_speed_struct + i)->Kp = 0.2F; /*12 motor5 6020 no.1 position*/
  (pid_speed_struct + i)->Ki = 0;
  (pid_speed_struct + i)->Kd = 0;
  (pid_speed_struct + i)->Limit_Iout = CHASSIS_SERVO_SPEED_LIM;
  (pid_speed_struct + i)->Limit_Out = CHASSIS_SERVO_SPEED_LIM;
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

int PID_Calculate (void)
{
  motor_measure_t *motinfo = (motor_measure_t *) get_measure_pointer (0);
  motor_measure_t *servoinfo = (motor_measure_t *) get_measure_pointer (0);
  PID *servo = pid_get_struct_pointer (8, NORMAL_MOTOR);
  chassis_transported_controller_data *rc_chsas = get_rc_data_from_chassis_pointer ();


  /*reset motinfo to motor 5 (first 6020)*/
  /*calculate PID position first*/
  for (int i = 0; i < 4;)
	{

	  (servo + i)->actual = (float) (servoinfo + i)->total_ecd;

	  if ((servo + i)->active == true)
		{
		  if ((servo + i)->Error == 0)
			{

			  (servo + i)->err = (servo + i)->ideal - (servo + i)->actual;
			  (servo + i)->integral += (servo + i)->err;

			  (servo + i)->Pout = (servo + i)->Kp * (servo + i)->err;
			  (servo + i)->Iout = (servo + i)->Ki * (servo + i)->integral;
			  (servo + i)->Dout = (servo + i)->Kd
								  * ((servo + i)->err - 2.0f * (servo + i)->err_last
									 + (servo + i)->err_last_last);

			  (servo + i)->output =
				  (servo + i)->Pout + (servo + i)->Iout + (servo + i)->Dout;
			  (servo + i)->err_last = (servo + i)->err;
			  PID_Out_Limit ((servo + i));

			  /*give PID speed target value*/
			  //todo 添加自由控制双环开启状态的宏
			  ((servo + i) - 4)->ideal =
				  (servo + i)->output;
			}
		}
	  motinfo++;
	  i++;
	}

  for (int i = 0; i < 8;)
	{
	  (pid_speed_struct + i)->actual = get_measure_pointer (i)->speed_rpm;

	  if ((pid_speed_struct + i)->active == true)
		{
		  if ((pid_speed_struct + i)->Error == 0)
			{

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
