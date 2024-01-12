//
// Created by zhpwa on 2024/1/12.
//

#include <math.h>
#include "app_nav.h"
#include "app_rc.h"
#include "arm_math.h"
#include "app_pid.h"

#include "chassis.h"

//-RC->rc.ch[2] Vy
//RC->rc.ch[3]  Vx
//RC->rc.ch[0]  Vw

static app_nav_t nav = {0};

extern RC_ctrl_t rc_ctrl;

float map_degree_to_8191 (float input)
{
  return input * 22.7527777777777777F;
}

app_nav_t *get_navigation_p (void)
{
  return &nav;
}

//fp64 atan_angle[4];
//
//int8_t dirt[4] = {1, -1, 1, -1};

//#define MS7010_FL_ANGLE (3514 + 1024)    //装上去的时候Y轴正向对应的编码值(偏置角度)
//#define MS7010_FR_ANGLE (1714  -1024)
//#define MS7010_BL_ANGLE (3751 -1024)
//#define MS7010_BR_ANGLE (4239 +1024)
void nav_main (void)
{
  PID *wheel0 = pid_get_struct_pointer (0, NORMAL_MOTOR);
  PID *wheel1 = pid_get_struct_pointer (1, NORMAL_MOTOR);
  PID *wheel2 = pid_get_struct_pointer (2, NORMAL_MOTOR);
  PID *wheel3 = pid_get_struct_pointer (3, NORMAL_MOTOR);

  PID *servo0_pos = pid_get_struct_pointer (4 + 4, NORMAL_MOTOR);/*4 + 4 indicates motor5(4) -> PID -> position(+4)*/
  PID *servo1_pos = pid_get_struct_pointer (5 + 4, NORMAL_MOTOR);/*4 + 4 indicates motor5(4) -> PID -> position(+4)*/
  PID *servo2_pos = pid_get_struct_pointer (6 + 4, NORMAL_MOTOR);/*4 + 4 indicates motor5(4) -> PID -> position(+4)*/
  PID *servo3_pos = pid_get_struct_pointer (7 + 4, NORMAL_MOTOR);/*4 + 4 indicates motor5(4) -> PID -> position(+4)*/

  motor_measure_t *gm6020 = get_measure_pointer (4);

  nav.Vx = (float) rc_ctrl.rc.ch[3];
  nav.Vy = (float) -rc_ctrl.rc.ch[2];
  nav.Vw = (float) -rc_ctrl.rc.ch[0];

//  float temp[4] = {0};
//
//  nav.V[0] = sqrt (pow (nav.Vy - nav.Vw * cos (PI / 4), 2) +
//				   pow (nav.Vx - nav.Vw * sin (PI / 4), 2));
//  nav.V[1] = sqrt (pow (nav.Vy - nav.Vw * cos (PI / 4), 2) +
//				   pow (nav.Vx + nav.Vw * sin (PI / 4), 2));
//  nav.V[2] = sqrt (pow (nav.Vy + nav.Vw * cos (PI / 4), 2) +
//				   pow (nav.Vx + nav.Vw * sin (PI / 4), 2));
//  nav.V[3] = sqrt (pow (nav.Vy + nav.Vw * cos (PI / 4), 2) +
//				   pow (nav.Vx - nav.Vw * sin (PI / 4), 2));
//
//  if (!(nav.Vy == 0 && nav.Vx == 0 && nav.Vw == 0))//防止除数为零
//	{
//	  //由于atan2算出来的结果是弧度，需转换成角度 计算公式为 弧度 * 180.f / PI 最终得到角度值 (0.707107f == 根号2)
//	  atan_angle[0] = atan ((nav.Vy - nav.Vw * cos (PI / 4)) /
//							(nav.Vx - nav.Vw * sin (PI / 4)));
//	  atan_angle[1] = atan ((nav.Vy - nav.Vw * cos (PI / 4)) /
//							(nav.Vx + nav.Vw * sin (PI / 4)));
//	  atan_angle[2] = atan ((nav.Vy + nav.Vw * cos (PI / 4)) /
//							(nav.Vx + nav.Vw * sin (PI / 4)));
//	  atan_angle[3] = atan ((nav.Vy + nav.Vw * cos (PI / 4)) /
//							(nav.Vx - nav.Vw * sin (PI / 4)));
//	}
//  nav.theta[0] = Angle_Limit (MS7010_FL_ANGLE + (fp32) (atan_angle[0] * 22.75277777777f), 8191.f);
//  nav.theta[1] = Angle_Limit (MS7010_FR_ANGLE + (fp32) (atan_angle[1] * 22.75277777777f), 8191.f);
//  nav.theta[2] = Angle_Limit (MS7010_BL_ANGLE + (fp32) (atan_angle[2] * 22.75277777777f), 8191.f);
//  nav.theta[3] = Angle_Limit (MS7010_BR_ANGLE + (fp32) (atan_angle[3] * 22.75277777777f), 8191.f);
//
//  //优弧 劣弧 驱动电机转向判断
//  if (ABS ((fp32) gm6020[0].total_ecd - wheel_angle[0]) > 2048)
//	{
//	  dirt[0] = -1;
//	  wheel_angle[0] = Angle_Limit (wheel_angle[0] - 4096, 8191);
//	}
//  else
//	dirt[0] = 1;
//
//  if (ABS ((fp32) gm6020[1].total_ecd - wheel_angle[1]) > 2048)
//	{
//	  dirt[1] = 1;
//	  wheel_angle[1] = Angle_Limit (wheel_angle[1] - 4096, 8191);
//	}
//  else
//	dirt[1] = -1;
//
//  if (ABS ((fp32) gm6020[2].total_ecd - wheel_angle[2]) > 2048)
//	{
//	  dirt[2] = -1;
//	  wheel_angle[2] = Angle_Limit (wheel_angle[2] - 4096, 8191);
//	}
//  else
//	dirt[2] = 1;
//
//  if (ABS ((fp32) gm6020[3].total_ecd - wheel_angle[3]) > 2048)
//	{
//	  dirt[3] = 1;
//	  wheel_angle[3] = Angle_Limit (wheel_angle[3] - 4096, 8191);
//	}
//  else
//	dirt[3] = -1;
//
//  wheel0->ideal = nav.V[0];
//  wheel1->ideal = nav.V[1];
//  wheel2->ideal = nav.V[2];
//  wheel3->ideal = nav.V[3];

//  for (int i = 0; i < 4;)
//	{
//	  if (isnan(nav.theta[i]))
//		{
//		  nav.theta[i] = -180.0F;
//		}
//	  i++;
//	}

  nav.Vx *= 0.03F;
  nav.Vy *= -0.03F;
  nav.Vw *= -0.0001F;

  chassis_vector_to_M7010_wheel_angle (nav.Vy, nav.Vx, nav.Vw, nav.theta);
  chassis_vector_to_M3508_wheel_speed (nav.Vy, nav.Vx, nav.Vw, nav.V);

  wheel0->ideal = nav.V[0];
  wheel1->ideal = nav.V[1];
  wheel2->ideal = nav.V[2];
  wheel3->ideal = nav.V[3];

  servo0_pos->ideal = 0 + nav.theta[0];
  servo1_pos->ideal = 0 + nav.theta[1];
  servo2_pos->ideal = 0 + nav.theta[2];
  servo3_pos->ideal = 0 + nav.theta[3];
//
//  servo0_pos->ideal = DEFALT_DGR_0 + map_degree_to_8191 (nav.theta[0] * 57.2957795130823F * -0.5F);
//  servo1_pos->ideal = DEFALT_DGR_1 + map_degree_to_8191 (nav.theta[1] * 57.2957795130823F * -0.5F);
//  servo2_pos->ideal = DEFALT_DGR_2 + map_degree_to_8191 (nav.theta[2] * 57.2957795130823F * -0.5F);
//  servo3_pos->ideal = DEFALT_DGR_3 + map_degree_to_8191 (nav.theta[3] * 57.2957795130823F * -0.5F);

};

