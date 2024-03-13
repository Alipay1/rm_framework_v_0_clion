//
// Created by zhpwa on 2024/1/12.
//

#include <math.h>
#include "app_nav.h"
#include "app_rc.h"
#include "arm_math.h"
#include "app_pid.h"
#include "chassis.h"
#include "bsp_isr.h"
#include "usart.h"

//-RC->rc.ch[2] Vy
//RC->rc.ch[3]  Vx
//RC->rc.ch[0]  Vw

static app_nav_t nav = {0};
gb_ecd_data *data;
__attribute__((unused)) int ecd_2 = 0;
extern RC_ctrl_t rc_ctrl;

float map_degree_to_8191 (float input)
{
  return input * 22.7527777777777777F;
}

app_nav_t *get_navigation_p (void)
{
  return &nav;
}

double radian_1 = 0;
double radian_2 = 0;
double radian_3 = 0;
double radian_4 = 0;
double TR_radian_xy = 0;
double TR_radian_1 = 0;
double TR_radian_2 = 0;
double TR_radian_3 = 0;
double TR_radian_4 = 0;
double radian_xy = 0;
double ag = 0;
double v1, v2, v3, v4 = 0;
double z1, z2, z9, z0 = 0;
int flag, num = 1;
//fp64 atan_angle[4];
//
float dirt[4] = {4, 4, 4, 4};

//#define MS7010_FL_ANGLE (3514 + 1024)    //装上去的时候Y轴正向对应的编码值(偏置角度)
//#define MS7010_FR_ANGLE (1714  -1024)
//#define MS7010_BL_ANGLE (3751 -1024)
//#define MS7010_BR_ANGLE (4239 +1024)
int w, s, a, d, q, e, ctrl;

void remoteKeyDecode ()
{
  w = (0x0001 & (rc_ctrl.key.v >> 0));
  s = -(0x0001 & (rc_ctrl.key.v >> 1));
  a = -(0x0001 & (rc_ctrl.key.v >> 2));
  d = (0x0001 & (rc_ctrl.key.v >> 3));
  q = -(0x0001 & (rc_ctrl.key.v >> 6));
  e = (0x0001 & (rc_ctrl.key.v >> 7));
  ctrl = (0x0001 & (rc_ctrl.key.v >> 5));
}

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

  motor_measure_t *motor4 = (motor_measure_t *) get_measure_pointer (4);
  motor_measure_t *motor5 = (motor_measure_t *) get_measure_pointer (5);
  motor_measure_t *motor6 = (motor_measure_t *) get_measure_pointer (6);
  motor_measure_t *motor7 = (motor_measure_t *) get_measure_pointer (7);

  data = get_ecd_data_from_gb_pointer ();
  remoteKeyDecode ();
  ecd_2 = data->ecd_1.ecd;
  ag = (ecd_2 - 761.0) * PI * 2 / 8192;
  if (rc_ctrl.rc.s[1] == 1)
	{
	  if (rc_ctrl.rc.s[0] == 1)
		{
		  nav.Vx = (double) rc_ctrl.rc.ch[2];
		  nav.Vy = (double) rc_ctrl.rc.ch[3];
		  nav.Vw = (double) rc_ctrl.rc.ch[0];
		}
	  if (rc_ctrl.rc.s[0] == 3)
		{
		  nav.Vx = (double) rc_ctrl.rc.ch[2];
		  nav.Vy = (double) rc_ctrl.rc.ch[3];
		  nav.Vw = 500;
		}
	  if (rc_ctrl.rc.s[0] == 2)
		{
		  nav.Vy = w * 500 + s * 500;
		  nav.Vx = d * 500 + a * 500;
		  nav.Vw = q * 500 + e * 500;
		}
	}
  if (rc_ctrl.rc.s[1] == 2)
	{
	  nav.Vy = 0;
	  nav.Vx = 0;
	  nav.Vw = 0;

	}
  TR_radian_1_3 = (8192 - 760.0) * PI * 2 / 8192;
  radian_xy = atan2 (nav.Vy, nav.Vx);
  TR_radian_xy = radian_xy + ag;
  TR_radian_1 = 3 * PI / 4 - radian_xy - ag;
  TR_radian_2 = 5 * PI / 4 - radian_xy - ag;
  TR_radian_3 = PI / 4 + radian_xy + ag;
  TR_radian_4 = 3 * PI / 4 - radian_xy + ag;
  TR_radian_xy = radian_xy + ag;
  z1 = sqrt (pow (nav.Vx, 2) + pow (nav.Vy, 2)) * cos (PI - TR_radian_xy);
  z2 = sqrt (pow (nav.Vx, 2) + pow (nav.Vy, 2)) * sin (PI - TR_radian_xy);

  if (ag < 0)
	{
	  TR_radian_xy = radian_xy + ag;
	  z9 = sqrt (pow (nav.Vx, 2) + pow (nav.Vy, 2)) * cos (3 * PI / 2 + TR_radian_xy);
	  z0 = sqrt (pow (nav.Vx, 2) + pow (nav.Vy, 2)) * sin (3 * PI / 2 + TR_radian_xy);
	  TR_radian_1 = 3 * PI / 4 - radian_xy - ag + 3 * PI / 2;
	}

  radian_1 = atan2 (z2 + nav.Vw * sin (-PI / 4 - PI / 2),
					z1 + nav.Vw * cos (-PI / 4 - PI / 2)) + PI / 8 + PI / 2;
  v1 = sqrt (pow (nav.Vx, 2) + pow (nav.Vy, 2) + pow (nav.Vw, 2)
			 - 2 * sqrt (pow (nav.Vx, 2) + pow (nav.Vy, 2)) * nav.Vw * cos (TR_radian_1));

  radian_2 = atan2 (z2 + nav.Vw * sin (PI / 4 + PI / 2),
					z1 + nav.Vw * cos (PI / 4 + PI / 2)) + PI / 6 - PI / 2;
  v2 = sqrt (pow (nav.Vx, 2) + pow (nav.Vy, 2) + pow (nav.Vw, 2)
			 - 2 * sqrt (pow (nav.Vx, 2) + pow (nav.Vy, 2)) * nav.Vw * cos (TR_radian_2));

  radian_3 = atan2 (z2 + nav.Vw * sin (3 * PI / 4 - PI / 2),
					z1 + nav.Vw * cos (3 * PI / 4 - PI / 2)) + PI / 7 + PI / 2;
  v3 = sqrt (pow (nav.Vx, 2) + pow (nav.Vy, 2) + pow (nav.Vw, 2)
			 - 2 * sqrt (pow (nav.Vx, 2) + pow (nav.Vy, 2)) * nav.Vw * cos (TR_radian_3));

  radian_4 = atan2 (z2 + nav.Vw * sin (5 * PI / 4 + PI / 2),
					z1 + nav.Vw * cos (5 * PI / 4 + PI / 2)) - PI / 6 - PI / 2;
  v4 = sqrt (pow (nav.Vx, 2) + pow (nav.Vy, 2) + pow (nav.Vw, 2)
			 - 2 * sqrt (pow (nav.Vx, 2) + pow (nav.Vy, 2)) * nav.Vw * cos (TR_radian_4));

  if (ag < 0)
	{
	  radian_1 = atan2 (z9 + nav.Vw * sin (-PI / 4 - PI / 2),
						z0 + nav.Vw * cos (-PI / 4 - PI / 2)) + PI / 8 + PI / 2;
	  v1 = sqrt (pow (nav.Vx, 2) + pow (nav.Vy, 2) + pow (nav.Vw, 2)
				 - 2 * sqrt (pow (nav.Vx, 2) + pow (nav.Vy, 2)) * nav.Vw * cos (TR_radian_1));

	  radian_2 = atan2 (z9 + nav.Vw * sin (PI / 4 + PI / 2),
						z0 + nav.Vw * cos (PI / 4 + PI / 2)) + PI / 6 - PI / 2;
	  v2 = sqrt (pow (nav.Vx, 2) + pow (nav.Vy, 2) + pow (nav.Vw, 2)
				 - 2 * sqrt (pow (nav.Vx, 2) + pow (nav.Vy, 2)) * nav.Vw * cos (TR_radian_2));

	  radian_3 = atan2 (z9 + nav.Vw * sin (3 * PI / 4 - PI / 2),
						z0 + nav.Vw * cos (3 * PI / 4 - PI / 2)) + PI / 7 + PI / 2;
	  v3 = sqrt (pow (nav.Vx, 2) + pow (nav.Vy, 2) + pow (nav.Vw, 2)
				 - 2 * sqrt (pow (nav.Vx, 2) + pow (nav.Vy, 2)) * nav.Vw * cos (TR_radian_3));

	  radian_4 = atan2 (z9 + nav.Vw * sin (5 * PI / 4 + PI / 2),
						z0 + nav.Vw * cos (5 * PI / 4 + PI / 2)) - PI / 6 - PI / 2;
	  v4 = sqrt (pow (nav.Vx, 2) + pow (nav.Vy, 2) + pow (nav.Vw, 2)
				 - 2 * sqrt (pow (nav.Vx, 2) + pow (nav.Vy, 2)) * nav.Vw * cos (TR_radian_4));
	}

  while (radian_1 < 0)radian_1 += 2 * PI;
  while (radian_1 > 2 * PI)radian_1 -= 2 * PI;
  while (radian_2 < 0)radian_2 += 2 * PI;
  while (radian_2 > 2 * PI)radian_2 -= 2 * PI;
  while (radian_2 < 0)radian_2 += 2 * PI;
  while (radian_2 > 2 * PI)radian_2 -= 2 * PI;
  while (radian_2 < 0)radian_2 += 2 * PI;
  while (radian_2 > 2 * PI)radian_2 -= 2 * PI;

//    if (radian_1 == PI / 8 + PI / 2 && num == 1) {
//        num = 0;
//    }
//    if (radian_2 == 0) { radian_2 = last_radian_2; }
//    if (radian_3 == 0) { radian_3 = last_radian_3; }
//    if (radian_4 == 0) { radian_4 = last_radian_4; }
  servo0_pos->ideal = map_degree_to_8191 (radian_1 * 57.2957795130823F);
  servo1_pos->ideal = map_degree_to_8191 (radian_2 * 57.2957795130823F);
  servo2_pos->ideal = map_degree_to_8191 (radian_3 * 57.2957795130823F);
  servo3_pos->ideal = map_degree_to_8191 (radian_4 * 57.2957795130823F);

//    if (radian_1 != PI / 8 + PI / 2) {
//        num = 1;
//        radian_1 = last_radian_1;
//    }
//    if (radian_2 == 0) { radian_2 = last_radian_2; }
//    if (radian_3 == 0) { radian_3 = last_radian_3; }
//    if (radian_4 == 0) { radian_4 = last_radian_4; }
//
//    last_radian_1 = radian_1;
//
//
//    last_radian_2 = radian_2;
//    last_radian_3 = radian_3;
//    last_radian_4 = radian_4;

//    if (nav.Vx != vx && nav.Vy != vy) {
  while (servo0_pos->actual - servo0_pos->ideal < -4096)
	{
	  servo0_pos->ideal = -8192 + servo0_pos->ideal;
//        if (nav.Vx == vx || nav.Vy == vy)
//            dirt[0] *= -1;
	}

  while (servo0_pos->actual - servo0_pos->ideal > 4096)
	{
	  servo0_pos->ideal = 8192 + servo0_pos->ideal;
//        if (nav.Vx == vx || nav.Vy == vy)
//            dirt[0] *= -1;
	}

  while (servo1_pos->actual - servo1_pos->ideal < -4095)
	{
	  servo1_pos->ideal = -8192 + servo1_pos->ideal;
	}
  while (servo1_pos->actual - servo1_pos->ideal > 4095)
	{
	  servo1_pos->ideal = 8192 + servo1_pos->ideal;
	}
  while (servo2_pos->actual - servo2_pos->ideal > 4095)
	{
	  servo2_pos->ideal = 8192 + servo2_pos->ideal;
	}
  while (servo2_pos->actual - servo2_pos->ideal < -4095)
	{
	  servo2_pos->ideal = -8192 + servo2_pos->ideal;
	}
  while (servo3_pos->actual - servo3_pos->ideal < -4095)
	{
	  servo3_pos->ideal = -8192 + servo3_pos->ideal;
	}
  while (servo3_pos->actual - servo3_pos->ideal > 4095)
	{
	  servo3_pos->ideal = 8192 + servo3_pos->ideal;
	}
  wheel0->ideal = v1 * dirt[0];
  wheel1->ideal = v2 * dirt[1];
  wheel2->ideal = v3 * dirt[2];
  wheel3->ideal = v4 * dirt[3];

//    if (abs(servo0_pos->actual - servo0_pos->ideal) > 4095) {
//        servo0_pos->ideal = -8192 * (flag + 1) + servo0_pos->ideal;
//    num = 1;



//    }
//    if (num == 1) {
//        if (servo0_pos->actual - servo0_pos->ideal < 4900 && servo0_pos->actual - servo0_pos->ideal > 0) {
//            num = 0;
//        }
//        if (servo0_pos->actual - servo0_pos->ideal > -4900 && servo0_pos->actual - servo0_pos->ideal < 0) {
//            num = 0;
//        }
//
//    }



}

