#include "chassis.h"
#include "arm_math.h"

#include "app_rc.h"
#include "app_pid.h"

extern RC_ctrl_t rc_ctrl;
typedef struct {
  uint8_t mode;
  //PID 三参数
  fp32 Kp;
  fp32 Ki;
  fp32 Kd;

  fp32 max_out;  //最大输出
  fp32 max_iout; //最大积分输出

  fp32 set;
  fp32 fdb;

  fp32 out;
  fp32 Pout;
  fp32 Iout;
  fp32 Dout;
  fp32 Dbuf[3];  //微分项 0最新 1上一次 2上上次
  fp32 error[3]; //误差项 0最新 1上一次 2上上次

} PidTypeDef;
PidTypeDef pid_car_lt1, pid_car_lt2, pid_car_rt1, pid_car_rt2, pid_car_follow_gimbal;

fp32 PID_CurrentLT1, PID_CurrentLT2, PID_CurrentRT1, PID_CurrentRT2;
fp32 PID_MS7010_Speed_Current_FL, PID_MS7010_Speed_Current_FR, PID_MS7010_Speed_Current_BL, PID_MS7010_Speed_Current_BR;
fp32 PID_MS7010_Current_FL, PID_MS7010_Current_FR, PID_MS7010_Current_BL, PID_MS7010_Current_BR;

fp32 M3508_SPEED[4], MS7010_ANGLE[4];
fp32 ms7010_speed[4];

fp32 relative_angle_set;

fp32 vx_set, vy_set, wz_set;

#define Motor_Ecd_to_Rad 0.000766990394f //2 * PI / 8192

#define WHEEL_PERIMETER             364.4247478164160156f    //车轮周长 (轮子直径 * PI 再转换成mm)
#define M3508_RATIO                     19                //电机减速比
#define Radius                                 116                //轮径mm

#define RC_SW_RIGHT (1)
#define RC_SW_LEFT (0)

//四个驱动电机朝向前方时 转向电机初始编码值
//#define MS7010_FL_ANGLE 0        //装上去的时候Y轴正向对应的编码值(偏置角度)
//#define MS7010_FR_ANGLE 0
//#define MS7010_BL_ANGLE 0
//#define MS7010_BR_ANGLE 0


//#define MS7010_FL_ANGLE  6826   //装上去的时候Y轴正向对应的编码值(偏置角度)4643 - 100 +4096
//#define MS7010_FR_ANGLE 6641
//#define MS7010_BL_ANGLE 2670
//#define MS7010_BR_ANGLE 5456

//#define MS7010_FL_ANGLE  448   //装上去的时候Y轴正向对应的编码值(偏置角度)4643 - 100 +4096
//#define MS7010_FR_ANGLE 689 + 4096
//#define MS7010_BL_ANGLE 4742+  2048
//#define MS7010_BR_ANGLE 3401 -  2048

//#define MS7010_FL_ANGLE 4643 - 100    //装上去的时候Y轴正向对应的编码值(偏置角度)
//#define MS7010_FR_ANGLE 689 + 310
//#define MS7010_BL_ANGLE 4742
//#define MS7010_BR_ANGLE 3401

//#define MS7010_FL_ANGLE 689     //装上去的时候Y轴正向对应的编码值(偏置角度)
//#define MS7010_FR_ANGLE 4643
//#define MS7010_BL_ANGLE 4742
//#define MS7010_BR_ANGLE 3401

#define MS7010_FL_ANGLE 3514 + 1024    //装上去的时候Y轴正向对应的编码值(偏置角度)
#define MS7010_FR_ANGLE 1714  -1024
#define MS7010_BL_ANGLE 3751 -1024
#define MS7010_BR_ANGLE 4239 +1024

//#define DEFALT_DGR_0 3514
//#define DEFALT_DGR_1 1714
//#define DEFALT_DGR_2 3751
//#define DEFALT_DGR_3 4239

//底盘控制模式
uint8_t CHASSIS_MODE;

void Chassis_Task (void const *argument)
{
  vTaskDelay (CHASSIS_TASK_INIT_TIME);

  //初始化底盘模式为 底盘无力模式
  CHASSIS_MODE = CHASSIS_ZERO_FORCE;

  pid_chassis_all_init (); //pid参数初始化

  while (1)
	{
	  chassis_mode_switch ();  //根据遥控器 选择底盘控制模式
	  chassis_target_calc (CHASSIS_MODE); //根据不同模式 计算控制目标值
	  chassis_calc_cmd (CHASSIS_MODE); //PID计算并输出

	  vTaskDelay (CHASSIS_CONTROL_TIME);

	}

}

//将电机转子转向内侧时 修正方向
int8_t dirt[4] = {1, -1, 1, -1};
void chassis_vector_to_M3508_wheel_speed (fp32 vx_set, fp32 vy_set, fp32 wz_set, fp32 wheel_speed[4])
{
  fp32 wheel_rpm_ratio;

  wheel_rpm_ratio = 60.0f / (WHEEL_PERIMETER * 3.14159f) * M3508_RATIO * 1000;

  wheel_speed[0] = dirt[0] * sqrt (pow (vy_set + wz_set * Radius * 0.707107f, 2)
								   + pow (vx_set - wz_set * Radius * 0.707107f, 2)
  ) * wheel_rpm_ratio;
  wheel_speed[1] = dirt[1] * sqrt (pow (vy_set - wz_set * Radius * 0.707107f, 2)
								   + pow (vx_set - wz_set * Radius * 0.707107f, 2)
  ) * wheel_rpm_ratio;
  wheel_speed[2] = dirt[2] * sqrt (pow (vy_set - wz_set * Radius * 0.707107f, 2)
								   + pow (vx_set + wz_set * Radius * 0.707107f, 2)
  ) * wheel_rpm_ratio;
  wheel_speed[3] = dirt[3] * sqrt (pow (vy_set + wz_set * Radius * 0.707107f, 2)
								   + pow (vx_set + wz_set * Radius * 0.707107f, 2)
  ) * wheel_rpm_ratio;

}

fp64 atan_angle[4];
void chassis_vector_to_M7010_wheel_angle (fp32 vx_set, fp32 vy_set, fp32 wz_set, fp32 wheel_angle[4])
{
  PID *servo0_pos = pid_get_struct_pointer (4 + 4, NORMAL_MOTOR);/*4 + 4 indicates motor5(4) -> PID -> position(+4)*/
  motor_measure_t *gm6020 = get_measure_pointer (4);

  //7010目标角度计算
  if (!(vx_set == 0 && vy_set == 0 && wz_set == 0))//防止除数为零
	{
	  //由于atan2算出来的结果是弧度，需转换成角度 计算公式为 弧度 * 180.f / PI 最终得到角度值 (0.707107f == 根号2)
	  atan_angle[0] =
		  atan2 ((vx_set - wz_set * Radius * 0.707107f), (vy_set + wz_set * Radius * 0.707107f)) * 180.0f / PI;
	  atan_angle[1] =
		  atan2 ((vx_set - wz_set * Radius * 0.707107f), (vy_set - wz_set * Radius * 0.707107f)) * 180.0f / PI;
	  atan_angle[2] =
		  atan2 ((vx_set + wz_set * Radius * 0.707107f), (vy_set + wz_set * Radius * 0.707107f)) * 180.0f / PI;
	  atan_angle[3] =
		  atan2 ((vx_set + wz_set * Radius * 0.707107f), (vy_set - wz_set * Radius * 0.707107f)) * 180.0f / PI;
	}

  // 将一圈360°转换成编码值的一圈0-8191 -> 角度 * 8191 / 360 最终转换为需要转动的角度对应的编码值，再加上偏置角度,最终得到目标编码值
  wheel_angle[0] = Angle_Limit (MS7010_FL_ANGLE + (fp32) (atan_angle[0] * 22.75277777777f), 8191.f);
  wheel_angle[1] = Angle_Limit (MS7010_FR_ANGLE + (fp32) (atan_angle[1] * 22.75277777777f), 8191.f);
  wheel_angle[2] = Angle_Limit (MS7010_BL_ANGLE + (fp32) (atan_angle[2] * 22.75277777777f), 8191.f);
  wheel_angle[3] = Angle_Limit (MS7010_BR_ANGLE + (fp32) (atan_angle[3] * 22.75277777777f), 8191.f);

  //优弧 劣弧 驱动电机转向判断
  if (ABS ((fp32) gm6020[0].total_ecd - wheel_angle[0]) > 2048)
	{
	  dirt[0] = -1;
	  wheel_angle[0] = Angle_Limit (wheel_angle[0] - 4096, 8191);
	}
  else
	dirt[0] = 1;

  if (ABS ((fp32) gm6020[1].total_ecd - wheel_angle[1]) > 2048)
	{
	  dirt[1] = 1;
	  wheel_angle[1] = Angle_Limit (wheel_angle[1] - 4096, 8191);
	}
  else
	dirt[1] = -1;

  if (ABS ((fp32) gm6020[2].total_ecd - wheel_angle[2]) > 2048)
	{
	  dirt[2] = -1;
	  wheel_angle[2] = Angle_Limit (wheel_angle[2] - 4096, 8191);
	}
  else
	dirt[2] = 1;

  if (ABS ((fp32) gm6020[3].total_ecd - wheel_angle[3]) > 2048)
	{
	  dirt[3] = 1;
	  wheel_angle[3] = Angle_Limit (wheel_angle[3] - 4096, 8191);
	}
  else
	dirt[3] = -1;

}

void chassis_mode_switch (void) //模式选择函数
{
  if (switch_is_mid (rc_ctrl.rc.s[RC_SW_RIGHT]) || switch_is_up (rc_ctrl.rc.s[RC_SW_RIGHT]))
	{
	  if (switch_is_down (rc_ctrl.rc.s[RC_SW_LEFT]))        //左侧拨杆在下面 遥控器模式
		CHASSIS_MODE = CHASSIS_RC_GYROSCOPE;
	  if (switch_is_mid (rc_ctrl.rc.s[RC_SW_LEFT]))            //左侧拨杆在中间 底盘跟随云台
		CHASSIS_MODE = CHASSIS_RC_FOLLOW_GIMBAL;
	  if (switch_is_up (rc_ctrl.rc.s[RC_SW_LEFT]))            //左侧拨杆在上面 PC模式
		CHASSIS_MODE = CHASSIS_PC_CONTROL;
	}
  else //右侧拨杆在下面 云台无力模式
	CHASSIS_MODE = CHASSIS_ZERO_FORCE;
}

//void chassis_target_calc (uint8_t Mode) //Target计算函数
//{
//
//  if (Mode == CHASSIS_PC_CONTROL) //PC模式
//	{ ;
//	}
//  else if (Mode == CHASSIS_RC_GYROSCOPE) //遥控器 小陀螺模式
//	{
//	  int16_t world_vx_set = -rc_ctrl.rc.ch[2] / 200.f;
//	  int16_t world_vy_set = rc_ctrl.rc.ch[3] / 200.f;
//
//	  //跟随的云台角度
//	  fp32 theta = (gimbal_measure[1].ecd - 6120) * Motor_Ecd_to_Rad;
//
//	  fp32 sin_yaw = arm_sin_f32 (theta);
//	  fp32 cos_yaw = arm_cos_f32 (theta);
//
//	  vx_set = cos_yaw * world_vx_set - sin_yaw * world_vy_set;
//	  vy_set = sin_yaw * world_vx_set + cos_yaw * world_vy_set;
//
//	  wz_set = -1;
//
//	}
//  else if (Mode == CHASSIS_RC_FOLLOW_GIMBAL) //底盘跟随云台模式
//	{
//	  vx_set = -rc_ctrl.rc.ch[2] / 200.f;
//	  vy_set = rc_ctrl.rc.ch[3] / 200.f;
//
//	  wz_set = PID_Calc_Ecd (&pid_car_follow_gimbal, gimbal_measure[0].ecd, 6120, 8191);
//	}
//  else if (Mode == CHASSIS_ZERO_FORCE) //无力模式
//	{ ;
//	}
//
//}

//将角度范围控制在 0 - 8191
float Angle_Limit (float angle, float max)
{
  if (angle > max)
	angle -= max;
  if (angle < 0)
	angle += max;
  return angle;
}
