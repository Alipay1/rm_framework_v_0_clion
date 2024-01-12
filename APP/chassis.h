#ifndef __CHASSIS_TASK_H__
#define __CHASSIS_TASK_H__

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "INS_task.h"

#define fp32 float
#define fp64 double

#define LIMIT(x, min, max) (((x)<=(min))?(min):(((x)>=(max))?(max):(x)))
#define ABS(x) (((x)>0)?(x):(-(x)))

#define CHASSIS_TASK_INIT_TIME    1000
#define CHASSIS_CONTROL_TIME        2            //射击任务间隔时间

//底盘跟随云台
#define CAR_FOLLOW_GIMBAL_KP 0.00015f
#define CAR_FOLLOW_GIMBAL_KI 0.0f
#define CAR_FOLLOW_GIMBAL_KD 0.0f
#define CAR_FOLLOW_GIMBAL_I_LIMIT 250.f
#define CAR_FOLLOW_GIMBAL_MAX 2000.f
static const fp32 PID_CAR_FOLLOW_GIMBAL[] = {CAR_FOLLOW_GIMBAL_KP, CAR_FOLLOW_GIMBAL_KI, CAR_FOLLOW_GIMBAL_KD};

typedef enum {
  CHASSIS_ZERO_FORCE = 0, //无力模式
  CHASSIS_RC_GYROSCOPE = 1, //遥控器 小陀螺模式
  CHASSIS_RC_FOLLOW_GIMBAL = 2, //遥控器 跟随云台模式
  CHASSIS_PC_CONTROL = 3, //PC 键鼠模式

} CHASSIS_MODE_e; //底盘模式

extern fp32 vx_set, vy_set, wz_set;
extern fp32 PID_CurrentLT1, PID_CurrentLT2, PID_CurrentRT1, PID_CurrentRT2;
extern fp32 M3508_SPEED[4], MS7010_ANGLE[4];

void pid_chassis_all_init (void);
void chassis_vector_to_M3508_wheel_speed (fp32 vx_set, fp32 vy_set, fp32 wz_set, fp32 wheel_speed[4]);
void chassis_vector_to_M7010_wheel_angle (fp32 vx_set, fp32 vy_set, fp32 wz_set, fp32 wheel_angle[4]);

void chassis_mode_switch (void); //模式选择函数
void chassis_target_calc (uint8_t Mode); //Target计算函数
void chassis_calc_cmd (uint8_t Mode); //底盘PID计算及输出后函数

float Angle_Limit (float angle, float max);

#endif

