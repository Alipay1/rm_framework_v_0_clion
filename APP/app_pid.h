//
// Created by zhpwa on 2024/1/10.
//

#ifndef APP_PID_H_
#define APP_PID_H_

#include "main.h"
#include "stdbool.h"
#include "bsp_can.h"

#include "arm_math.h"

/*total number of PID structs*/
#define PID_SPEED_STRUCT_NUM 12U

extern int GLOBAL_INIT;
extern float g_pitch;

typedef struct pid_struct {
  int motor_number;
  bool active;
  float ideal;
  float actual;
  float output;
  float integral;
  float err;
  float err_last;
  float err_last_last;
  float Kp;
  float Ki;
  float Kd;

  float Pout;
  float Iout;
  float Dout;

  float Limit_Iout;
  float Limit_Out;

  float Kib;

  uint8_t Error;
} PID;

typedef enum {
  NONE_ERR = 0X00,        // 0000 0000
  MOTOR_STALL = 0x01,     // 0000 0001
  MOTOR_OVER_HEAT = 0x02, // 0000 0010
  //    NONE = 0x04,         // 0000 0100
  //    NONE = 0x08, // 0000 1000
  //    NONE = 0x10,                // 0001 0000
  //    NONE = 0x20,     // 0010 0000
  //    NONE = 0x40,            // 0100 0000
  //    NONE = 0x80,                 // 1000 0000
} motor_error_t;

enum {
  NORMAL_MOTOR = 0,
  PITCH_MOTOR = 5,
  YAW_MOTOR = 6,
};

int PID_Setup (void);
int PID_Calculate (void);

float abs_f (float value);

float32_t PID_PITCH_Union_Calculate (PID *pxStructV, PID *pxStructA, float absolute_angle);
float32_t PID_YAW_Union_Calculate (PID *pxStructV, PID *pxStructP, float position, bool mode);
PID *pid_get_struct_pointer (uint32_t num, uint32_t in_array);
int32_t get_yaw_motor_position (void);

int pid_sscanf (char *input);

#endif //APP_PID_H_
