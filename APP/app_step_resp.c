//
// Created by zhpwa on 2024/1/10.
//
#if USE_STEP_RESPONSE == 0 /*defined in freertos.c*/
#include "app_step_resp.h"

static float step_response_global_variable = 0.0F;

float get_bsp_pid_step_response_target (void)
{
  return step_response_global_variable;
}

void set_bsp_pid_step_response_target (float set)
{
  step_response_global_variable = set;
}

#endif