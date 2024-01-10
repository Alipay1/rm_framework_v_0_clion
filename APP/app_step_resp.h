//
// Created by zhpwa on 2024/1/10.
//
#if USE_STEP_RESPONSE == 0 /*defined in freertos.c*/
#ifndef APP_STEP_RESP_H_
#define APP_STEP_RESP_H_

float get_bsp_pid_step_response_target (void);
void set_bsp_pid_step_response_target (float set);

#endif //APP_STEP_RESP_H_
#endif