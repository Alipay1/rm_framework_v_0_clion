//
// Created by zhpwa on 2024/1/10.
//

#ifndef _BSP_CAN_H_
#define _BSP_CAN_H_

#include "main.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

#define PITCH_MOTOR_NUM 4 /*从零计数所以是第五个*/
#define YAW_MOTOR_NUM 5   /*从零计数所以是第六个*/

#define BSP_CAN_TOTAL_MORTOR_COUNT 8

#define MOUSE_FREQ 200
#define MOUSE_PERIOD (float)(1 / MOUSE_FREQ)

#define ENCODER_MAX_VALUE 8191
#define ENCODER_THRESHOLD (ENCODER_MAX_VALUE / 2)

#define bsp_can_get_motor_measure(ptr, data)                                   \
    {                                                                  \
        (ptr)->last_last_ecd = (ptr)->last_ecd;                        \
        (ptr)->last_ecd = (ptr)->ecd;                                  \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);           \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);     \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]); \
        (ptr)->temperate = (data)[6];                                  \
    }


#define bsp_can_get_rc_from_chassis(ptr, data)                          \
    {                                                           \
        (ptr)->mouse.x = (int16_t)((data)[1] << 8 | (data)[0]); \
        (ptr)->mouse.y = (int16_t)((data)[3] << 8 | (data)[2]); \
        (ptr)->mouse.press_l = (data)[4];                       \
        (ptr)->mouse.press_r = (data)[5];                       \
        (ptr)->key.v = (uint16_t)((data)[7] << 8 | (data)[6]);  \
                                                                \
        (ptr)->mouse_integeral.x += (float)(ptr)->mouse.x;      \
        (ptr)->mouse_integeral.y += (float)(ptr)->mouse.y;      \
        (ptr)->mouse_integeral.z += (float)(ptr)->mouse.z;      \
    }

typedef enum {
    USING_CAN1 = 1,
    USING_CAN2 = 2
} can_select;

typedef enum {
    CAN_CHASSIS_ALL_ID = 0x200,

    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,

    CAN_YAW_MOTOR_ID = 0x205, /*6020 1*/
    CAN_PIT_MOTOR_ID = 0x206, /*6020 2*/
    CAN_TRIGGER_MOTOR_ID = 0x207,

    CAN_GIMBAL_ALL_ID = 0x1FF,

    CHASSIS_CONTROLLER = 0x519,

    ECD_REPORT_ID = 0x520,
    GB_R = 0x119,
    ch_set = 0x120,
    AMMO_BOOSTER = 0x521,

    RC_SWITCH = 0x420,

} can_msg_id;

typedef enum {
    CAN_CHANNEL_1 = 1,
    CAN_CHANNEL_2 = 2
} can_channel_id;

typedef enum {
    MOTOR_1234 = 1,
    MOTOR_5678 = 2,
    ECD_REPORT = 3,
} motor_id_range;

typedef struct {
    int16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
    int16_t last_last_ecd;
    float total_ecd;
} motor_measure_t;

typedef struct {
    struct {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t press_l;
        uint8_t press_r;
    } mouse;

    struct {
        float x;
        float y;
        float z;
    } mouse_integral;

    struct {
        uint16_t v;
    } key;
} chassis_transported_controller_data;

typedef struct {
    uint8_t ID;
    uint8_t shooting_rate;
    uint16_t bullet_speed;
    uint8_t muzzle_heat;
    uint8_t muzzle_heat_lim;
    uint8_t muzzle_cooling_rate;
    uint8_t muzzle_speed_lim;
} ammo_booster_data;

/*
motor data,  0:chassis motor1 3508;1:chassis motor3 3508;2:chassis motor3 3508;3:chassis motor4 3508;
4:yaw gimbal motor 6020;5:pitch gimbal motor 6020;6:trigger motor 2006;
电机数据, 0:底盘电机1 3508电机,  1:底盘电机2 3508电机,2:底盘电机3 3508电机,3:底盘电机4 3508电机;
4:yaw云台电机 6020电机; 5:pitch云台电机 6020电机; 6:拨弹电机 2006电机*/

void CAN_FilterSetup(void);

motor_measure_t *get_measure_pointer(uint32_t i);

chassis_transported_controller_data *get_rc_data_from_chassis_pointer(void);

uint32_t
CAN_SendMessage(can_channel_id CAN_Channel, motor_id_range MOTOR_ID_RANGE, int16_t Motor1, int16_t Motor2,
                int16_t Motor3, int16_t Motor4);

ammo_booster_data *get_ammo_booster_info_ptr(void);

int32_t bsp_can_updateTotalAngle(uint32_t motor_num);

#endif //_BSP_CAN_H_
