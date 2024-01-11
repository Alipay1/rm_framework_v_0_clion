//
// Created by zhpwa on 2024/1/11.
//

#ifndef BSP_BUZ_H_
#define BSP_BUZ_H_

#include "tim.h"

typedef enum {
  BSP_BUZ_MAJOR_C = 60,
  BSP_BUZ_MAJOR_D = 62,
  BSP_BUZ_MAJOR_E = 64,
  BSP_BUZ_MAJOR_F = 65,
  BSP_BUZ_MAJOR_G = 67,
  BSP_BUZ_MAJOR_A = 69,
  BSP_BUZ_MAJOR_B = 71,
} bsp_buz_major_e;
typedef enum {
  BSP_BUZ_TONE_DO = 1,
  BSP_BUZ_TONE_RE,
  BSP_BUZ_TONE_MI,
  BSP_BUZ_TONE_FA,
  BSP_BUZ_TONE_SOL,
  BSP_BUZ_TONE_LA,
  BSP_BUZ_TONE_SI,
} bsp_buz_tone_e;

HAL_StatusTypeDef bsp_buz_init (void);
int bsp_buz_init_pitch (void);
int bsp_buz_set_major (bsp_buz_major_e major);
int bsp_buz_apply_frequency (int frequency);
int bsp_buz_set_pitch (bsp_buz_tone_e tone);
void bsp_buz_mute (void);
void bsp_buz_start (void);

#endif //BSP_BUZ_H_
