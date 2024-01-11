//
// Created by zhpwa on 2024/1/11.
//

#include "bsp_buz.h"
#include "bsp_buz_defs.h"

#include "arm_math.h"

static bsp_buz_major_e buz_major = BSP_BUZ_MAJOR_C;

HAL_StatusTypeDef bsp_buz_init (void)
{
  HAL_StatusTypeDef ret = HAL_ERROR;
  ret = HAL_TIM_Base_Start (&htim4);
  ret |= HAL_TIM_PWM_Start (&htim4, TIM_CHANNEL_3);

  bsp_buz_init_pitch ();
  return ret;
}

int bsp_buz_init_pitch (void)
{
//  float standardFrequency = 440.0F;
//
//  float power = 0.0F;
//
//  for (int i = 0; i <= 127; i++)
//	{
////	  noteFrequency[i] = (standardFrequency / 32.0) * pow (2.0, (i - 9.0) / 12.0);
//	}
}

//static uint32_t

int bsp_buz_set_major (bsp_buz_major_e major)
{
  switch (major)
	{
	  default:
	  case BSP_BUZ_MAJOR_C:
	  case BSP_BUZ_MAJOR_D:
	  case BSP_BUZ_MAJOR_E:
	  case BSP_BUZ_MAJOR_F:
	  case BSP_BUZ_MAJOR_G:
	  case BSP_BUZ_MAJOR_A:
	  case BSP_BUZ_MAJOR_B: buz_major = major;
	}
}

int bsp_buz_apply_frequency (int frequency)
{
  uint32_t apb1ClockFreq = HAL_RCC_GetPCLK1Freq (); // 获取 APB1 时钟频率
  uint32_t timClockFreq = apb1ClockFreq * 2;      // 定时器时钟频率

  uint32_t psc = 0;
  uint32_t arr = UINT16_MAX + 1;

  return frequency;
}

int bsp_buz_set_pitch (bsp_buz_tone_e tone)
{
  int frequency = 0;

  // 音阶到 MIDI 音高的半音步映射，减去 1 以适应从 1 开始的枚举
  int semitoneOffsets[] = {0, 2, 4, 5, 7, 9, 11}; // DO, RE, MI, FA, SOL, LA, SI

  // 根据 tone 枚举减去 1 来获取正确的半音步偏移
  int toneOffset = (tone > 0 && tone <= 7) ? semitoneOffsets[tone - 1] : 0;

  // 计算目标音阶的 MIDI 音高值
  int targetNoteMidi = buz_major + toneOffset;

  // 防止数组越界
  if (targetNoteMidi >= 0 && targetNoteMidi < 128)
	{
	  frequency = noteFrequency[targetNoteMidi];
	}

  htim4.Instance->PSC = psc_table[targetNoteMidi]; // 更新预分频器
  htim4.Instance->ARR = arr_table[targetNoteMidi];     // 更新自动重装载寄存器

  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, arr_table[targetNoteMidi] / 16);

  return bsp_buz_apply_frequency (frequency);
}
