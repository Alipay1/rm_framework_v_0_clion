//
// Created by zhpwa on 2024/1/9.
//

#ifndef C_BOARD_STANDARD_ROBOT_BSP_ISR_H
#define C_BOARD_STANDARD_ROBOT_BSP_ISR_H

#include <stdint-gcc.h>

typedef struct {
    struct {
        int16_t ecd;
    } ecd_1;
} gb_ecd_data;

#define get_rc_from_gb(ptr, data)                             \
  {                                                                \
    (ptr)->ecd_1.ecd = (int16_t)((data)[1] << 8 | (data)[0]);   \
  }


gb_ecd_data *get_ecd_data_from_gb_pointer(void);

#endif // C_BOARD_STANDARD_ROBOT_BSP_ISR_H
