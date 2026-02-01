#ifndef BSP_FRIC_H
#define BSP_FRIC_H
#include "struct_typedef.h"

#define FRIC_UP 1720					// 范围为 1000-1800 1800大概为28m/s 右键提速射击
#define FRIC_DOWN 1400        // 左键射击（正常速度）  1600
#define FRIC_OFF 1000        //

extern void fric_off(void);
extern void fric1_on(uint16_t cmd);
extern void fric2_on(uint16_t cmd);
#endif
