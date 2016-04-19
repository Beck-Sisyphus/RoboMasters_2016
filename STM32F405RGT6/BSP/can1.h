#ifndef __CAN_H__
#define __CAN_H__

#include <stm32f4xx.h>
#include "delay.h"

#define abs(x) ((x)>0? (x):(-(x)))

extern volatile unsigned char OverCurr_flag;

void CAN1_Configuration(void);

#endif
