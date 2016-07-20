#ifndef __BSP_H__
#define __BSP_H__

#include <stm32f4xx.h>
#include "can1.h"
#include "can2.h"
#include "led.h"
#include "usart2.h"
#include "usart1.h"
#include "delay.h"
#include "pwm.h"
#include "laser.h"
#include "usart3.h"
#include "tim2.h"
#include "tim6.h"

void BSP_Init(void);

#endif
