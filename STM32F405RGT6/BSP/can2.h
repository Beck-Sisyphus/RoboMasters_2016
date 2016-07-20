#ifndef __CAN2_H__
#define __CAN2_H__

#include <stm32f4xx.h>
#include "CanBusTask.h"
#include "main.h"

void CAN2_Configuration(void);
void CAN2_RX0_IRQHandler(void);

#endif
