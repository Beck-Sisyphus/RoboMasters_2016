#ifndef __REMOTE_CONTROL_H__
#define __REMOTE_CONTROL_H__
#include <stm32f4xx.h>


typedef enum
{
	NOSHOOTING = 0,
	SHOOTING = 1,
}Shoot_State_e;

void Remote_Control(void);
uint8_t IsRemoteBeingAction(void);
Shoot_State_e GetShootState(void);
int round_div(int, float);
#endif
