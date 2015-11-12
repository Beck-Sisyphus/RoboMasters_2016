#ifndef __CAN2_H__
#define __CAN2_H__

#include <stm32f4xx.h>
#include "laser.h"
#include "can1.h"

#define pitch_max 15.0
#define yaw_max 720.0				//cyq:ÔÆÌ¨½Ç¶ÈµÄ·¶Î§

void CAN2_Configuration(void);
void GYRO_RST(void);
void Encoder_sent(float encoder_angle);
extern float YAW_Angle;
extern float dipan_gyro_angle;
extern uint8_t shooting_flag;
extern int8_t gyro_ok_flag;

#endif 
