#ifndef __CAN2_H__
#define __CAN2_H__

#include <stm32f4xx.h>
#include "laser.h"
#include "can1.h"

#define pitch_max 15.0
#define yaw_max 720.0			

#define MOTOR_YAW 				1
#define MOTOR_PITCH 			2
#define MOTOR_FRONT_RIGHT 		3
#define MOTOR_FRONT_LEFT 		4
#define MOTOR_BACK_LEFT 		5
#define MOTOR_BACK_RIGHT 		6

extern float YAW_Angle;
extern float PITCH_Angle;

// yaw and pitch angle rx messages from CAN
extern uint16_t temp_yaw_angle;
extern uint16_t temp_pitch_angle;

extern uint16_t temp_yaw_current;
extern uint16_t temp_pitch_current;


extern float dipan_gyro_angle;
extern uint8_t shooting_flag;
extern int8_t gyro_ok_flag;

void CAN2_Configuration(void);
void GYRO_RST(void);
void Encoder_sent(float encoder_angle);

void Motor_Reset_Can_2(void);
void Motor_Current_Send(int, int);

void PitchYaw_Address_Setup(void);
void Wheels_Address_Setup(void);
void Set_PitchYaw_Current(void);
void Set_Wheels_Current(void);

void Motor_ManSet_Can_2(void);

#endif 
