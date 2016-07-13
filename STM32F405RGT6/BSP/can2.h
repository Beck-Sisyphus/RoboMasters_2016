#ifndef __CAN2_H__
#define __CAN2_H__

#include <stm32f4xx.h>
#include "laser.h"
#include "can1.h"
#include "usart1.h"
#include "usart3.h"

#define pitch_max 15.0
#define yaw_max 720.0

// For Red motor
// #define MOTOR_YAW 				1
// #define MOTOR_PITCH 			2
// #define MOTOR_FRONT_LEFT 		3
// #define MOTOR_BACK_LEFT 		4
// #define MOTOR_FRONT_RIGHT 		5
// #define MOTOR_BACK_RIGHT 		6

// For Blue motor and all 2016 soldier robots
#define MOTOR_YAW 				1
#define MOTOR_PITCH 			2
#define MOTOR_FRONT_RIGHT 		3
#define MOTOR_FRONT_LEFT 		4
#define MOTOR_BACK_LEFT 		5
#define MOTOR_BACK_RIGHT 		6

void CAN2_Configuration(void);

void Motor_Reset_Can_2(void);

void PitchYaw_Address_Setup(void);
void Wheels_Address_Setup(void);
void Set_PitchYaw_Current(void);
void Set_Wheels_Current(void);

void Motor_ManSet_Can_2(void);

void pitchyaw_control(int16_t, int16_t);
void wheel_control(int16_t, int16_t, int16_t, int16_t);

// void GYRO_RST(void);
// void Encoder_sent(float encoder_angle);

#endif
