#ifndef __CAN_H__
#define __CAN_H__

#include <stm32f4xx.h>
#include "delay.h"

#define abs(x) ((x)>0? (x):(-(x)))
#define MOTOR_NUM1                      0x01
#define MOTOR_NUM2                      0x02
#define MOTOR_NUM3                      0x03
#define MOTOR_NUM4                      0x04
#define MOTOR_NUM5                      0x05
#define MOTOR_NUM6                      0x06
#define MOTOR_NUM7                      0x07
#define MOTOR_NUM8                      0x08

#define PWM_MODE                        0x01
#define SPEED_MODE                      0x02
#define PWM_LOCATION_MODE               0x03
#define SPEED_LOCATION_MODE             0x04

#define ID_MOTOR1_CHOOSE_MODE           0x10
#define ID_MOTOR1_PWM_MODE              0x11
#define ID_MOTOR1_SPEED_MODE            0x12
#define ID_MOTOR1_PWM_LOCATION_MODE     0x13
#define ID_MOTOR1_SPEED_LOCATION_MODE   0x14 

#define ID_MOTOR2_CHOOSE_MODE           0x20
#define ID_MOTOR2_PWM_MODE              0x21
#define ID_MOTOR2_SPEED_MODE            0x22
#define ID_MOTOR2_PWM_LOCATION_MODE     0x23
#define ID_MOTOR2_SPEED_LOCATION_MODE   0x24 

#define ID_MOTOR3_CHOOSE_MODE           0x30
#define ID_MOTOR3_PWM_MODE              0x31
#define ID_MOTOR3_SPEED_MODE            0x32
#define ID_MOTOR3_PWM_LOCATION_MODE     0x33
#define ID_MOTOR3_SPEED_LOCATION_MODE   0x34 

#define ID_MOTOR4_CHOOSE_MODE           0x40
#define ID_MOTOR4_PWM_MODE              0x41
#define ID_MOTOR4_SPEED_MODE            0x42
#define ID_MOTOR4_PWM_LOCATION_MODE     0x43
#define ID_MOTOR4_SPEED_LOCATION_MODE   0x44 

#define ID_MOTOR5_CHOOSE_MODE           0x50
#define ID_MOTOR5_PWM_MODE              0x51
#define ID_MOTOR5_SPEED_MODE            0x52
#define ID_MOTOR5_PWM_LOCATION_MODE     0x53
#define ID_MOTOR5_SPEED_LOCATION_MODE   0x54 

#define ID_MOTOR6_CHOOSE_MODE           0x60
#define ID_MOTOR6_PWM_MODE              0x61
#define ID_MOTOR6_SPEED_MODE            0x62
#define ID_MOTOR6_PWM_LOCATION_MODE     0x63
#define ID_MOTOR6_SPEED_LOCATION_MODE   0x64 

#define ID_MOTOR7_CHOOSE_MODE           0x70
#define ID_MOTOR7_PWM_MODE              0x71
#define ID_MOTOR7_SPEED_MODE            0x72
#define ID_MOTOR7_PWM_LOCATION_MODE     0x73
#define ID_MOTOR7_SPEED_LOCATION_MODE   0x74 

#define ID_MOTOR8_CHOOSE_MODE           0x80
#define ID_MOTOR8_PWM_MODE              0x81
#define ID_MOTOR8_SPEED_MODE            0x82
#define ID_MOTOR8_PWM_LOCATION_MODE     0x83
#define ID_MOTOR8_SPEED_LOCATION_MODE   0x84 


extern volatile unsigned char OverCurr_flag;

void CurrentProtect(void);
void CAN1_Configuration(void);

void Motor_Reset(int Motor_ID);
void Motor_Init(int Motor_ID,int Motor_Mode);

void Motor_PWM_Set(int Motor_ID,int Give_PWM);
void Motor_Speed_Set(int Motor_ID,int Give_Speed);
void Motor_PWM_Location_Set(int Motor_ID,int Give_PWM,int Give_PWM_Location);
void Motor_Speed_Location_Set(int Motor_ID,int Give_Speed,int Give_Speed_Location);

#endif 
