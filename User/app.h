#ifndef __APP_H__
#define __APP_H__
#include <stm32f4xx.h>

#define ESC_MAX 5000.0
#define abs(x) ((x)>0? (x):(-(x)))

void Cmd_ESC(int16_t current_201,int16_t current_202,int16_t current_203);

float Velocity_Control_201(float current_velocity_201,float target_velocity_201);
float Position_Control_201(float current_position_201,float target_position_201);
float Velocity_Control_201_Shoot(float current_velocity_201,float target_velocity_201);

float Velocity_Control_203(float current_velocity_203,float target_velocity_203);
float Position_Control_203(float current_position_203,float target_position_203);

#endif
