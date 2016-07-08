#ifndef __APP_CHASSIS_H__
#define __APP_CHASSIS_H__
#include <stm32f4xx.h>

#define abs(x) ((x)>0? (x):(-(x)))
#define ESC_MAX 50000.0f

int16_t Velocity_Control_201(float);
int16_t Velocity_Control_202(float);
int16_t Velocity_Control_203(float);
int16_t Velocity_Control_204(float);
int16_t PID_Control(float measured,float target, const float p, const float i, const float d);

#endif
