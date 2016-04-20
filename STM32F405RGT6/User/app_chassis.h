#ifndef __APP_CHASSIS_H__
#define __APP_CHASSIS_H__
#include <stm32f4xx.h>

#define abs(x) ((x)>0? (x):(-(x)))
#define GAP 0.0f
#define ESC_MAX 50000.0f

float Velocity_Control_201(float target_velocity_201);
float Velocity_Control_202(float target_velocity_202);
float Velocity_Control_203(float target_velocity_203);
float Velocity_Control_204(float target_velocity_204);
float PID_Control(float measured,float target, const float p, const float i, const float d);

#endif
