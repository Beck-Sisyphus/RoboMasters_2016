#ifndef __APP_CHASSIS_H__
#define __APP_CHASSIS_H__
#include <stm32f4xx.h>

#define abs(x) ((x)>0? (x):(-(x)))
#define GAP 0.0f
#define ESC_MAX 50000.0f

void Velocity_Control_201(float target_velocity_201);
void Velocity_Control_202(float target_velocity_202);
void Velocity_Control_203(float target_velocity_203);
void Velocity_Control_204(float target_velocity_204);
int16_t PID_Control(float measured,float target, const float p, const float i, const float d);

#endif
