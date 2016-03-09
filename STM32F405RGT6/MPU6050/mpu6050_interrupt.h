#ifndef __MPU6050_INTERRUPT_H__
#define __MPU6050_INTERRUPT_H__
#include <stm32f4xx.h>

void MPU6050_Interrupt_Configuration(void);
extern float target_pitch_angle;
extern float target_yaw_angle;
extern float this_203_angle;
extern float velocity_203_output;
extern float position_203_output;
#endif
