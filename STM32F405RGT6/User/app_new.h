#ifndef __APP_NEW_H_
#define __APP_NEW_H_
#include <stm32f4xx.h>
#include "main.h"

#define ESC_MAX 50000.0f
#define GAP 0.1f
#define abs(x) ((x)>0? (x):(-(x)))

#define ENCODER_MAX 8172

// pitch-yaw-roll coordinate system in radian for turret on sample robot
#define REAL_PITCH_LOW 750  // real lowest angle value that the cannon can reach, measured by protractor
#define REAL_PITCH_HIGH 1990 // real lowest angle value that the cannon can reach, measured by protractor
#define REAL_YAW_LOW -1798
#define REAL_YAW_HIGH 1798

#define BLUE_PITCH_LOW 7530
#define BLUE_PITCH_HIGH 6244
#define BLUE_YAW_RIGHT 100
#define BLUE_YAW_LEFT 4871

#define RED_PITCH_LOW  5238
#define RED_PITCH_HIGH 4039
#define RED_YAW_RIGHT  0
#define RED_YAW_LEFT   4640

#define REAL_CANNON_PITCH_HIGH 2269 // 130 degree
#define REAL_CANNON_PITCH_LOW 907 // 52 degree
#define REAL_CANNON_YAW_LEFT -1798 // 130 degree
#define REAL_CANNON_YAW_RIGHT 1798 // 52 degree

#define CANNON_PITCH_HIGH 1175
#define CANNON_PITCH_LOW  7480
#define CANNON_YAW_LEFT 5171
#define CANNON_YAW_RIGHT 470

static int16_t PID_Control(float measured, float target, int sign, const float p, const float i, const float d);
static int map(int x, int in_min, int in_max, int out_min, int out_max);
#endif
