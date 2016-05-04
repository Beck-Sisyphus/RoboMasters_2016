#ifndef __APP_H__
#define __APP_H__
#include <stm32f4xx.h>

#define ESC_MAX 50000.0f
#define GAP 1.0f
#define abs(x) ((x)>0? (x):(-(x)))

// Measured reading
// #define REAL_PITCH_LOW 43  // real lowest angle value that the cannon can reach, measured by protractor
// #define REAL_PITCH_HIGH 114 // real lowest angle value that the cannon can reach, measured by protractor
// #define REAL_YAW_LOW -103
// #define REAL_YAW_HIGH 103

#define REAL_PITCH_LOW 750  // real lowest angle value that the cannon can reach, measured by protractor
#define REAL_PITCH_HIGH 1990 // real lowest angle value that the cannon can reach, measured by protractor
#define REAL_YAW_LOW -1798
#define REAL_YAW_HIGH 1798
// Encoder reading
#define BLUE_PITCH_LOW 7530 // 4789
#define BLUE_PITCH_HIGH 6244 // 3544
#define BLUE_YAW_RIGHT 100  // 507740
#define BLUE_YAW_LEFT 4871  //
// Encoder reading
#define RED_PITCH_LOW 3092
#define RED_PITCH_HIGH 2584
#define RED_YAW_RIGHT 2284
#define RED_YAW_LEFT 6888

void set_Pitch_Position(int16_t);
void set_Yaw_Position(int16_t);
void set_Pitch_Yaw_Position(int16_t, int16_t);

float Velocity_Control_205(float, float);
static float Position_Control_205(float, float);

float Velocity_Control_206(float, float);
static float Position_Control_206(float, float);

static long map(long x, long in_min, long in_max, long out_min, long out_max);

#endif
