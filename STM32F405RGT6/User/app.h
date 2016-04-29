#ifndef __APP_H__
#define __APP_H__
#include <stm32f4xx.h>

#define ESC_MAX 50000.0f
#define GAP 1.0f
#define abs(x) ((x)>0? (x):(-(x)))

/*  System Coordinate defination
    Roll-Pitch-Yaw Angles
    x-axis forward, y-axis right, z-axis down
*/
// Measured reading in radian unit, * 1000 for communication with ROS
#define REAL_PITCH_LOW 977   // 56 degree, real lowest angle value that the cannon can reach
#define REAL_PITCH_HIGH 1990 // 114 degree, real lowest angle value that the cannon can reach
#define REAL_YAW_LOW -1798   // -103 degree
#define REAL_YAW_HIGH 1798   // 103 degree
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

void set_Pitch_Position(uint16_t);
void set_Yaw_Position(uint16_t);
void set_Pitch_Yaw_Position(uint16_t, uint16_t);

float Velocity_Control_205(float, float);
float Position_Control_205(float, float);

float Velocity_Control_206(float, float);
float Position_Control_206(float, float);

static long map(long x, long in_min, long in_max, long out_min, long out_max);

#endif
