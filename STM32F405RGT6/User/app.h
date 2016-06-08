#ifndef __APP_H__
#define __APP_H__
#include <stm32f4xx.h>

#define ESC_MAX 50000.0f
#define GAP 0.1f
#define abs(x) ((x)>0? (x):(-(x)))

#define ENCODER_MAX 8172

// pitch-yaw-roll coordinate system in degree
// #define REAL_PITCH_LOW 43  // real lowest angle value that the cannon can reach, measured by protractor
// #define REAL_PITCH_HIGH 114 // real lowest angle value that the cannon can reach, measured by protractor
// #define REAL_YAW_LOW -103
// #define REAL_YAW_HIGH 103

// pitch-yaw-roll coordinate system in radian for turret on sample robot
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
#define RED_PITCH_LOW  5238 // 3092
//#define RED_PITCH_MID 4500
#define RED_PITCH_HIGH 4039 // 2584
#define RED_YAW_RIGHT  0    // 2284
//#define RED_YAW_MID 2320
#define RED_YAW_LEFT   4640 // 6888


#define REAL_CANNON_PITCH_HIGH 2269 // 130 degree
#define REAL_CANNON_PITCH_LOW 907 // 52 degree
#define REAL_CANNON_YAW_LEFT -1798 // 130 degree
#define REAL_CANNON_YAW_RIGHT 1798 // 52 degree

#define CANNON_PITCH_HIGH 1175
#define CANNON_PITCH_LOW  7480
#define CANNON_YAW_LEFT 5171
#define CANNON_YAW_RIGHT 470

// 1363 7426 3016 6469


// Encoder reading
#define YELLOW_PITCH_LOW 7530 // 4789
#define YELLOW_PITCH_HIGH 6244 // 3544
#define YELLOW_YAW_RIGHT 4036  // 507740
#define YELLOW_YAW_LEFT 4871  //

void set_Pitch_Position(int16_t);
void set_Yaw_Position(int16_t);
void set_Pitch_Yaw_Position(int16_t, int16_t);

float Velocity_Control_205(float, float);
static float Position_Control_205(float, float);

float Velocity_Control_206(float, float);
static float Position_Control_206(float, float);

static int map(int x, int in_min, int in_max, int out_min, int out_max);

#endif
