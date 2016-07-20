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
#define BLUE_YAW_LEFT 4871
#define BLUE_YAW_RIGHT 100

#define RED_PITCH_LOW  5238
#define RED_PITCH_HIGH 4039
#define RED_YAW_LEFT   4640
#define RED_YAW_RIGHT  0

#define REAL_CANNON_PITCH_HIGH 2269 // 130 degree
#define REAL_CANNON_PITCH_LOW 907 // 52 degree
#define REAL_CANNON_YAW_LEFT -1798 // 130 degree
#define REAL_CANNON_YAW_RIGHT 1798 // 52 degree

#define CANNON_PITCH_HIGH 1175
#define CANNON_PITCH_LOW  7480
#define CANNON_YAW_LEFT 5171
#define CANNON_YAW_RIGHT 470

typedef struct PID_Regulator_t
{
    int sign;
    float kp;
    float ki;
    float kd;
    // float ref;
    // float fdb;
    // float err[2];
    // float output;
}PID_Regulator_t;

#define PITCH_POSITION_PID_DEFAULT {\
  1,\
  1,\
  0,\
  0.001,\
}\

#define PITCH_SPEED_PID_DEFAULT {\
  -1,\
  15.0,\
  0.0,\
  0.0,\
}\

#define YAW_POSITION_PID_DEFAULT {\
  1,\
  1.0,\
  0.0,\
  0.0,\
}\

#define YAW_SPEED_PID_DEFAULT {\
  1,\
  10.0,\
  0.0,\
  0.0,\
}\

#define CHASSIS_MOTOR_ROTATE_PID_DEFAULT {\
  -1,\
  1.20f,\
  0.0f,\
  0.0f,\
}\

#define CHASSIS_MOTOR_SPEED_PID_DEFAULT {\
  1,\
  5.0,\
  0.0,\
  0.0,\
}\



int16_t set_chassis_motor_velocity(int can_address, int remote_receiver_velocity);
void set_Pitch_Yaw_Position(int16_t real_angle_pitch, int16_t real_angle_yaw);
void CMControlLoop(void);

static int16_t PID_Control_test(float measured, float target, PID_Regulator_t * pid);
static int16_t PID_Control(float measured, float target, int sign, const float p, const float i, const float d);
static int map(int x, int in_min, int in_max, int out_min, int out_max);
#endif
