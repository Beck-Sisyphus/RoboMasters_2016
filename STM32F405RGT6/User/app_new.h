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

#define PITCH_SPEED_PID_DEFAULT {\
  -1,\
  15.0,\
  0.0,\
  0.01,\
}\

#define PITCH_POSITION_PID_DEFAULT {\
  -1,\
  0.75,\
  0,\
  0,\
}\

#define YAW_SPEED_PID_DEFAULT {\
  -1,\
  10.0,\
  0.0,\
  0.0,\
}\

#define YAW_POSITION_PID_DEFAULT {\
  -1,\
  1.0,\
  0.0,\
  0.0,\
}\

#define CHASSIS_MOTOR_ROTATE_PID_DEFAULT {\
  -1,\
  0.0,\
  0.0,\
  0.0,\
}\

#define CHASSIS_MOTOR_SPEED_PID_DEFAULT {\
  1,\
  5.0,\
  0.0,\
  0.0,\
}\

typedef struct motor_mapping_t
{
  int real_low;
  int real_high;
  int ecd_low;
  int ecd_high;
} motor_mapping_t;

typedef struct gimbal_mapping_t
{
  motor_mapping_t pitch;
  motor_mapping_t yaw;
}gimbal_mapping_t;

// measured by protractor, in radian * 1000, in aircraft coordinate
// CAN bus feedback for both RM6025 and RM6623
#define GIMBAL_BLUE_SAMPLE_ROBOT_0 {\
	{\
    750,\
    1990,\
    7530,\
    6244,\
  },\
  {\
    -1798,\
    1798,\
    4871,\
    100,\
  },\
}\

#define GIMBAL_RED_SAMPLE_ROBOT_1 {\
  {\
    750,\
    1990,\
    5238,\
    4039,\
  },\
  {\
    -1798,\
    1798,\
    4640,\
    0,\
  },\
}\
// 70 degree，127 degree
#define GIMBAL_SOLDIER_5 {\
  {\
    1222,\
    2217,\
    6210,\
    7600,\
  },\
  {\
    -1571,\
    1571,\
    6880,\
    2767,\
  },\
}\

// 52 degree，130 degree
#define GIMBAL_HERO_ROBOT_CANNON_7 {\
  {\
    907 ,\
    2269,\
    7480,\
    1175,\
  },\
  {\
    -1798,\
    1798,\
    5171,\
    470,\
  },\
}\

#define GIMBAL_DEFAULT {\
  {\
    0,\
    0,\
    0,\
    0,\
  },\
  {\
    0,\
    0,\
    0,\
    0,\
  },\
}\

int16_t set_chassis_motor_velocity(int can_address, int remote_receiver_velocity);
void set_Pitch_Yaw_Position(int16_t real_angle_pitch, int16_t real_angle_yaw);

static int16_t PID_Control_test(float measured, float target, PID_Regulator_t * pid);
static int16_t PID_Control(float measured, float target, int sign, const float p, const float i, const float d);
static int map_motor(int x, const motor_mapping_t * map);
static int map(int x, int in_min, int in_max, int out_min, int out_max);
#endif
