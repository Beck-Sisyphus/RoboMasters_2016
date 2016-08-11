#ifndef __APP_NEW_H_
#define __APP_NEW_H_
#include <stm32f4xx.h>
#include "main.h"

#define ESC_MAX 4000.0f
#define GAP 0.1f
#define abs(x) ((x)>0? (x):(-(x)))

#define ENCODER_MAX 8172
#define KAL_CONST_Z (100)
#define STATE_SWITCH_DELAY_TICK 10000

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
typedef struct motor_mapping_t motor_mapping_t;

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

//initiate status:
typedef enum
{
    PREPARE_STATE,     	// Initialization state for 4 seconds
    STANDBY_STATE,			// gimbal doesn't move
    NORMAL_STATE,
    STOP_STATE,        	// stop movement
}WorkState_e;

#define PREPARE_TIME_TICK_MS 3000      //prapare time in ms

#define PITCH_POSITION_PID_DEFAULT {\
  1,\
  1,\
  0,\
  0,\
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
  0.120f,\
  0.0f,\
  0.0f,\
}\

#define CHASSIS_MOTOR_SPEED_PID_DEFAULT {\
  1,\
  0.6,\
  0.0,\
  0.0,\
}\

static void SetWorkState(WorkState_e state);
WorkState_e GetWorkState(void);

int16_t set_chassis_motor_velocity(int can_address, int remote_receiver_velocity);
void set_Pitch_Yaw_Position(int16_t real_angle_pitch, int16_t real_angle_yaw);
void CMControlLoop(void);
void Control_Task(void);
void WorkStateFSM(void);
static void WorkStateSwitchProcess(void);
void ControlLoopTaskInit(void);
float calculateYawAngle(void);
static int16_t GimbalYawControlModeSwitch(float real_angle_yaw, PID_Regulator_t * yawPositionPID);

static int16_t max_min_angle(int16_t user_input, volatile motor_mapping_t * motor);
static int16_t min(int16_t, int16_t);
static int16_t max(int16_t, int16_t);
static int16_t PID_Control(float measured, float target, PID_Regulator_t * pid);
static int map(int x, int in_min, int in_max, int out_min, int out_max);
#endif
