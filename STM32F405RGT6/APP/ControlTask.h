#ifndef _CONTROL_TASK_H_
#define _CONTROL_TASK_H_
#include "main.h"

//initialize status:
typedef enum
{
    PREPARE_STATE,      // The initialization state when power turned on
    STANDBY_STATE,      // The state when gimbal stop moving
    MANUAL_STATE,       // For manual control the robot
    SEMI_AUTONOMOUS_STATE,  // Allow the computer vision to control the gimbal
    FULL_AUTONOMOUS_STATE,  // Let the SLAM to control the chassis for navigate
    STOP_STATE,        	// The state without movement
}WorkState_e;

#define YAW_POSITION_KP_DEFAULTS  10.0
#define YAW_POSITION_KI_DEFAULTS  0
#define YAW_POSITION_KD_DEFAULTS  0

#define YAW_SPEED_KP_DEFAULTS  1.0
#define YAW_SPEED_KI_DEFAULTS  0
#define YAW_SPEED_KD_DEFAULTS  0

// avoid bang --->  position:20.0  speed:19.0
//big bang   22.5 20.0
#define PITCH_POSITION_KP_DEFAULTS  10.0
#define PITCH_POSITION_KI_DEFAULTS  0
#define PITCH_POSITION_KD_DEFAULTS  0

#define PITCH_SPEED_KP_DEFAULTS  1.0
#define PITCH_SPEED_KI_DEFAULTS  0
#define PITCH_SPEED_KD_DEFAULTS  0

// Measured reading
#define REAL_PITCH_LOW 43  // real lowest angle value that the cannon can reach, measured by protractor
#define REAL_PITCH_HIGH 114 // real lowest angle value that the cannon can reach, measured by protractor
#define REAL_YAW_LOW -103
#define REAL_YAW_HIGH 103

/*
#define REAL_PITCH_LOW 750  // real lowest angle value that the cannon can reach, measured by protractor
#define REAL_PITCH_HIGH 1990 // real lowest angle value that the cannon can reach, measured by protractor
#define REAL_YAW_LOW -1798
#define REAL_YAW_HIGH 1798
*/
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

#define GIMBAL_MOTOR_PITCH_POSITION_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	PITCH_POSITION_KP_DEFAULTS,\
	PITCH_POSITION_KI_DEFAULTS,\
	PITCH_POSITION_KD_DEFAULTS,\
	0,\
	0,\
	0,\
	4900,\
	1000,\
	1500,\
	0,\
	4900,\
	0,\
	0,\
	0,\
	&PID_Calc,\
	&PID_Reset,\
}\

#define GIMBAL_MOTOR_PITCH_SPEED_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	PITCH_SPEED_KP_DEFAULTS,\
	PITCH_SPEED_KI_DEFAULTS,\
	PITCH_SPEED_KD_DEFAULTS,\
	0,\
	0,\
	0,\
	4900,\
	1000,\
	1500,\
	0,\
	4900,\
	0,\
	0,\
	0,\
	&PID_Calc,\
	&PID_Reset,\
}\

#define GIMBAL_MOTOR_YAW_POSITION_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	YAW_POSITION_KP_DEFAULTS,\
	YAW_POSITION_KI_DEFAULTS,\
	YAW_POSITION_KD_DEFAULTS,\
	0,\
	0,\
	0,\
	4900,\
	1000,\
	1500,\
	0,\
	5000,\
	0,\
	0,\
	0,\
	&PID_Calc,\
	&PID_Reset,\
}\

#define GIMBAL_MOTOR_YAW_SPEED_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	YAW_SPEED_KP_DEFAULTS,\
	YAW_SPEED_KI_DEFAULTS,\
	YAW_SPEED_KD_DEFAULTS,\
	0,\
	0,\
	0,\
	4900,\
	1000,\
	1500,\
	0,\
	4900,\
	0,\
	0,\
	0,\
	&PID_Calc,\
	&PID_Reset,\
}\

extern PID_Regulator_t GMPPositionPID;      //pitch axis position PID loop
extern PID_Regulator_t GMPSpeedPID;      		//pitch axis velocity PID loop
extern PID_Regulator_t GMYPositionPID;			//yaw axis position PID loop
extern PID_Regulator_t GMYSpeedPID;      		//yaw axis velocity PID loop

WorkState_e GetWorkState(void);
void ControtLoopTaskInit(void);
void Control_Task(void);
void WorkStateFSM(void);
static void WorkStateSwitchProcess(void);
void GMPitchControlLoop(void);
void GMYawControlLoop(void);
void SetGimbalMotorOutput(void);
void ControtLoopTaskInit(void);

//模式切换处理，得到位置环的设定值和给定值
// Mode switch process, to get the position target value and measured value
void GimbalYawControlModeSwitch(void);
void SetGimbalMotorOutput(void);

#endif
