#ifndef _CAN_BUS_TASK_H_
#define _CAN_BUS_TASK_H_
#include "main.h"

/* CAN Bus 1 */
#define CAN_BUS1_ZGYRO_FEEDBACK_MSG_ID            0x401

/* CAN Bus 2 */
#define CAN_BUS2_MOTOR1_FEEDBACK_MSG_ID           0x201
#define CAN_BUS2_MOTOR2_FEEDBACK_MSG_ID           0x202
#define CAN_BUS2_MOTOR3_FEEDBACK_MSG_ID           0x203
#define CAN_BUS2_MOTOR4_FEEDBACK_MSG_ID           0x204
#define CAN_BUS2_MOTOR5_FEEDBACK_MSG_ID           0x205
#define CAN_BUS2_MOTOR6_FEEDBACK_MSG_ID           0x206

extern volatile Encoder CM1Encoder;
extern volatile Encoder CM2Encoder;
extern volatile Encoder CM3Encoder;
extern volatile Encoder CM4Encoder;
extern volatile Encoder GMYawEncoder;
extern volatile Encoder GMPitchEncoder;
extern float ZGyroModuleAngle;

#define pitch_max 15.0
#define yaw_max 720.0
#define CHASSIS_MOTOR_STRENGTH 11

// For Red motor
// #define MOTOR_YAW 				1
// #define MOTOR_PITCH 			2
// #define MOTOR_FRONT_LEFT 		3
// #define MOTOR_BACK_LEFT 		4
// #define MOTOR_FRONT_RIGHT 		5
// #define MOTOR_BACK_RIGHT 		6

// For Blue motor and all 2016 soldier robots
#define MOTOR_YAW 				1
#define MOTOR_PITCH 			2
#define MOTOR_FRONT_RIGHT 		3
#define MOTOR_FRONT_LEFT 		4
#define MOTOR_BACK_LEFT 		5
#define MOTOR_BACK_RIGHT 		6

void GetEncoderBias(volatile Encoder *v, CanRxMsg * msg);
void EncoderProcess(volatile Encoder *v, CanRxMsg * msg);
void CanReceiveMsgProcess(CanRxMsg * msg);

void Encoder_sent(float encoder_angle);
void Motor_Reset_Can_2(void);
void PitchYaw_Address_Setup(void);
void Wheels_Address_Setup(void);
void Set_PitchYaw_Current(void);
void Set_Wheels_Current(void);
void Motor_ManSet_Can_2(void);
void Set_Gimbal_Current(int16_t, int16_t);
void wheel_control(int16_t, int16_t, int16_t);

#endif

// Comments from Tu Vu
/*******
RX addresses
0x201: Front right wheel
0x202: Front left wheel
0x203: Back left wheel
0x204: Back right wheel
0x205: Yaw
0x206: Pitch

All rx messages mapped the same way:
data 0 and 1 measure angle
data 4 and 5 relates to what current you are tx to motors
data 4 and 5 NOT same as current tx value
*******/
/***************
Wheel RX Address
0x201: Front right wheel
0x202: Front left wheel
0x203: Back left wheel
0x204: Back right wheel

For sample robot 2015, with EC60 motor drives:
    data 0 and 1 measure position of wheel, from 0 to 8191
    clockwise wheel rotation decreases wheel's position
    wheel position value repeats (decrease from 0 means go back to 8200 again)

    data 4 and 5 relates to what current you are tx to motors
    data 4 and 5 NOT same as current tx value

    data 2, 3, 6, 7 not useful

For robot we build in 2016, with 820R motor drives:
    You calibrate the CAN address in the first time
    data 0 and 1 measure position of wheel, from 0 to 8191
    data 2 and 3 measure rotational speed, in unit of RPM
    data 4,5, 6, and 7 are null.
***************/

/************** End of Wheel Motor RX Code and Address **************/
    /************ YAW ************/
    // Yaw angle range is: [around 40, around 4800]
    // 40 is right-most yaw position
    // around 4800 is left-most yaw position
    // data 0 and 1 measure angle

    // data 4 and 5 relates to what current you are tx to motors
    // data 4 and 5 NOT same as current tx value
    // -1000 current value = 27852 (yaw_data4)<<8|(yaw_data5) value
    // -750 current value = 24305 (yaw_data4)<<8|(yaw_data5) value
    // -500 current value = 16630 (yaw_data4)<<8|(yaw_data5) value
    // pitch has same current to (data4<<8)|(data5) conversion

    /************ PITCH ************/
    // pitch angle range is [around 3520, around 4500]
    // around 3520 is highest pitch position
    // around 4500 is lowest pitch position
    // data 0 and 1 measure angle

    // data 4 and 5 relates to what current you are tx to motors
    // data 4 and 5 NOT same as current tx value
    // -1000 current value = 27852 (pitch_data4)<<8|(pitch_data5) value
    // -750 current value = 24305 (pitch_data4)<<8|(pitch_data5) value
    // -500 current value = 16630 (pitch_data4)<<8|(pitch_data5) value
    // yaw has same current to (data4<<8)|(data5) conversion
