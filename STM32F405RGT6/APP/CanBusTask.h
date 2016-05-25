#ifndef _CAN_BUS_TASK_H_
#define _CAN_BUS_TASK_H_

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
