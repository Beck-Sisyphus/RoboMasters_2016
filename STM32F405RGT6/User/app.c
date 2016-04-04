#include "main.h"

extern uint16_t measured_yaw_angle;
extern uint16_t measured_pitch_angle;

extern int16_t measured_yaw_current;
extern int16_t measured_pitch_current;

extern int16_t target_yaw_current;
extern int16_t target_pitch_current;

extern MPU6050_REAL_DATA MPU6050_Real_Data;


/*
    Top level Function to implement PID control on Pitch Servo
*/
// Pitch reading on blue robot is normalized to 4789 ~ 3544, lowest to highest
// Current range from 1500 - 2300 will make it rise
void set_Pitch_Position(uint16_t target_pitch_angle)
{
    // PID for position
    float pitch_position_change = Position_Control_205((float)measured_pitch_angle, (float)target_pitch_angle);
    float pitch_velocity_change = Velocity_Control_205((float)MPU6050_Real_Data.Gyro_X, pitch_position_change);
    Motor_Current_Send(2, (int16_t)pitch_position_change);
    // Motor_Current_Send(2, (int16_t)pitch_velocity_change);
}

// Yaw reading on blue robot is normalized to 4803 ~ 37, left - right
// current range from -500 ~ 500 will good for tuning
void set_Yaw_Position(uint16_t target_yaw_angle)
{
    // PID for position
    float yaw_position_change = Position_Control_206((float)measured_yaw_angle, (float)target_yaw_angle);
    float yaw_velocity_change = Velocity_Control_206((float)MPU6050_Real_Data.Gyro_Y, yaw_position_change);
    Motor_Current_Send(1, yaw_position_change);
    // Motor_Current_Send(1, (int16_t)yaw_velocity_change);
}


/********************************************************************************
    Send signals to other motor controller board
    ID for this board is 0x200
    If only use two motor controller board,
        the data return ID are 0x201 and 0x202
    Currently sending to three electronic speed control board
*********************************************************************************/
void Cmd_ESC(int16_t current_201,int16_t current_202,int16_t current_203)
{
    CanTxMsg tx_message;

    tx_message.StdId = 0x200;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;

    tx_message.Data[0] = (unsigned char)(current_201 >> 8);
    tx_message.Data[1] = (unsigned char)current_201;
    tx_message.Data[2] = (unsigned char)(current_202 >> 8);
    tx_message.Data[3] = (unsigned char)current_202;
    tx_message.Data[4] = (unsigned char)(current_203 >> 8);
    tx_message.Data[5] = (unsigned char)current_203;
    tx_message.Data[6] = 0x00;
    tx_message.Data[7] = 0x00;

    CAN_Transmit(CAN1,&tx_message);
}

/********************************************************************************
@@ Description: Close loop control for the pitch axis speed on motor controller
 @ Input      : Current speed from pitch axis
 @ Output     : Target speed for pitch axis
*********************************************************************************/
float Velocity_Control_205(float current_velocity_205,float target_velocity_205)
{
    // Constants from Xian jiangtong University
    // const float v_p = 15.0;
    // const float v_i = 0.03;
    // const float v_d = 1.0;
    // Constants from Northeast Forestry University
    const float v_p = 25.0;
    const float v_i = 0.0;
    const float v_d = 12.0;
    static float error_v[2] = {0.0,0.0};
    static float output = 0;
    static float inte = 0;

    if(abs(current_velocity_205) < GAP)
    {
        current_velocity_205 = 0.0;
    }

    error_v[0] = error_v[1];
    error_v[1] = target_velocity_205 - current_velocity_205;
    inte += error_v[1];

    output = error_v[1] * v_p
            + inte * v_i
             + (error_v[1] - error_v[0]) * v_d;

    if(output > ESC_MAX)
    {
        output = ESC_MAX;
    }

    if(output < -ESC_MAX)
    {
        output = -ESC_MAX;
    }

    return output; // For Blue rover, position reading is in inverse direction
}


/********************************************************************************
@@ Description: Close loop control for the pitch axis position on motor controller
 @ Input      : Current position from pitch axis
 @ Output     : Target position for pitch axis
*********************************************************************************/
float Position_Control_205(float current_position_205,float target_position_205)
{
    // Constants from Xian jiangtong University
    const float l_p = 16;
    const float l_i = 0.0;
    const float l_d = 0.6;
    // Constants from Northeast Forestry University
    // const float l_p = 30;
    // const float l_i = 0.01;
    //const float l_d = 30.0;

    static float error_l[2] = {0.0,0.0};
    static float output = 0;
    static float inte = 0;

    error_l[0] = error_l[1];
    error_l[1] = target_position_205 - current_position_205;
    inte += error_l[1];

    output = error_l[1] * l_p
            + inte * l_i
            + (error_l[1] - error_l[0]) * l_d;

    if(output > ESC_MAX)
    {
        output = ESC_MAX;
    }

    if(output < -ESC_MAX)
    {
        output = -ESC_MAX;
    }

    return output;
}
/********************************************************************************
@@ Description: Close loop control for the yaw axis speed on motor controller
 @ Input      : Current speed from yaw axis
 @ Output     : Target speed for yaw axis
*********************************************************************************/
float Velocity_Control_206(float current_velocity_206,float target_velocity_206)
{
    const float v_p = 50.0;
    const float v_d = 0.0;

    static float error_v[2] = {0.0,0.0};
    static float output = 0;

    if(abs(current_velocity_206) < GAP)
    {
        current_velocity_206 = 0.0;
    }

    error_v[0] = error_v[1];
    error_v[1] = target_velocity_206 - current_velocity_206;

    output = error_v[1] * v_p
             + (error_v[1] - error_v[0]) * v_d;

    if(output > ESC_MAX)
    {
        output = ESC_MAX;
    }

    if(output < -ESC_MAX)
    {
        output = -ESC_MAX;
    }

    return -output;
}

/********************************************************************************
@@ Description: Close loop control for the yaw axis position on motor controller
 @ Input      : Current position from yaw axis
 @ Output     : Target position for yaw axis
*********************************************************************************/
float Position_Control_206(float current_position_206,float target_position_206)
{
    const float l_p = 30.0;//3#5#:0.760
    const float l_i = 0.001;
    const float l_d = 0.5;//3.5;

    static float error_l[3] = {0.0,0.0,0.0};
    static float output = 0;

    error_l[0] = error_l[1];
    error_l[1] = error_l[2];
    error_l[2] = target_position_206 - current_position_206;

    output = error_l[2] * l_p
							+ error_l[2] * l_i
							+ (error_l[2] - error_l[1]) * l_d;

    if(output > ESC_MAX)
    {
        output = ESC_MAX;
    }

    if(output < -ESC_MAX)
    {
        output = -ESC_MAX;
    }

    return output;
}
