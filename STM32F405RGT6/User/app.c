#include "main.h"

extern uint16_t measured_yaw_angle;
extern uint16_t measured_pitch_angle;
extern MPU6050_REAL_DATA MPU6050_Real_Data;

/* Pitch Constants  */
// from Xian jiangtong University
const float v_p_205 = 7.0;
const float v_i_205 = 0;
const float v_d_205 = 0;
// Constants from Northeast Forestry University
// const float v_p_205 = 25.0;
// const float v_i_205 = 0.0;
// const float v_d_205 = 12.0;

// Constants from Xian jiangtong University
const float l_p_205 = 100;
const float l_i_205 = 0.1;
const float l_d_205 = 10;
// Constants from Northeast Forestry University
// const float l_p_205 = 30;
// const float l_i_205 = 0.01;
//const float l_d_205 = 30.0;

/* Yaw Constants*/
const float v_p_206 = 100.0;
const float v_i_206 = 0.0001;
const float v_d_206 = 0.0;

const float l_p_206 = 10.0;//3#5#:0.760
const float l_i_206 = 1.0;
const float l_d_206 = 1.0;//3.5;

/*
@@ Description: Top level Function to implement PID control on Pitch Servo
 @ Input:       Real angle from z axis to the position, medium 90
 @ Output:      send command to pitch servo to execute
*/
void set_Pitch_Position(uint16_t real_angle_pitch)
{
    // PID for position
    float target_pitch_angle = map(real_angle_pitch, REAL_PITCH_LOW, REAL_PITCH_HIGH, BLUE_PITCH_LOW, BLUE_PITCH_HIGH);
    float pitch_position_change = Position_Control_205((float)measured_pitch_angle, (float)target_pitch_angle);
    float pitch_velocity_change = Velocity_Control_205((float)MPU6050_Real_Data.Gyro_Y, pitch_position_change);

    pitchyaw_control(0, (int16_t) pitch_position_change);
    // pitchyaw_control(0, (int16_t)pitch_velocity_change);

    // Motor_Current_Send(2, (int16_t)pitch_position_change);
    //Motor_Current_Send(2, (int16_t)pitch_velocity_change);
}

/*
@@ Description: Top level Function to implement PID control on Yaw Servo
 @ Input:       Real angle from x axis to the position, medium 0
 @ Output:      send command to pitch servo to execute
*/
void set_Yaw_Position(uint16_t real_angle_yaw)
{
    // PID for position
    float target_yaw_angle = map(real_angle_yaw, REAL_YAW_LOW, REAL_YAW_HIGH, BLUE_YAW_RIGHT, BLUE_YAW_LEFT);
    float yaw_position_change = Position_Control_206((float)measured_yaw_angle, (float)target_yaw_angle);
    float yaw_velocity_change = Velocity_Control_206((float)MPU6050_Real_Data.Gyro_Z, yaw_position_change);

    pitchyaw_control((int16_t) yaw_position_change, 0);
    //pitchyaw_control((int16_t) yaw_velocity_change, 0);

    // Motor_Current_Send(1, (int16_t) yaw_position_change);
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

    output = error_v[1] * v_p_205
            + inte * v_i_205
             + (error_v[1] - error_v[0]) * v_d_205;

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
    static float error_l[2] = {0.0,0.0};
    static float output = 0;
    static float inte = 0;

    error_l[0] = error_l[1];
    error_l[1] = target_position_205 - current_position_205;
    inte += error_l[1];

    output = error_l[1] * l_p_205
            + inte * l_i_205
            + (error_l[1] - error_l[0]) * l_d_205;

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
@@ Description: Close loop control for the yaw axis speed on motor controller
 @ Input      : Current speed from yaw axis
 @ Output     : Target speed for yaw axis
*********************************************************************************/
float Velocity_Control_206(float current_velocity_206,float target_velocity_206)
{
    static float error_v[2] = {0.0,0.0};
    static float output = 0;
    static float inte = 0;

    if(abs(current_velocity_206) < GAP)
    {
        current_velocity_206 = 0.0;
    }

    error_v[0] = error_v[1];
    error_v[1] = target_velocity_206 - current_velocity_206;
    inte += error_v[1];

    output = error_v[1] * v_p_206
             + inte * v_i_206
             + (error_v[1] - error_v[0]) * v_d_206;

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
    static float error_l[2] = {0.0,0.0};
    static float output = 0;
    static float inte = 0;

    error_l[0] = error_l[1];
    error_l[1] = target_position_206 - current_position_206;
    inte += error_l[1];

    output = error_l[1] * l_p_206
							+ inte * l_i_206
							+ (error_l[1] - error_l[0]) * l_d_206;

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

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
