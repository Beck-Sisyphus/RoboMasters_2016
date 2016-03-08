#include "main.h"

///Turns on Beck's trying for PID controller 
#define PID true


extern uint16_t temp_yaw_angle;
extern uint16_t temp_pitch_angle;

extern uint16_t temp_yaw_current;
extern uint16_t temp_pitch_current;

#if PID
    extern uint16_t measure_yaw_current;
    extern uint16_t measure_pitch_current;
#endif

/*
    Top level Function to implement PID control on Pitch Servo
*/
void pitch_Position(float target_pitch_angle)
{
    // PID for position
    float pitch_position_change = Position_Control_205(temp_pitch_angle, target_pitch_angle);
    float pitch_velocity_change = Velocity_Control_205(temp_pitch_current, pitch_position_change);
    Motor_Current_Send(2, temp_pitch_current + pitch_velocity_change);
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
float Velocity_Control_205(float current_velocity_201,float target_velocity_201)
{
    const float v_p = 15.0;
    const float v_d = 1.0;
    
    static float error_v[2] = {0.0,0.0};
    static float output = 0;
    
    if(abs(current_velocity_201) < GAP)
    {
        current_velocity_201 = 0.0;
    }
    
    error_v[0] = error_v[1];
    error_v[1] = target_velocity_201 - current_velocity_201;
    
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
    
    return -output;//cyq:for 6015 motor, negative feedback
}


/********************************************************************************
@@ Description: Close loop control for the pitch axis position on motor controller
 @ Input      : Current position from pitch axis
 @ Output     : Target position for pitch axis
*********************************************************************************/
float Position_Control_205(float current_position_201,float target_position_201)
{
    
    const float l_p = 16.0;
    const float l_i = 0.0;
    const float l_d = 0.6;

    static float error_l[2] = {0.0,0.0};
    static float output = 0;
    static float inte = 0;
    
    error_l[0] = error_l[1];
    error_l[1] = target_position_201 - current_position_201;
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
float Velocity_Control_206(float current_velocity_203,float target_velocity_203)
{
    const float v_p = 50.0;
    const float v_d = 0.0;
    
    static float error_v[2] = {0.0,0.0};
    static float output = 0;
		
    if(abs(current_velocity_203) < GAP)
    {
        current_velocity_203 = 0.0;
    }
    
    error_v[0] = error_v[1];
    error_v[1] = target_velocity_203 - current_velocity_203;
    
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
    
    return -output;//cyq:for6015 ·´Ïò
}

/********************************************************************************
@@ Description: Close loop control for the yaw axis position on motor controller
 @ Input      : Current position from yaw axis
 @ Output     : Target position for yaw axis
*********************************************************************************/
float Position_Control_206(float current_position_203,float target_position_203)
{
    const float l_p = 30.010;//3#5#:0.760
	const float l_i = 0.0;//0.000035;
    const float l_d = 0.5;//3.5;
    
    static float error_l[3] = {0.0,0.0,0.0};
    static float output = 0;
    
    error_l[0] = error_l[1];
    error_l[1] = error_l[2];    
    error_l[2] = target_position_203 - current_position_203;
 
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
    
    return -output;
}
