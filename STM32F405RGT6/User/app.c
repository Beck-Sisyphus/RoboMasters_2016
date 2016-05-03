#include "main.h"
extern RC_Ctl_t RC_Ctl;


extern int16_t measured_yaw_angle;
extern int16_t measured_pitch_angle;

volatile int16_t pitch_Position;
volatile int16_t yaw_Position;
volatile int16_t pitch_Velocity;
volatile int16_t yaw_Velocity;

// for velocity controlling pitch and yaw with remote
volatile int16_t remote_pitch_change;
volatile int16_t remote_yaw_change;

// extern MPU6050_REAL_DATA MPU6050_Real_Data;

/* Pitch Constants  */
// from Xian jiangtong University
// const float v_p_205 = 15.0;
// const float v_i_205 = 0.03;
// const float v_d_205 = 1.0;
// Constants from Northeast Forestry University
// const float v_p_205 = 25.0;
// const float v_i_205 = 0.0;
// const float v_d_205 = 12.0;
// Beck's testing on Constants
const float v_p_205 = 10.0;
const float v_i_205 = 0.0;
const float v_d_205 = 0.0;

// Constants from Xian jiangtong University
// const float l_p_205 = 16;
// const float l_i_205 = 0.0;
// const float l_d_205 = 0.6;
// Constants from Northeast Forestry University
// const float l_p_205 = 30;
// const float l_i_205 = 0.01;
// const float l_d_205 = 30.0;
// Beck's testing on constants
const float l_p_205 = 1.0;
const float l_i_205 = 0.0;
const float l_d_205 = 0.0;

/* Yaw Constants*/

// 100 v_p for remote control velocity control for yaw
const float v_p_206_remote = 100.0;
const float v_p_206 = 20.0;

const float v_i_206 = 0.0;
const float v_d_206 = 0.0;
const float l_p_206 = 0.6;//3#5#:0.760
const float l_i_206 = 0.0;
const float l_d_206 = 0.0;//3.5;

/*
@@ Description: Top level Function to implement PID control on Pitch Servo
 @ Input:       Real angle from z axis to the position, medium 90
 @ Output:      send command to pitch servo to execute
*/
void set_Pitch_Yaw_Position(int16_t real_angle_pitch, int16_t real_angle_yaw)
{

    float target_pitch_angle;
    float pitch_position_change;
    float pitch_velocity_change;

    float target_yaw_angle;
    float yaw_position_change;
    float yaw_velocity_change;

    /********** Blue Motor **********/
    // // PID for pitch
    // float target_pitch_angle = map(real_angle_pitch, REAL_PITCH_LOW, REAL_PITCH_HIGH, BLUE_PITCH_LOW, BLUE_PITCH_HIGH);
    // float pitch_position_change = Position_Control_205((float)measured_pitch_angle, (float)target_pitch_angle);
    // float pitch_velocity_change = Velocity_Control_205((float)MPU6050_Real_Data.Gyro_Y, pitch_position_change);

    // // PID for yaw
    // float target_yaw_angle = map(real_angle_yaw, REAL_YAW_LOW, REAL_YAW_HIGH, BLUE_YAW_RIGHT, BLUE_YAW_LEFT);
    // float yaw_position_change = Position_Control_206((float)measured_yaw_angle, (float)target_yaw_angle);
    // float yaw_velocity_change = Velocity_Control_206((float)MPU6050_Real_Data.Gyro_Z, yaw_position_change);

    // // pitchyaw_control((int16_t) yaw_position_change, (int16_t) pitch_position_change);
    // pitchyaw_control((int16_t) yaw_velocity_change, (int16_t)pitch_velocity_change);



    /********** Red Motor **********/
    if (RC_Ctl.rc.s1 == RC_SW_DOWN && RC_Ctl.rc.s2 == RC_SW_DOWN) {
        // Pitch, yaw velocity control with remote        
        // PID for pitch
        pitch_velocity_change = Velocity_Control_205((float)MPU6050_Real_Data.Gyro_Y, real_angle_pitch);


        // PID for yaw
        yaw_velocity_change = Velocity_Control_206((float)MPU6050_Real_Data.Gyro_Z, real_angle_yaw, v_p_206_remote); 
    } else {
        // Auto target control
        // PID for pitch
        target_pitch_angle = map(real_angle_pitch, REAL_PITCH_LOW, REAL_PITCH_HIGH, RED_PITCH_LOW, RED_PITCH_HIGH);
        pitch_position_change = Position_Control_205((float)measured_pitch_angle, (float)target_pitch_angle);
        pitch_velocity_change = Velocity_Control_205((float)MPU6050_Real_Data.Gyro_Y, pitch_position_change);

        // PID for yaw
        target_yaw_angle = map(real_angle_yaw, REAL_YAW_LOW, REAL_YAW_HIGH, RED_YAW_RIGHT, RED_YAW_LEFT);
        yaw_position_change = Position_Control_206((float)measured_yaw_angle, (float)target_yaw_angle);
        yaw_velocity_change = Velocity_Control_206((float)MPU6050_Real_Data.Gyro_Z, yaw_position_change, v_p_206);
    }

    //     // Auto target control
    //     // PID for pitch
    // float target_pitch_angle = map(real_angle_pitch, REAL_PITCH_LOW, REAL_PITCH_HIGH, RED_PITCH_LOW, RED_PITCH_HIGH);
    // float pitch_position_change = Position_Control_205((float)measured_pitch_angle, (float)target_pitch_angle);
    // float pitch_velocity_change = Velocity_Control_205((float)MPU6050_Real_Data.Gyro_Y, pitch_position_change);

    // // PID for yaw
    // float target_yaw_angle = map(real_angle_yaw, REAL_YAW_LOW, REAL_YAW_HIGH, RED_YAW_RIGHT, RED_YAW_LEFT);
    // float yaw_position_change = Position_Control_206((float)measured_yaw_angle, (float)target_yaw_angle);
    // float yaw_velocity_change = Velocity_Control_206((float)MPU6050_Real_Data.Gyro_Z, yaw_position_change);

    // pitchyaw_control((int16_t) yaw_position_change, (int16_t) pitch_position_change);
    pitchyaw_control((int16_t) yaw_velocity_change, (int16_t)pitch_velocity_change);
}

/*
@@ Description: Top level Function to implement PID control on Pitch Servo
 @ Input:       Real angle from z axis to the position, medium 90
 @ Output:      send command to pitch servo to execute
*/
void set_Pitch_Position(int16_t real_angle_pitch)
{

    // Blue Motor
    // // PID for position
    // float target_pitch_angle = map(real_angle_pitch, REAL_PITCH_LOW, REAL_PITCH_HIGH, BLUE_PITCH_LOW, BLUE_PITCH_HIGH);
    // float pitch_position_change = Position_Control_205((float)measured_pitch_angle, (float)target_pitch_angle);
    // float pitch_velocity_change = Velocity_Control_205((float)MPU6050_Real_Data.Gyro_Y, pitch_position_change);

    // // pitchyaw_control(0, (int16_t) pitch_position_change);
    // pitchyaw_control(0, (int16_t)pitch_velocity_change);

    // Red Motor
    // PID for position
    float target_pitch_angle = map(real_angle_pitch, REAL_PITCH_LOW, REAL_PITCH_HIGH, RED_PITCH_LOW, RED_PITCH_HIGH);
    float pitch_position_change = Position_Control_205((float)measured_pitch_angle, (float)target_pitch_angle);
    float pitch_velocity_change = Velocity_Control_205((float)MPU6050_Real_Data.Gyro_Y, pitch_position_change);

    // pitchyaw_control(0, (int16_t) pitch_position_change);
    pitchyaw_control(0, (int16_t)pitch_velocity_change);
}

/*
@@ Description: Top level Function to implement PID control on Yaw Servo
 @ Input:       Real angle from x axis to the position, medium 0
 @ Output:      send command to pitch servo to execute
*/
void set_Yaw_Position(int16_t real_angle_yaw)
{

    // Blue Motor
    // // PID for position
    // float target_yaw_angle = map(real_angle_yaw, REAL_YAW_LOW, REAL_YAW_HIGH, BLUE_YAW_RIGHT, BLUE_YAW_LEFT);
    // float yaw_position_change = Position_Control_206((float)measured_yaw_angle, (float)target_yaw_angle);
    // float yaw_velocity_change = Velocity_Control_206((float)MPU6050_Real_Data.Gyro_Z, yaw_position_change);

    // // pitchyaw_control((int16_t) yaw_position_change, 0);
    // pitchyaw_control((int16_t) yaw_velocity_change, 0);


    // Red Motor
    // PID for position
    float target_yaw_angle = map(real_angle_yaw, REAL_YAW_LOW, REAL_YAW_HIGH, RED_YAW_RIGHT, RED_YAW_LEFT);
    float yaw_position_change = Position_Control_206((float)measured_yaw_angle, (float)target_yaw_angle);
    float yaw_velocity_change = Velocity_Control_206((float)MPU6050_Real_Data.Gyro_Z, yaw_position_change, v_p_206);

    // pitchyaw_control((int16_t) yaw_position_change, 0);
    pitchyaw_control((int16_t) yaw_velocity_change, 0);
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

    if((abs(current_velocity_205) < GAP) ||
        (RC_Ctl.rc.s1 == RC_SW_DOWN && RC_Ctl.rc.s2 == RC_SW_DOWN))
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

    return -output; // For Blue rover, position reading is in inverse direction
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

float Velocity_Control_206(float current_velocity_206,float target_velocity_206, float V_p)
{
    static float error_v[2] = {0.0,0.0};
    static float output = 0;
    static float inte = 0;

    if((abs(current_velocity_206) < GAP) || 
        (RC_Ctl.rc.s1 == RC_SW_DOWN && RC_Ctl.rc.s2 == RC_SW_DOWN))
    {
        current_velocity_206 = 0.0;
    }

    error_v[0] = error_v[1];
    error_v[1] = target_velocity_206 - current_velocity_206;
    inte += error_v[1];

    // output = error_v[1] * v_p_206
    //          + inte * v_i_206
    //          + (error_v[1] - error_v[0]) * v_d_206;
    output = error_v[1] * V_p
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

    return -output;
}

static long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
