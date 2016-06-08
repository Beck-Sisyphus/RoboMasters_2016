#include "main.h"

extern int16_t measured_yaw_angle;
extern int16_t measured_pitch_angle;

volatile int16_t pitch_Position;
volatile int16_t yaw_Position;
// volatile int16_t pitch_Velocity;
// volatile int16_t yaw_Velocity;

// for velocity controlling pitch and yaw with remote
// volatile int16_t remote_pitch_change;
// volatile int16_t remote_yaw_change;

volatile extern int manual_Control_Turret;

// extern MPU6050_REAL_DATA MPU6050_Real_Data;
/* Pitch Constants  */
const float v_p_205 = 15.0;
const float v_i_205 = 0.0;
const float v_d_205 = 0.01;
const float l_p_205 = 0.75;
const float l_i_205 = 0.0;
const float l_d_205 = 0.0;

/* Yaw Constants*/
// const float v_p_206_remote = 100.0;
const float v_p_206 = 10.0;
const float v_i_206 = 0.0;
const float v_d_206 = 0.0;
const float l_p_206 = 1;
const float l_i_206 = 0.0;
const float l_d_206 = 0.0;

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

    // Auto target control
    switch (ROBOT_SERIAL_NUMBER) {
      case BLUE_SAMPLE_ROBOT:
              // TODO: need to double check on the yaw coordinate system mapping
              target_pitch_angle = map(real_angle_pitch, REAL_PITCH_LOW, REAL_PITCH_HIGH, BLUE_PITCH_LOW, BLUE_PITCH_HIGH);
              target_yaw_angle = map(real_angle_yaw, REAL_YAW_LOW, REAL_YAW_HIGH, BLUE_YAW_RIGHT, BLUE_YAW_LEFT);
              break;
      case RED_SAMPLE_ROBOT:
              target_pitch_angle = map(real_angle_pitch, REAL_PITCH_LOW, REAL_PITCH_HIGH, RED_PITCH_LOW, RED_PITCH_HIGH);
              target_yaw_angle = map(real_angle_yaw, REAL_YAW_LOW, REAL_YAW_HIGH, RED_YAW_RIGHT, RED_YAW_LEFT);
              break;
      case HERO_ROBOT_CANNON:
              target_pitch_angle = map(real_angle_pitch, REAL_CANNON_PITCH_LOW, REAL_CANNON_PITCH_HIGH,
                                                         CANNON_PITCH_LOW, CANNON_PITCH_HIGH + ENCODER_MAX);
              target_yaw_angle = map(real_angle_yaw, REAL_CANNON_YAW_LEFT, REAL_CANNON_YAW_RIGHT,
                                                     CANNON_YAW_LEFT, CANNON_YAW_RIGHT);
              // Break point for pitch
              if (measured_pitch_angle < CANNON_PITCH_LOW - 1000) {
                  measured_pitch_angle = measured_pitch_angle + ENCODER_MAX;
              }
              break;
      default:break;
    }

    // PID for pitch
    pitch_position_change = Position_Control_205((float)measured_pitch_angle, (float)target_pitch_angle);
    pitch_velocity_change = Velocity_Control_205((float)MPU6050_Real_Data.Gyro_Y, pitch_position_change);

    // PID for yaw
    yaw_position_change = Position_Control_206((float)measured_yaw_angle, (float)target_yaw_angle);
    yaw_velocity_change = Velocity_Control_206((float)MPU6050_Real_Data.Gyro_Z, yaw_position_change);

    pitchyaw_control((int16_t) yaw_velocity_change, (int16_t)pitch_velocity_change);
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

    if((abs(current_velocity_205) < GAP))
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

    switch (ROBOT_SERIAL_NUMBER) {
      case BLUE_SAMPLE_ROBOT:
              // Blue pitch motor need negative feedback for speed
              output = -output; break;
      case RED_SAMPLE_ROBOT:
              // red pitch motor need positive feedback for speed
              output = output; break;
      case HERO_ROBOT_CANNON:
              // hero cannon pitch motor need negative feedback for speed
              output = -output; break;
      default:break;
    }

    return output;
}

/********************************************************************************
@@ Description: Close loop control for the pitch axis position on motor controller
 @ Input      : Current position from pitch axis
 @ Output     : Target position for pitch axis
*********************************************************************************/
static float Position_Control_205(float current_position_205,float target_position_205)
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

    switch (ROBOT_SERIAL_NUMBER) {
      case BLUE_SAMPLE_ROBOT:
              // Blue pitch motor need negative feedback
              output = -output; break;
      case RED_SAMPLE_ROBOT:
              // red pitch motor need positive feedback
              output = -output; break;
      case HERO_ROBOT_CANNON:
              // hero cannon pitch motor need positive feedback for position
              output = output; break;
      default:break;
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
    static float error_v[2] = {0.0,0.0};
    static float output = 0;
    static float inte = 0;

    if((abs(current_velocity_206) < GAP))
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
    switch (ROBOT_SERIAL_NUMBER) {
      case BLUE_SAMPLE_ROBOT:
              // Blue pitch motor need negative feedback for speed
              output = -output; break;
      case RED_SAMPLE_ROBOT:
              // red pitch motor need positive feedback for speed
              output = output; break;
      case HERO_ROBOT_CANNON:
              // hero cannon pitch motor need negative feedback for speed
              output = output; break;
      default:break;
    }

    return output;
}

/********************************************************************************
@@ Description: Close loop control for the yaw axis position on motor controller
 @ Input      : Current position from yaw axis
 @ Output     : Target position for yaw axis
*********************************************************************************/
static float Position_Control_206(float current_position_206,float target_position_206)
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
    switch (ROBOT_SERIAL_NUMBER) {
      case BLUE_SAMPLE_ROBOT:
              // Blue pitch motor need negative feedback for speed
              output = -output; break;
      case RED_SAMPLE_ROBOT:
              // red pitch motor need positive feedback for speed
              output = -output; break;
      case HERO_ROBOT_CANNON:
              // positive speed, moving right; negative feedback
              output = -output; break;
      default:break;
    }

    return output;
}

static int map(int x, int in_min, int in_max, int out_min, int out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
