#include "app_new.h"

#define PID_CHASSIS false

/* variables for global control */
volatile int16_t pitch_Position;
volatile int16_t yaw_Position;

/* sensor value from the motor controller and gyro scope */
extern int16_t measured_201_speed;
extern int16_t measured_202_speed;
extern int16_t measured_203_speed;
extern int16_t measured_204_speed;

extern int16_t measured_201_angle;
extern int16_t measured_202_angle;
extern int16_t measured_203_angle;
extern int16_t measured_204_angle;

extern int16_t measured_yaw_angle;
extern int16_t measured_pitch_angle;
extern MPU6050_REAL_DATA MPU6050_Real_Data;

const float v_p_201 = 1.0;
const float v_i_201 = 0;
const float v_d_201 = 0;

const float v_p_202 = 1.0;
const float v_i_202 = 0;
const float v_d_202 = 0;

const float v_p_203 = 1.0;
const float v_i_203 = 0;
const float v_d_203 = 0;

const float v_p_204 = 1.0;
const float v_i_204 = 0;
const float v_d_204 = 0;

const float v_p_205 = 15.0;
const float v_i_205 = 0.0;
const float v_d_205 = 0.01;
const float l_p_205 = 0.75;
const float l_i_205 = 0.0;
const float l_d_205 = 0.0;

const float v_p_206 = 10.0;
const float v_i_206 = 0.0;
const float v_d_206 = 0.0;
const float l_p_206 = 1;
const float l_i_206 = 0.0;
const float l_d_206 = 0.0;

int16_t set_chassis_motor_velocity(int can_address, int remote_receiver_velocity)
{
    int target_velocity = map(remote_receiver_velocity, -660, 660, -7780, 7780);
    int result_velocity;
    switch (can_address) {
        case 201:
            result_velocity = PID_Control((float)measured_201_speed, (float)target_velocity, -1, v_p_201, v_i_201, v_d_201);
            break;
        case 202:
            result_velocity = PID_Control((float)measured_202_speed, (float)target_velocity, -1, v_p_202, v_i_202, v_d_202);
            break;
        case 203:
            result_velocity = PID_Control((float)measured_203_speed, (float)target_velocity, -1, v_p_203, v_i_203, v_d_203);
            break;
        case 204:
            result_velocity = PID_Control((float)measured_204_speed, (float)target_velocity, -1, v_p_204, v_i_204, v_d_204);
            break;
        default:break;
    }
    return result_velocity;
}

/*
@@ Description: Top level Function to implement PID control on Pitch Servo
 @ Input:       Real angle from z axis to the position, medium 90
 @ Output:      send command to pitch servo to execute
*/
void set_Pitch_Yaw_Position(int16_t real_angle_pitch, int16_t real_angle_yaw)
{
    float target_pitch_angle_205;
    int sign_pitch_position_205 = 0;
    int sign_pitch_velocity_205 = 0;

    float target_yaw_angle_206;
    int sign_yaw_position_206 = 0;
    int sign_yaw_velocity_206 = 0;

    // Coordinate change
    switch (ROBOT_SERIAL_NUMBER) {
        case BLUE_SAMPLE_ROBOT_0:
            target_pitch_angle_205 = map(real_angle_pitch, REAL_PITCH_LOW, REAL_PITCH_HIGH, BLUE_PITCH_LOW, BLUE_PITCH_HIGH);
            target_yaw_angle_206 = map(real_angle_yaw, REAL_YAW_LOW, REAL_YAW_HIGH, BLUE_YAW_RIGHT, BLUE_YAW_LEFT);
            sign_pitch_position_205 = -1;
            sign_pitch_velocity_205 = -1;
            sign_yaw_position_206 = -1;
            sign_yaw_velocity_206 = -1;
            break;
        case RED_SAMPLE_ROBOT_1:
            target_pitch_angle_205 = map(real_angle_pitch, REAL_PITCH_LOW, REAL_PITCH_HIGH, RED_PITCH_LOW, RED_PITCH_HIGH);
            target_yaw_angle_206 = map(real_angle_yaw, REAL_YAW_LOW, REAL_YAW_HIGH, RED_YAW_RIGHT, RED_YAW_LEFT);
            sign_pitch_position_205 = -1;
            sign_pitch_velocity_205 = 1;
            sign_yaw_position_206 = -1;
            sign_yaw_velocity_206 = 1;
            break;
        case SOLDIER_2:
            target_pitch_angle_205 = map(real_angle_pitch, REAL_PITCH_LOW, REAL_PITCH_HIGH, 0, 0);
            target_yaw_angle_206 = map(real_angle_yaw, REAL_YAW_LOW, REAL_YAW_HIGH, 0, 0);
            sign_pitch_position_205 = 0;
            sign_pitch_velocity_205 = 0;
            sign_yaw_position_206 = 0;
            sign_yaw_velocity_206 = 0;
            break;
        case SOLDIER_3:
            target_pitch_angle_205 = map(real_angle_pitch, REAL_PITCH_LOW, REAL_PITCH_HIGH, 0, 0);
            target_yaw_angle_206 = map(real_angle_yaw, REAL_YAW_LOW, REAL_YAW_HIGH, 0, 0);
            sign_pitch_position_205 = 0;
            sign_pitch_velocity_205 = 0;
            sign_yaw_position_206 = 0;
            sign_yaw_velocity_206 = 0;
            break;
        case SOLDIER_4:
            target_pitch_angle_205 = map(real_angle_pitch, REAL_PITCH_LOW, REAL_PITCH_HIGH, 0, 0);
            target_yaw_angle_206 = map(real_angle_yaw, REAL_YAW_LOW, REAL_YAW_HIGH, 0, 0);
            sign_pitch_position_205 = 0;
            sign_pitch_velocity_205 = 0;
            sign_yaw_position_206 = 0;
            sign_yaw_velocity_206 = 0;
            break;
        case SOLDIER_5:
            target_pitch_angle_205 = map(real_angle_pitch, REAL_PITCH_LOW, REAL_PITCH_HIGH, 0, 0);
            target_yaw_angle_206 = map(real_angle_yaw, REAL_YAW_LOW, REAL_YAW_HIGH, 0, 0);
            sign_pitch_position_205 = 0;
            sign_pitch_velocity_205 = 0;
            sign_yaw_position_206 = 0;
            sign_yaw_velocity_206 = 0;
            break;
        case BASE_ROBOT_6:
            target_pitch_angle_205 = map(real_angle_pitch, REAL_PITCH_LOW, REAL_PITCH_HIGH, 0, 0);
            target_yaw_angle_206 = map(real_angle_yaw, REAL_YAW_LOW, REAL_YAW_HIGH, 0, 0);
            sign_pitch_position_205 = 0;
            sign_pitch_velocity_205 = 0;
            sign_yaw_position_206 = 0;
            sign_yaw_velocity_206 = 0;
            break;
        case HERO_ROBOT_CANNON_7:
            target_pitch_angle_205 = map(real_angle_pitch, REAL_CANNON_PITCH_LOW, REAL_CANNON_PITCH_HIGH,
                                                       CANNON_PITCH_LOW, CANNON_PITCH_HIGH + ENCODER_MAX);
            target_yaw_angle_206 = map(real_angle_yaw, REAL_CANNON_YAW_LEFT, REAL_CANNON_YAW_RIGHT,
                                                   CANNON_YAW_LEFT, CANNON_YAW_RIGHT);
            // Break point for pitch
            if (measured_pitch_angle < CANNON_PITCH_LOW - 1000) {
                measured_pitch_angle = measured_pitch_angle + ENCODER_MAX;
            }
            sign_pitch_position_205 = 1;
            sign_pitch_velocity_205 = -1;
            sign_yaw_position_206 = -1;
            sign_yaw_velocity_206 = 1;
            break;
        case HERO_ROBOT_TURRET_8:
            target_pitch_angle_205 = map(real_angle_pitch, REAL_PITCH_LOW, REAL_PITCH_HIGH, 0, 0);
            target_yaw_angle_206 = map(real_angle_yaw, REAL_YAW_LOW, REAL_YAW_HIGH, 0, 0);
            sign_pitch_position_205 = 0;
            sign_pitch_velocity_205 = 0;
            sign_yaw_position_206 = 0;
            sign_yaw_velocity_206 = 0;
            break;
        default:break;
    }

    // PID for pitch
    float pitch_position_change_205 = PID_Control((float)measured_pitch_angle, (float)target_pitch_angle_205, sign_pitch_position_205, l_p_205, l_i_205, l_d_205);
    float pitch_velocity_change_205 = PID_Control((float)MPU6050_Real_Data.Gyro_Y, pitch_position_change_205, sign_pitch_velocity_205, v_p_205, v_i_205, v_d_205);

    // PID for yaw
    float yaw_position_change_206 = PID_Control((float)measured_yaw_angle, (float)target_yaw_angle_206, sign_yaw_position_206, l_p_206, l_i_206, l_d_206);
    float yaw_velocity_change_206 = PID_Control((float)MPU6050_Real_Data.Gyro_Z, yaw_position_change_206, sign_yaw_velocity_206, v_p_206, v_i_206, v_d_206);

    pitchyaw_control((int16_t) yaw_velocity_change_206, (int16_t)pitch_velocity_change_205);
}

static int16_t PID_Control(float measured, float target, int sign, const float p, const float i, const float d)
{
    static float error_v[2] = {0.0,0.0};
    static float output = 0;
    static float inte = 0;

    if(abs(measured) < GAP) { measured = 0.0;}

    error_v[0] = error_v[1];
    error_v[1] = target - measured;
    inte += error_v[1];

    output = error_v[1] * p
            + inte * i
             + (error_v[1] - error_v[0]) * d;
    output = output * sign;

    if(output > ESC_MAX) { output = ESC_MAX; }
    if(output < -ESC_MAX){ output = -ESC_MAX;}

    return (int16_t)output; // For Blue rover, position reading is in inverse direction
}

static int map(int x, int in_min, int in_max, int out_min, int out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
