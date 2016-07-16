#include "app_new.h"

#define PID_CHASSIS true

/* variables for global control */
volatile int16_t pitch_Position;
volatile int16_t yaw_Position;

/* sensor value from the motor controller and gyro scope */
extern int16_t measured_201_speed;
extern int16_t measured_202_speed;
extern int16_t measured_203_speed;
extern int16_t measured_204_speed;

extern float measured_201_angle;
extern float measured_202_angle;
extern float measured_203_angle;
extern float measured_204_angle;

extern int16_t measured_yaw_angle;
extern int16_t measured_pitch_angle;
extern MPU6050_REAL_DATA MPU6050_Real_Data;

PID_Regulator_t PitchSpeedPID = PITCH_SPEED_PID_DEFAULT;
PID_Regulator_t PitchPositionPID = PITCH_POSITION_PID_DEFAULT;
PID_Regulator_t YawSpeedPID = YAW_SPEED_PID_DEFAULT;
PID_Regulator_t YawPositionPID = YAW_POSITION_PID_DEFAULT;

PID_Regulator_t CMRotatePID = CHASSIS_MOTOR_ROTATE_PID_DEFAULT;
PID_Regulator_t CM1SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t CM2SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t CM3SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t CM4SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;

const gimbal_mapping_t gimbal_0 = GIMBAL_BLUE_SAMPLE_ROBOT_0;
const gimbal_mapping_t gimbal_1 = GIMBAL_RED_SAMPLE_ROBOT_1;
const gimbal_mapping_t gimbal_2 = GIMBAL_DEFAULT;
const gimbal_mapping_t gimbal_3 = GIMBAL_DEFAULT;
const gimbal_mapping_t gimbal_4 = GIMBAL_DEFAULT;
const gimbal_mapping_t gimbal_5 = GIMBAL_SOLDIER_5;
const gimbal_mapping_t gimbal_6 = GIMBAL_DEFAULT;
const gimbal_mapping_t gimbal_7 = GIMBAL_HERO_ROBOT_CANNON_7;
const gimbal_mapping_t gimbal_8 = GIMBAL_DEFAULT;

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
    int target_velocity = map(remote_receiver_velocity, -660, 660, -8171, 8171);
    int result_velocity;
    // result_velocity = target_velocity;
    switch (can_address) {
        case 201:
            result_velocity = PID_Control_test((float)measured_201_speed, (float)target_velocity, &CM1SpeedPID);
            break;
        case 202:
            result_velocity = PID_Control_test((float)measured_202_speed, (float)target_velocity, &CM2SpeedPID);
            break;
        case 203:
            result_velocity = PID_Control_test((float)measured_203_speed, (float)target_velocity, &CM3SpeedPID);
            break;
        case 204:
            result_velocity = PID_Control_test((float)measured_204_speed, (float)target_velocity, &CM4SpeedPID);
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
            target_pitch_angle_205 = map_motor(real_angle_pitch, &gimbal_0.pitch);
            target_yaw_angle_206   = map_motor(real_angle_pitch, &gimbal_0.yaw);
            // target_pitch_angle_205 = map(real_angle_pitch, REAL_PITCH_LOW, REAL_PITCH_HIGH, BLUE_PITCH_LOW, BLUE_PITCH_HIGH);
            // target_yaw_angle_206 = map(real_angle_yaw, REAL_YAW_LOW, REAL_YAW_HIGH, BLUE_YAW_LEFT, BLUE_YAW_RIGHT);
            sign_pitch_position_205 = -1;
            sign_pitch_velocity_205 = -1;
            sign_yaw_position_206 = 1;
            sign_yaw_velocity_206 = -1;
            break;
        case RED_SAMPLE_ROBOT_1:
            target_pitch_angle_205 = map_motor(real_angle_pitch, &gimbal_1.pitch);
            target_yaw_angle_206   = map_motor(real_angle_pitch, &gimbal_1.yaw);
            // target_pitch_angle_205 = map(real_angle_pitch, REAL_PITCH_LOW, REAL_PITCH_HIGH, RED_PITCH_LOW, RED_PITCH_HIGH);
            // target_yaw_angle_206 = map(real_angle_yaw, REAL_YAW_LOW, REAL_YAW_HIGH, RED_YAW_LEFT, RED_YAW_RIGHT);
            sign_pitch_position_205 = -1;
            sign_pitch_velocity_205 = 1;
            sign_yaw_position_206 = 1;
            sign_yaw_velocity_206 = 1;
            break;
        case SOLDIER_2:
            target_pitch_angle_205 = map_motor(real_angle_pitch, &gimbal_2.pitch);
            target_yaw_angle_206   = map_motor(real_angle_pitch, &gimbal_2.yaw);
            // target_pitch_angle_205 = map(real_angle_pitch, REAL_PITCH_LOW, REAL_PITCH_HIGH, 0, 0);
            // target_yaw_angle_206 = map(real_angle_yaw, REAL_YAW_LOW, REAL_YAW_HIGH, 0, 0);
            sign_pitch_position_205 = 0;
            sign_pitch_velocity_205 = 0;
            sign_yaw_position_206 = 0;
            sign_yaw_velocity_206 = 0;
            break;
        case SOLDIER_3:
            target_pitch_angle_205 = map_motor(real_angle_pitch, &gimbal_3.pitch);
            target_yaw_angle_206   = map_motor(real_angle_pitch, &gimbal_3.yaw);
            // target_pitch_angle_205 = map(real_angle_pitch, REAL_PITCH_LOW, REAL_PITCH_HIGH, 0, 0);
            // target_yaw_angle_206 = map(real_angle_yaw, REAL_YAW_LOW, REAL_YAW_HIGH, 0, 0);
            sign_pitch_position_205 = 0;
            sign_pitch_velocity_205 = 0;
            sign_yaw_position_206 = 0;
            sign_yaw_velocity_206 = 0;
            break;
        case SOLDIER_4:
            target_pitch_angle_205 = map_motor(real_angle_pitch, &gimbal_4.pitch);
            target_yaw_angle_206   = map_motor(real_angle_pitch, &gimbal_4.yaw);
            // target_pitch_angle_205 = map(real_angle_pitch, REAL_PITCH_LOW, REAL_PITCH_HIGH, 0, 0);
            // target_yaw_angle_206 = map(real_angle_yaw, REAL_YAW_LOW, REAL_YAW_HIGH, 0, 0);
            sign_pitch_position_205 = 0;
            sign_pitch_velocity_205 = 0;
            sign_yaw_position_206 = 0;
            sign_yaw_velocity_206 = 0;
            break;
        case SOLDIER_5:
            target_pitch_angle_205 = map_motor(real_angle_pitch, &gimbal_5.pitch);
            target_yaw_angle_206   = map_motor(real_angle_pitch, &gimbal_5.yaw);
            // target_pitch_angle_205 = map(real_angle_pitch, REAL_PITCH_LOW, REAL_PITCH_HIGH, 0, 0);
            // target_yaw_angle_206 = map(real_angle_yaw, REAL_YAW_LOW, REAL_YAW_HIGH, 0, 0);
            sign_pitch_position_205 = 0;
            sign_pitch_velocity_205 = 0;
            sign_yaw_position_206 = 0;
            sign_yaw_velocity_206 = 0;
            break;
        case BASE_ROBOT_6:
            target_pitch_angle_205 = map_motor(real_angle_pitch, &gimbal_6.pitch);
            target_yaw_angle_206   = map_motor(real_angle_pitch, &gimbal_6.yaw);
            // target_pitch_angle_205 = map(real_angle_pitch, REAL_PITCH_LOW, REAL_PITCH_HIGH, 0, 0);
            // target_yaw_angle_206 = map(real_angle_yaw, REAL_YAW_LOW, REAL_YAW_HIGH, 0, 0);
            sign_pitch_position_205 = 0;
            sign_pitch_velocity_205 = 0;
            sign_yaw_position_206 = 0;
            sign_yaw_velocity_206 = 0;
            break;
        case HERO_ROBOT_CANNON_7:
            target_pitch_angle_205 = map_motor(real_angle_pitch, &gimbal_7.pitch);
            target_yaw_angle_206   = map_motor(real_angle_pitch, &gimbal_7.yaw);
            // target_pitch_angle_205 = map(real_angle_pitch, REAL_CANNON_PITCH_LOW, REAL_CANNON_PITCH_HIGH,
            //                                            CANNON_PITCH_LOW, CANNON_PITCH_HIGH + ENCODER_MAX);
            // target_yaw_angle_206 = map(real_angle_yaw, REAL_CANNON_YAW_LEFT, REAL_CANNON_YAW_RIGHT,
            //                                        CANNON_YAW_LEFT, CANNON_YAW_RIGHT);
            // // Break point for pitch
            // if (measured_pitch_angle < CANNON_PITCH_LOW - 1000) {
            //     measured_pitch_angle = measured_pitch_angle + ENCODER_MAX;
            // }
            sign_pitch_position_205 = 1;
            sign_pitch_velocity_205 = -1;
            sign_yaw_position_206 = -1;
            sign_yaw_velocity_206 = 1;
            break;
        case HERO_ROBOT_TURRET_8:
            target_pitch_angle_205 = map_motor(real_angle_pitch, &gimbal_8.pitch);
            target_yaw_angle_206   = map_motor(real_angle_pitch, &gimbal_8.yaw);
            // target_pitch_angle_205 = map(real_angle_pitch, REAL_PITCH_LOW, REAL_PITCH_HIGH, 0, 0);
            // target_yaw_angle_206 = map(real_angle_yaw, REAL_YAW_LOW, REAL_YAW_HIGH, 0, 0);
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

static int16_t PID_Control_test(float measured, float target, PID_Regulator_t * pid)
{
    static float error_v[2] = {0.0,0.0};
    static float output = 0;
    static float inte = 0;

    if(abs(measured) < GAP) { measured = 0.0;}

    error_v[0] = error_v[1];
    error_v[1] = target - measured;
    inte += error_v[1];

    output = error_v[1] * pid->kp
            + inte * pid->ki
             + (error_v[1] - error_v[0]) * pid->kd;
    output = output * pid->sign;

    if(output > ESC_MAX) { output = ESC_MAX; }
    if(output < -ESC_MAX){ output = -ESC_MAX;}
    return (int16_t)output;
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

static int map_motor(int x, const motor_mapping_t * map)
{
  return (x - map->real_low) * (map->ecd_high - map->ecd_low)\
          / ( map->real_low - map->real_high) + map->ecd_low;
}

static int map(int x, int in_min, int in_max, int out_min, int out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
