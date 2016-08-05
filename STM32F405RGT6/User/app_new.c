#include "app_new.h"
#include <time.h>

#define PID_CHASSIS true

/* variables for global control */
volatile int16_t pitch_Position;
volatile int16_t yaw_Position;
volatile int16_t rotate_feedback;

volatile extern int16_t drive;
volatile extern int16_t strafe;
volatile extern int16_t rotate;

/* sensor value from the motor controller and gyro scope */
extern volatile Encoder CM1Encoder;
extern volatile Encoder CM2Encoder;
extern volatile Encoder CM3Encoder;
extern volatile Encoder CM4Encoder;
extern volatile Encoder GMYawEncoder;
extern volatile Encoder GMPitchEncoder;

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

// control the motor velocity, the higher the intervals is, the slower the motor will reach target_velocity
int16_t set_velocity(Encoder en, float target_v, PID_Regulator_t CMSpeedPID, int intervals) {
    float inter_v = target_v / intervals;
    float velocity = inter_v;
    int result_v;
    while (velocity < targe_v) {
        result_v = PID_Control((float) en.velocity_raw, (float) velocity, &CMSpeedPID);
        velocity += inter_v;
        delay(100);
    }
    return result_v;
}

int16_t set_chassis_motor_velocity(int can_address, int remote_receiver_velocity)
{
    int target_velocity = map(remote_receiver_velocity, -660, 660, -8171, 8171);
    int result_velocity;
    // result_velocity = target_velocity;
    switch (can_address) {
        case 201:
            result_velocity = set_velocity(CM1Encoder, (float) target_velocity, CM1SpeedPID, 5);
            break;
        case 202:
            result_velocity = set_velocity(CM2Encoder, (float) target_velocity, CM2SpeedPID, 5);
            break;
        case 203:
            result_velocity = set_velocity(CM3Encoder, (float) target_velocity, CM3SpeedPID, 5);
            break;
        case 204:
            result_velocity = set_velocity(CM4Encoder, (float) target_velocity, CM4SpeedPID, 5);
            break;
        default:break;
    }
    return result_velocity;
}

void CMControlLoop(void)
{
    rotate_feedback = 0.01 * PID_Control(GMYawEncoder.ecd_angle, 0, &CMRotatePID);
    int16_t target_velocity_201 = (-1*drive + strafe + rotate); //  + rotate_feedback
    int16_t target_velocity_202 = (drive + strafe + rotate); //  + rotate_feedback
    int16_t target_velocity_203 = (drive - strafe + rotate); //  + rotate_feedback
    int16_t target_velocity_204 = (-1*drive - strafe + rotate); //  + rotate_feedback
    // #if PID_CHASSIS
    int16_t motor_201_vel = set_chassis_motor_velocity(201, target_velocity_201);
    int16_t motor_202_vel = set_chassis_motor_velocity(202, target_velocity_202);
    int16_t motor_203_vel = set_chassis_motor_velocity(203, target_velocity_203);
    int16_t motor_204_vel = set_chassis_motor_velocity(204, target_velocity_204);
    // #else
        // int16_t motor_201_vel = 11 * target_velocity_201;
        // int16_t motor_202_vel = 11 * target_velocity_202;
        // int16_t motor_203_vel = 11 * target_velocity_203;
        // int16_t motor_204_vel = 11 * target_velocity_204;
    // #endif
    wheel_control(motor_201_vel, motor_202_vel, motor_203_vel, motor_204_vel);
    // wheel_control(-500, 500, 500, -500);
}

/*
@@ Description: Top level Function to implement PID control on Pitch Servo
 @ Input:       Real angle from z axis to the position, medium 90
 @ Output:      send command to pitch servo to execute
*/
void set_Pitch_Yaw_Position(int16_t real_angle_pitch, int16_t real_angle_yaw)
{
    // Coordinate change
    switch (ROBOT_SERIAL_NUMBER) {
        case BLUE_SAMPLE_ROBOT_0:
            // TODO: need to retune, the speed pid sign could be the same
            PitchPositionPID.sign = -1;
            PitchSpeedPID.sign = -1;
            YawPositionPID.sign = 1;
            YawSpeedPID.sign = -1;
            break;
        case RED_SAMPLE_ROBOT_1:
            // TODO: need to retune, the speed pid sign could be the same
            PitchPositionPID.sign = -1;
            PitchSpeedPID.sign = 1;
            YawPositionPID.sign = 1;
            YawSpeedPID.sign = 1;
            break;
        case SOLDIER_2:
            PitchPositionPID.sign = 0;
            PitchSpeedPID.sign = 0;
            YawPositionPID.sign = 0;
            YawSpeedPID.sign = 0;
            break;
        case SOLDIER_3:
            PitchPositionPID.sign = 0;
            PitchSpeedPID.sign = 0;
            YawPositionPID.sign = 0;
            YawSpeedPID.sign = 0;
            break;
        case SOLDIER_4:
            PitchPositionPID.sign = 0;
            PitchSpeedPID.sign = 0;
            YawPositionPID.sign = 0;
            YawSpeedPID.sign = 0;
            break;
        case SOLDIER_5:
            PitchPositionPID.sign = 1;
            PitchSpeedPID.sign = -1;
            YawPositionPID.sign = 1;
            YawSpeedPID.sign = 1;
            break;
        case BASE_ROBOT_6:
            PitchPositionPID.sign = 0;
            PitchSpeedPID.sign = 0;
            YawPositionPID.sign = 0;
            YawSpeedPID.sign = 0;
            break;
        case HERO_ROBOT_CANNON_7:
            // TODO: there is a gap in the turret, need to be careful
            PitchPositionPID.sign = 1;
            PitchSpeedPID.sign = -1;
            YawPositionPID.sign = -1;
            YawSpeedPID.sign = 1;
            break;
        case HERO_ROBOT_TURRET_8:
            PitchPositionPID.sign = 0;
            PitchSpeedPID.sign = 0;
            YawPositionPID.sign = 0;
            YawSpeedPID.sign = 0;
            break;
        default:break;
    }

    // PID for pitch
    float pitch_position_change_206 = PID_Control(GMPitchEncoder.ecd_angle, (float)real_angle_pitch, &PitchPositionPID);
    float pitch_velocity_change_206 = PID_Control((float)MPU6050_Real_Data.Gyro_Y, pitch_position_change_206, &PitchSpeedPID);
    // float pitch_velocity_change_206 = PID_Control((float)MPU6050_Real_Data.Gyro_Y, 0, &PitchSpeedPID);

    // PID for yaw
    float yaw_position_change_205 = PID_Control(GMYawEncoder.ecd_angle, (float)real_angle_yaw, &YawPositionPID);
    float yaw_velocity_change_205 = PID_Control((float)MPU6050_Real_Data.Gyro_Z, yaw_position_change_205, &YawSpeedPID);

    pitchyaw_control((int16_t) yaw_velocity_change_205, (int16_t)pitch_velocity_change_206);
    // pitchyaw_control((int16_t) yaw_velocity_change_205, 1000);
}

static int16_t PID_Control(float measured, float target, PID_Regulator_t * pid)
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

static int map(int x, int in_min, int in_max, int out_min, int out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
