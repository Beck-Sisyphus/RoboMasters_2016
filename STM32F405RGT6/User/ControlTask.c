#include "CanBusTask.h"
#include "ControlTask.h"

/* variables for global control */
volatile int16_t pitch_Position = 0;
volatile int16_t yaw_Position = 0;
volatile int16_t rotate_feedback;
volatile int16_t chassis_to_turret_offset = 0;

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

WorkState_e lastWorkState = PREPARE_STATE;
WorkState_e workState = PREPARE_STATE;

static void SetWorkState(WorkState_e state)
{
    workState = state;
}

WorkState_e GetWorkState(void)
{
	return workState;
}

int16_t set_chassis_motor_velocity(int can_address, int remote_receiver_velocity)
{
    int target_velocity = map(remote_receiver_velocity, -660, 660, -8171, 8171);
    int result_velocity;
    if (ROBOT_SERIAL_NUMBER == HERO_ROBOT_TURRET_8) {
        result_velocity = target_velocity;
    }
    else {
        switch (can_address) {
            case 201:
                result_velocity = PID_Control((float)CM1Encoder.velocity_raw, (float)target_velocity, &CM1SpeedPID);
                break;
            case 202:
                result_velocity = PID_Control((float)CM2Encoder.velocity_raw, (float)target_velocity, &CM2SpeedPID);
                break;
            case 203:
                result_velocity = PID_Control((float)CM3Encoder.velocity_raw, (float)target_velocity, &CM3SpeedPID);
                break;
            case 204:
                result_velocity = PID_Control((float)CM4Encoder.velocity_raw, (float)target_velocity, &CM4SpeedPID);
                break;
            default:break;
        }
    }
    return result_velocity;
}

void CMControlLoop(void)
{
  	if(GetWorkState()==PREPARE_STATE) {
        rotate_feedback = 0;
  	}
  	else {
        rotate_feedback = PID_Control(GMYawEncoder.ecd_angle, chassis_to_turret_offset, &CMRotatePID);
  	}
  	// if(Is_Lost_Error_Set(LOST_ERROR_RC))
  	// {
    // 		 drive = 0;
    //     strafe = 0;
    //     rotate = 0;
  	// }
    int16_t target_velocity_201 = (-1*drive + strafe + rotate) + rotate_feedback;
    int16_t target_velocity_202 = (drive + strafe + rotate) + rotate_feedback;
    int16_t target_velocity_203 = (drive - strafe + rotate) + rotate_feedback;
    int16_t target_velocity_204 = (-1*drive - strafe + rotate) + rotate_feedback;

    int16_t motor_201_vel = set_chassis_motor_velocity(201, target_velocity_201);
    int16_t motor_202_vel = set_chassis_motor_velocity(202, target_velocity_202);
    int16_t motor_203_vel = set_chassis_motor_velocity(203, target_velocity_203);
    int16_t motor_204_vel = set_chassis_motor_velocity(204, target_velocity_204);

    wheel_control(motor_201_vel, motor_202_vel, motor_203_vel, motor_204_vel);
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
        case SOLDIER_2: break;
        case SOLDIER_3: break;
        case SOLDIER_4:
//            PitchPositionPID.sign = 0;
  //          PitchSpeedPID.sign = 0;
    //        YawPositionPID.sign = 0;
      //      YawSpeedPID.sign = 0;
            break;
        case SOLDIER_5: break;
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

    // PID for yaw
    float yaw_position_change_205 = PID_Control(GMYawEncoder.ecd_angle, (float)real_angle_yaw, &YawPositionPID);
    float yaw_velocity_change_205 = PID_Control((float)MPU6050_Real_Data.Gyro_Z, yaw_position_change_205, &YawSpeedPID);
    // float yaw_velocity_change_205 = PID_Control((float)MPU6050_Real_Data.Gyro_Z, 0, &YawSpeedPID);

    // PID for pitch
    float pitch_position_change_206 = PID_Control(GMPitchEncoder.ecd_angle, (float)real_angle_pitch, &PitchPositionPID);
    float pitch_velocity_change_206 = PID_Control((float)MPU6050_Real_Data.Gyro_Y, pitch_position_change_206, &PitchSpeedPID);
    // float pitch_velocity_change_206 = PID_Control((float)MPU6050_Real_Data.Gyro_Y, 0, &PitchSpeedPID);

    pitchyaw_control((int16_t) yaw_velocity_change_205, (int16_t)pitch_velocity_change_206);
    // pitchyaw_control((int16_t) yaw_velocity_change_205, 1000);
}

static uint32_t time_tick_1ms = 0;
// Control task running at timer 2 interrupt with 1ms
void Control_Task(void)
{
    time_tick_1ms++;
    // WorkStateFSM();
    // WorkStateSwitchProcess();
    // //启动后根据磁力计的数据初始化四元数
    // // Initialize quaternion coordinate from magnetometer
  	// if(time_tick_1ms <100) { Init_Quaternion(); }
  	// //平台稳定启动后，复位陀螺仪模块
    // // Reset the gyroscope after the gimbal is stablized
  	// if(time_tick_1ms == PREPARE_TIME_TICK_MS/2) { GYRO_RST(); }

    // Gimbal control
    // Mode switch method, to get the target and measured value of position loop
  	// GimbalYawControlModeSwitch();
    yaw_Position = max_min_angle(yaw_Position, GMYawEncoder.motor);
    pitch_Position = max_min_angle(pitch_Position, GMPitchEncoder.motor);
    set_Pitch_Yaw_Position(pitch_Position, yaw_Position);
    //chassis motor control every 4mss
  	//if(time_tick_1ms%4 == 0)
  	//{
  		  CMControlLoop();
  	//}
}

void WorkStateFSM(void)
{
    lastWorkState = workState;
    // switch(workState)
    // {
    // 		case PREPARE_STATE:
    // 		{
    //   			if(GetInputMode() == STOP || Is_Serious_Error())
    //   			{
    //   				workState = STOP_STATE;
    //   			}
    //   			else if(time_tick_1ms > PREPARE_TIME_TICK_MS)
    //   			{
    //   				workState = NORMAL_STATE;
    //   			}
    // 		}break;
    // 		case NORMAL_STATE:
    // 		{
    //   			if(GetInputMode() == STOP || Is_Serious_Error())
    //   			{
    //   				workState = STOP_STATE;
    //   			}
    //   			else if((!IsRemoteBeingAction() ||(Get_Lost_Error(LOST_ERROR_RC) == LOST_ERROR_RC)) && GetShootState() != SHOOTING)
    //   			{
    //   				workState = STANDBY_STATE;
    //   			}
		//     }break;
    // 		case STANDBY_STATE:
    // 		{
    //   			if(GetInputMode() == STOP || Is_Serious_Error())
    //   			{
    //   				workState = STOP_STATE;
    //   			}
    //   			else if(IsRemoteBeingAction() || (GetShootState()==SHOOTING) || GetFrictionState() == FRICTION_WHEEL_START_TURNNING)
    //   			{
    //   				workState = NORMAL_STATE;
    //   			}
    // 		}break;
    // 		case STOP_STATE:
    // 		{
    //   			if(GetInputMode() != STOP && !Is_Serious_Error())
    //   			{
    //   				workState = PREPARE_STATE;
    //   			}
    // 		}break;
    //     default: {}
    // }
}

static void WorkStateSwitchProcess(void)
{
    // If switch from other state to prepare state, initialize all parameters
  	if((lastWorkState != workState) && (workState == PREPARE_STATE))
  	{
    		// ControtLoopTaskInit();
    		// RemoteTaskInit();
  	}
}

static int16_t max_min_angle(int16_t user_input, volatile motor_mapping_t * motor)
{
    user_input = min(user_input, motor->real_high);
    user_input = max(user_input, motor->real_low);
	  return user_input;
}

static int16_t min(int16_t a, int16_t b) {
    if(a<b) {
        return a;
    } else {
        return b;
    }
}

static int16_t max(int16_t a, int16_t b) {
    if(a>b) {
        return a;
    } else {
        return b;
    }
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
