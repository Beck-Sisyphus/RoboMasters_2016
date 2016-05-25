#include "main.h"

PID_Regulator_t GMPPositionPID = GIMBAL_MOTOR_PITCH_POSITION_PID_DEFAULT;
PID_Regulator_t GMPSpeedPID = GIMBAL_MOTOR_PITCH_SPEED_PID_DEFAULT;
PID_Regulator_t GMYPositionPID = GIMBAL_MOTOR_YAW_POSITION_PID_DEFAULT;
PID_Regulator_t GMYSpeedPID = GIMBAL_MOTOR_YAW_SPEED_PID_DEFAULT;

/*--------------------------------------------CTRL Variables----------------------------------------*/
WorkState_e lastWorkState = PREPARE_STATE;
WorkState_e workState = PREPARE_STATE;

static uint32_t time_tick_1ms = 0;
/*
*********************************************************************************************************
*                                            FUNCTIONS
*********************************************************************************************************
*/

static void SetWorkState(WorkState_e state)
{
    workState = state;
}

WorkState_e GetWorkState()
{
	return workState;
}

//控制任务，放在timer6 1ms定时中断中执行
// Control task running at timer 6 interrupt with 1ms
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
  //模式切换处理，得到位置环的设定值和给定值
  // Mode switch method, to get the target and measured value of position loop
	GimbalYawControlModeSwitch();
  GMPitchControlLoop();
	GMYawControlLoop();
	SetGimbalMotorOutput();

	// //chassis motor control
	// if(time_tick_1ms%4 == 0)         //motor control frequency 4ms
	// {
	// 	// Supervise task //监控任务
	// 	SuperviseTask();
	// 	// Chassis control task//底盘控制任务
	// 	CMControlLoop();
  //   // Shooting mechanism control task//发射机构控制任务
	// 	ShooterMControlLoop();
	// }
}

void WorkStateFSM(void)
{
	lastWorkState = workState;
	switch(workState)
	{
		case PREPARE_STATE:
		{
			if(GetInputMode() == STOP || Is_Serious_Error())
			{
				workState = STOP_STATE;
			}
			else if(time_tick_1ms > PREPARE_TIME_TICK_MS)
			{
				workState = SEMI_AUTONOMOUS_STATE;
			}
		}break;
		case SEMI_AUTONOMOUS_STATE:
		{
			if(GetInputMode() == STOP || Is_Serious_Error())
			{
				workState = STOP_STATE;
			}
			else if((!IsRemoteBeingAction() ||(Get_Lost_Error(LOST_ERROR_RC) == LOST_ERROR_RC)) && GetShootState() != SHOOTING)
			{
				workState = STANDBY_STATE;
			}
		}break;
    case MANUAL_STATE:
    {
      if(GetInputMode() == STOP || Is_Serious_Error())
      {
        workState = STOP_STATE;
      }
    }break;
		case STANDBY_STATE:
		{
			if(GetInputMode() == STOP || Is_Serious_Error())
			{
				workState = STOP_STATE;
			}
			else if(IsRemoteBeingAction() || (GetShootState()==SHOOTING) || GetFrictionState() == FRICTION_WHEEL_START_TURNNING)
			{
				workState = SEMI_AUTONOMOUS_STATE;
			}
		}break;
		case STOP_STATE:
		{
			if(GetInputMode() != STOP && !Is_Serious_Error())
			{
				workState = PREPARE_STATE;
			}
		}break;
		default:
		{

		}
	}
}

static void WorkStateSwitchProcess(void)
{
	//如果从其他模式切换到prepare模式，要将一系列参数初始化
  // If switch from other state to prepare state, initialize all parameters
	if((lastWorkState != workState) && (workState == PREPARE_STATE))
	{
		ControtLoopTaskInit();
		RemoteTaskInit();
	}
}

// Pitch axis double loop control
void GMPitchControlLoop(void)
{
	GMPPositionPID.kp = PITCH_POSITION_KP_DEFAULTS;
	GMPPositionPID.ki = PITCH_POSITION_KI_DEFAULTS;
	GMPPositionPID.kd = PITCH_POSITION_KD_DEFAULTS;

	GMPSpeedPID.kp = PITCH_SPEED_KP_DEFAULTS;
	GMPSpeedPID.ki = PITCH_SPEED_KI_DEFAULTS;
	GMPSpeedPID.kd = PITCH_SPEED_KD_DEFAULTS;

	GMPPositionPID.ref = GimbalRef.pitch_angle_dynamic_ref;
	// GMPPositionPID.fdb = -GMPitchEncoder.ecd_angle * GMPitchRamp.Calc(&GMPitchRamp);    // With ramp function
  GMPPositionPID.fdb = -GMPitchEncoder.ecd_angle; // Without ramp function
	GMPPositionPID.Calc(&GMPPositionPID);   // Pitch axis position control loop
	//pitch speed control
	GMPSpeedPID.ref = GMPPositionPID.output;
	GMPSpeedPID.fdb = MPU6050_Real_Data.Gyro_Y;
	GMPSpeedPID.Calc(&GMPSpeedPID);
}

void GMYawControlLoop(void)
{
	GMYPositionPID.kp = YAW_POSITION_KP_DEFAULTS;
	GMYPositionPID.ki = YAW_POSITION_KI_DEFAULTS;
	GMYPositionPID.kd = YAW_POSITION_KD_DEFAULTS;

	GMYSpeedPID.kp = YAW_SPEED_KP_DEFAULTS;
	GMYSpeedPID.ki = YAW_SPEED_KI_DEFAULTS;
	GMYSpeedPID.kd = YAW_SPEED_KD_DEFAULTS;

  GMYPositionPID.ref = GimbalRef.yaw_angle_dynamic_ref;
  GMYPositionPID.fdb = -GMYawEncoder.ecd_angle;
	GMYPositionPID.Calc(&GMYPositionPID);
	//yaw speed control
	GMYSpeedPID.ref = GMYPositionPID.output;
	GMYSpeedPID.fdb = MPU6050_Real_Data.Gyro_Z;
	GMYSpeedPID.Calc(&GMYSpeedPID);
}

void SetGimbalMotorOutput(void)
{
  // // Gimbal control output //云台控制输出
	// if((GetWorkState() == STOP_STATE) ||Is_Serious_Error() || GetWorkState() == CALI_STATE)
	// {
	// 	Set_Gimbal_Current(0, 0);     //yaw + pitch
	// }
	// else
	// {
  Set_Gimbal_Current(-(int16_t)GMYSpeedPID.output, -(int16_t)GMPSpeedPID.output);
	// }
}

// Control Task initialization //控制任务初始化程序
void ControtLoopTaskInit(void)
{
	// Clear the timing  //计数初始化
	time_tick_1ms = 0;   // Clear the count in the interrupt //中断中的计数清零
	// Parameter initialization	//程序参数初始化
	AppParamInit();
	// initialize parameter deviation after calibration	//校准后参数偏差值初始化
	Sensor_Offset_Param_Init(&gAppParamStruct);
	// Set the work state	//设置工作模式
	SetWorkState(PREPARE_STATE);

	// // Ramp function initialization	//斜坡初始化
	// GMPitchRamp.SetScale(&GMPitchRamp, PREPARE_TIME_TICK_MS);
	// GMYawRamp.SetScale(&GMYawRamp, PREPARE_TIME_TICK_MS);
	// GMPitchRamp.ResetCounter(&GMPitchRamp);
	// GMYawRamp.ResetCounter(&GMYawRamp);

	// Initialize the given angle for gimbal	//云台给定角度初始化
	GimbalRef.pitch_angle_dynamic_ref = 0.0f;
	GimbalRef.yaw_angle_dynamic_ref = 0.0f;

  // // monitoring task initialization //监控任务初始化
  // LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_RC));
  // LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_IMU));
  // LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_ZGYRO));
  // LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_MOTOR1));
  // LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_MOTOR2));
  // LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_MOTOR3));
  // LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_MOTOR4));
  // LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_MOTOR5));
  // LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_MOTOR6));
  // LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_DEADLOCK));
  // LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_NOCALI));

	// // PID initialization
	// ShootMotorSpeedPID.Reset(&ShootMotorSpeedPID);
	GMPPositionPID.Reset(&GMPPositionPID);
	GMPSpeedPID.Reset(&GMPSpeedPID);
	GMYPositionPID.Reset(&GMYPositionPID);
	GMYSpeedPID.Reset(&GMYSpeedPID);
	// CMRotatePID.Reset(&CMRotatePID);
	// CM1SpeedPID.Reset(&CM1SpeedPID);
	// CM2SpeedPID.Reset(&CM2SpeedPID);
	// CM3SpeedPID.Reset(&CM3SpeedPID);
	// CM4SpeedPID.Reset(&CM4SpeedPID);
}
