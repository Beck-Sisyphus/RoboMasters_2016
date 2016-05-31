#ifndef _CALIBRATION_TASK_H_
#define _CALIBRATION_TASK_H_
#include "main.h"

typedef __packed struct
{
    int16_t     GimbalYawOffset;
    int16_t     GimbalPitchOffset;
    // uint8_t     GimbalCaliFlag;
}GimbalCaliStruct_t;

typedef __packed struct
{
    int16_t     GyroXOffset;
    int16_t     GyroYOffset;
    int16_t     GyroZOffset;
    // uint8_t     GyroCaliFlag;
}GyroCaliStruct_t;

typedef __packed struct
{
    int16_t     AccXOffset;
    int16_t     AccYOffset;
    int16_t     AccZOffset;
    float       AccXScale;
    float       AccYScale;
    float       AccZScale;
    // uint8_t     AccCaliFlag;
}AccCaliStruct_t;

typedef __packed struct
{
    int16_t     MagXOffset;
    int16_t     MagYOffset;
    int16_t     MagZOffset;
    float       MagXScale;
    float       MagYScale;
    float       MagZScale;
    uint8_t     MagCaliFlag;
}MagCaliStruct_t;

typedef __packed struct
{
	int8_t pid_type;		// position PID
	int8_t motor_type;   //motor type ie: pitch yaw 201 202 203 204
	int16_t kp_offset;
	int16_t ki_offset;
	int16_t kd_offset;
}PIDParamStruct_t;

typedef __packed struct
{
    // uint8_t     ParamSavedFlag;    				//header
    // uint32_t    FirmwareVersion;    			//version
    GimbalCaliStruct_t GimbalCaliData;    //gimbal pitch yaw encoder offset
    GyroCaliStruct_t   GyroCaliData;      //gyro offset data
    AccCaliStruct_t    AccCaliData;    		//ACC offset data
    MagCaliStruct_t    MagCaliData;				//Mag offset data
  	PIDParamStruct_t   PitchPositionPID;
  	PIDParamStruct_t   PitchSpeedPID;
  	PIDParamStruct_t   YawPositionPID;
  	PIDParamStruct_t   YawSpeedPID;
}AppParam_t;

extern GimbalCaliStruct_t GimbalSavedCaliData;

void Sensor_Offset_Param_Init(void);

#endif
