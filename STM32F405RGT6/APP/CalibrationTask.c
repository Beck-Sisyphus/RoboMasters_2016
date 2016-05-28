#include "main.h"
#include "protocal.h"
#include "CalibrationTask.h"

static int16_t GimbalPitchOffset = 4500;
static int16_t GimbalYawOffset = 2320;

// These struct are used in the application
GimbalCaliStruct_t GimbalSavedCaliData;    	    //gimbal pitch yaw encoder offset
// GyroCaliStruct_t GyroSavedCaliData;     	    //gyro offset data
// AccCaliStruct_t AccSavedCaliData;    	    	//ACC offset data
// MagCaliStruct_t MagSavedCaliData;			    //Mag offset data
// PIDParamStruct_t PitchPositionSavedPID;        	//PID offset data
// PIDParamStruct_t PitchSpeedSavedPID;        	//PID offset data
// PIDParamStruct_t YawPositionSavedPID;        	//PID offset data
// PIDParamStruct_t YawSpeedSavedPID;        	    //PID offset data

void Sensor_Offset_Param_Init(void)
{
	// memcpy(&MagSavedCaliData, &(appParam->MagCaliData), sizeof((appParam->MagCaliData)));
	// memcpy(&GyroSavedCaliData, &(appParam->GyroCaliData), sizeof((appParam->GyroCaliData)));
  // memcpy(&GimbalSavedCaliData, &(appParam->GimbalCaliData), sizeof((appParam->GimbalCaliData)));

	// memcpy(&PitchPositionSavedPID, &(appParam->PitchPositionPID), sizeof((appParam->PitchPositionPID)));
	// memcpy(&PitchSpeedSavedPID, &(appParam->PitchSpeedPID), sizeof((appParam->PitchSpeedPID)));
	// memcpy(&YawPositionSavedPID, &(appParam->YawPositionPID), sizeof((appParam->YawPositionPID)));
	// memcpy(&YawSpeedSavedPID, &(appParam->YawSpeedPID), sizeof((appParam->YawSpeedPID)));
  GimbalSavedCaliData.GimbalPitchOffset = GimbalPitchOffset;
  GimbalSavedCaliData.GimbalYawOffset = GimbalYawOffset;

	GMPitchEncoder.ecd_bias = GimbalSavedCaliData.GimbalPitchOffset;
	GMYawEncoder.ecd_bias = GimbalSavedCaliData.GimbalYawOffset;
}
