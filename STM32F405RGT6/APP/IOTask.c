#include "main.h"
#include "protocal.h"

static UploadParamType_e upload_type = REIMU;  //上传数据的类型

void UploadParameter(void)
{
    static int16_t ax, ay, az, gx, gy, gz, hx, hy, hz;
    switch(upload_type)
    {
        case REIMU:
        {
            //GMPitchEncoder_ecder
            IMU_Info_Send((int16_t)(angle[0]*10.0f),(int16_t)(angle[1]*10.0f),(int16_t)(angle[2]*10.0f),(int16_t)GMYawEncoder.raw_value,(int16_t)GMPitchEncoder.raw_value, 0);
            upload_type = REVERSION; //更改状态
        }	break;
        // case REVERSION:
        // {
        //     Version_Send(VERSION);   //发送软件版本号
        //     upload_type = REPID; //更改状态
        // }break;
        case REPID:
        {
            PID_Paremeter_Send(GMPPositionPID.kp, GMPPositionPID.ki,GMPPositionPID.kd, GMPSpeedPID.kp,GMPSpeedPID.ki,GMPSpeedPID.kd,GMYPositionPID.kp,GMYPositionPID.ki,GMYPositionPID.kd,GMYSpeedPID.kp,GMYSpeedPID.ki,GMYSpeedPID.kd);
            upload_type = REERROR; //更改状态
        }break;
        case REERROR:
        {
            Robot_Error_Code_Send(Get_Lost_Error(LOST_ERROR_ALL));
            upload_type = REMOV;
        }break;
        case REMOV:
        {
            MPU6050_getlastMotion6(&ax, &ay, &az, &gx, &gy, &gz);   //发送的是原始数据
            HMC58X3_getlastValues(&hx, &hy, &hz);
            Sensor_Info_Send(ax, ay, az, gx, gy, gz, hx, hy, hz);
            upload_type = REHMC; //如果正在进行磁标定，则发送当前磁力计标定值
        }break;
        case REHMC:
        {
            Mag_Cali_Info_Send(MagMaxMinData.MaxMagX,MagMaxMinData.MaxMagY,MagMaxMinData.MaxMagZ,MagMaxMinData.MinMagX,MagMaxMinData.MinMagY,MagMaxMinData.MinMagZ);
            upload_type = REOFFSET;
        }break;
        case REOFFSET:         //发送校准数据
        {
            Offset_Info_Send(Is_AppParam_Calied(), gAppParamStruct.GyroCaliData.GyroXOffset, gAppParamStruct.GyroCaliData.GyroYOffset, gAppParamStruct.GyroCaliData.GyroZOffset, \
            gAppParamStruct.MagCaliData.MagXOffset, gAppParamStruct.MagCaliData.MagYOffset, gAppParamStruct.MagCaliData.MagZOffset, \
            gAppParamStruct.GimbalCaliData.GimbalYawOffset, gAppParamStruct.GimbalCaliData.GimbalPitchOffset);
            upload_type = REIMU;          //数据不会实时刷新???因为没有更新config
        }break;
        default:
        {
        }break;
    }

}
