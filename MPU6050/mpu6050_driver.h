#ifndef __MPU6050_DRIVER_H__
#define __MPU6050_DRIVER_H__

typedef struct __MPU6050_RAW_Data__
{
    short Accel_X;  // Original register value for X axis acceleration 
    short Accel_Y;  // Original register value for Y axis acceleration 
    short Accel_Z;  // Original register value for Z axis acceleration 
    short Temp;     // Original register value for temperature 
    short Gyro_X;   // Original register value for X axis angular acceleration
    short Gyro_Y;   // Original register value for Y axis angular acceleration
    short Gyro_Z;   // Original register value for Z axis angular acceleration
}MPU6050_RAW_DATA;

typedef struct __MPU6050_REAL_Data__
{
    float Accel_X;  // Converted value for X axis acceleration 
    float Accel_Y;  // Converted value for Y axis acceleration 
    float Accel_Z;  // Converted value for Z axis acceleration 
    float Temp;     // Converted value for temperature in Celsius 
    float Gyro_X;   // Converted value for X axis angular acceleration 
    float Gyro_Y;   // Converted value for Y axis angular acceleration 
    float Gyro_Z;   // Converted value for Z axis angular acceleration 
}MPU6050_REAL_DATA;

extern MPU6050_RAW_DATA    MPU6050_Raw_Data; 
extern MPU6050_REAL_DATA   MPU6050_Real_Data;

int MPU6050_Initialization(void);
int MPU6050_ReadData(void);
void MPU6050_Gyro_calibration(void);

#endif
