#include "main.h"

ACCEL_AVERAGE_DATA   Accel_Raw_Average_Data; 
GYRO_RADIAN_DATA     Gyro_Radian_Data;
MPU6050_ANGLE        MPU6050_Angle;

void MPU6050_Data_Filter(void)
{
    unsigned int i=0;
    static unsigned int first_flag = 0;
    static unsigned int filter_cnt = 0;    // Counter for filters
    
    long temp_accel_x = 0; // Buffer for the sum of the original accelerometer X axis data
    long temp_accel_y = 0; // Buffer for the sum of the original accelerometer Y axis data
    long temp_accel_z = 0; // Buffer for the sum of the original accelerometer Z axis data
    
    static short accel_x_buffer[10] = {0}; // Buffer for 10 recent original accelerometer X axis data
    static short accel_y_buffer[10] = {0}; // Buffer for 10 recent original accelerometer Y axis data
    static short accel_z_buffer[10] = {0}; // Buffer for 10 recent original accelerometer Z axis data
    
    if(first_flag == 0) // If using this function for the first time, initialize buffers
    {
        first_flag = 1; // Run this part only once
        for(i=0;i<10;i++)
        {
            accel_x_buffer[i] = MPU6050_Raw_Data.Accel_X;
            accel_y_buffer[i] = MPU6050_Raw_Data.Accel_Y;
            accel_z_buffer[i] = MPU6050_Raw_Data.Accel_Z;
        }
    }
    else  // After 10 time reading
    {
        accel_x_buffer[filter_cnt] = MPU6050_Raw_Data.Accel_X;
        accel_y_buffer[filter_cnt] = MPU6050_Raw_Data.Accel_Y;
        accel_z_buffer[filter_cnt] = MPU6050_Raw_Data.Accel_Z;   
        
        filter_cnt ++;
        if(filter_cnt == 10)
        {
            filter_cnt = 0;
        }        
    }
    
    for(i=0;i<10;i++)
    {
        temp_accel_x += accel_x_buffer[i];
        temp_accel_y += accel_y_buffer[i];
        temp_accel_z += accel_z_buffer[i];
    }
    
    Accel_Raw_Average_Data.X = (float)temp_accel_x / 10.0;
    Accel_Raw_Average_Data.Y = (float)temp_accel_y / 10.0;
    Accel_Raw_Average_Data.Z = (float)temp_accel_z / 10.0;
    
    Gyro_Radian_Data.X = (float)(MPU6050_Real_Data.Gyro_X  * (3.14159265/180.0));
    Gyro_Radian_Data.Y = (float)(MPU6050_Real_Data.Gyro_Y  * (3.14159265/180.0));
    Gyro_Radian_Data.Z = (float)(MPU6050_Real_Data.Gyro_Z  * (3.14159265/180.0));
}

void MPU6050_Angle_Calculate( float gyro_x,
                              float gyro_y,
                              float gyro_z,
                              float accel_x,
                              float accel_y,
                              float accel_z)
{
    static float q0 = 1;
    static float q1 = 0;
    static float q2 = 0;
    static float q3 = 0;
    
    static float exInt = 0;
    static float eyInt = 0;
    static float ezInt = 0;
    
    const float kp = 0.3; //
    const float ki = 0.00; //0.0;
    const float halfT = 0.001; // calculate the half period
    
    float norm; // norm for the acceleration vector
    float vx,vy,vz;
    float ex,ey,ez;

    float ax,ay,az; // acceleration vector over norm
    float gx,gy,gz; // gyro vector

    static float pre_ax = 0;
    static float pre_ay = 0;
    static float pre_az = 0;
    // acceleration filter
    accel_x = accel_x *0.02 + pre_ax * 0.98;
    pre_ax = accel_x;
    
    accel_y = accel_y *0.02 + pre_ay * 0.98;
    pre_ay = accel_y;

    accel_z = accel_z *0.02 + pre_az * 0.98;
    pre_az = accel_z;    
    
    norm = sqrt(accel_x * accel_x + accel_y * accel_y + accel_z * accel_z);
    ax = accel_x / norm;
    ay = accel_y / norm;
    az = accel_z / norm;
    
    vx = 2 * (q1*q3 - q0*q2);
    vy = 2 * (q0*q1 + q2*q3);
    vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
    
    ex = (ay*vz - az*vy);
    ey = (az*vx - ax*vz);
    ez = (ax*vy - ay*vx);
    
    exInt += ki*ex;
    eyInt += ki*ey;
    ezInt += ki*ez;
    
    gx = gyro_x + kp*ex + exInt;
    gy = gyro_y + kp*ey + eyInt;
    gz = gyro_z + kp*ez + ezInt;
    
    q0 += (      - q1*gx - q2*gy - q3*gz)*halfT;
    q1 += (q0*gx +         q2*gz - q3*gy)*halfT;
    q2 += (q0*gy - q1*gz +         q3*gx)*halfT;
    q3 += (q0*gz + q1*gy - q2*gx        )*halfT;

    norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 = q0 / norm;
    q1 = q1 / norm;
    q2 = q2 / norm;
    q3 = q3 / norm;
    
    MPU6050_Angle.Rool = asin(-2 * q1 * q3 + 2 * q0* q2) * (180.0/3.14159265); 
    MPU6050_Angle.Pitch  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1) * (180.0/3.14159265); 
    MPU6050_Angle.Yaw = atan2( 2 * q1 * q2 + 2 * q0 * q3,1.0 - 2.0 * ( q2 * q2 + q3 * q2 ) ) * (180.0/3.14159265);//²»×¼

}


