#include "main.h"
// code credit of complementary filter to Pieter-Jan Van de Maele
// Define the registers address for MPU6050
#define	SMPLRT_DIV		0x19	// Gyroscope sampling rate  default 0X07 125Hz
#define	CONFIG			0x1A	// Low-pass filter rate     default 0x00 
#define	GYRO_CONFIG		0x1B	// Gyroscope self check and measurement range   default 0x18 Non self-check 2000deg/s
#define	ACCEL_CONFIG	0x1C	// Accelerometer self-check and measurement range default 0x01 Non self-check 2G 5Hz

#define INT_PIN_CFG     0x37
#define INT_ENABLE      0x38
#define INT_STATUS      0x3A    // Read-only


#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C

#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E

#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40

#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42

#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44	

#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46

#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48

#define	PWR_MGMT_1		0x6B	// Power management default 0x00 regular usage
#define	WHO_AM_I		0x75	// read-only  default read-out should be MPU6050_ID = 0x68


#define MPU6050_ID              0x68
#define MPU6050_DEVICE_ADDRESS  0xD0
#define MPU6050_DATA_START      ACCEL_XOUT_H   // Since all data addresses are continuous, read them all 
#define M_PI 3.14159265358979323846 
#define Alpha	0.98
#define TimeSlice	0.01
MPU6050_RAW_DATA    MPU6050_Raw_Data; 
MPU6050_REAL_DATA   MPU6050_Real_Data;
const float RADIAN_TO_ANGLE = 180/M_PI;
int gyroADC_X_offset=0,gyroADC_Y_offset=0,gyroADC_Z_offset=0;
unsigned char offset = 136;
float gyro_angle_x = 0;
float gyro_angle_y = 0;
float gyro_angle_z = 0;

float gyro_rate_x = 0;
//MPU6050 Initialzaiton, return 0 if success, otherwise return 0xff
int MPU6050_Initialization(void)
{
    unsigned char temp_data = 0x00;

    IIC_GPIO_Init();  //Initialize I2C port
    // new code
    //HEAT_Configuration();
    
    if(IIC_ReadData(MPU6050_DEVICE_ADDRESS,WHO_AM_I ,&temp_data ,1)==0) // Check if MPU6050 is connected to I2C bus
    {
        if(temp_data != MPU6050_ID)
        {
            printf("error 1A\r\n");
            return 0xff; // Check fail, return 0xff
        }
    }
    else
    {
        printf("error 1B\r\n");
        return 0xff; // read fail, return 0xff
    }
    
    if(IIC_WriteData(MPU6050_DEVICE_ADDRESS,PWR_MGMT_1,0x01) == 0xff)    // wake up from sleep mode
    {
        printf("error 1C\r\n");
        return 0xff;
    }
    if(IIC_WriteData(MPU6050_DEVICE_ADDRESS,SMPLRT_DIV,0x07) == 0xff)// cyq: 07 update frequency to 1kHz
    {
        printf("error 1D\r\n");
        return 0xff;
    }
    if(IIC_WriteData(MPU6050_DEVICE_ADDRESS,CONFIG,0x00) == 0xff)
    {
        printf("error 1E\r\n");
        return 0xff;
    }
    if(IIC_WriteData(MPU6050_DEVICE_ADDRESS,GYRO_CONFIG,0x08) == 0xff)
    {
        printf("error 1F\r\n");
        return 0xff;
    }
    if(IIC_WriteData(MPU6050_DEVICE_ADDRESS,ACCEL_CONFIG,0x08) == 0xff)
    {
        printf("error 1G\r\n");
        return 0xff;
    }
    if(IIC_WriteData(MPU6050_DEVICE_ADDRESS,INT_PIN_CFG,0x00) == 0xff)
    {
        printf("error 1H\r\n");
        return 0xff;
    }
    if(IIC_WriteData(MPU6050_DEVICE_ADDRESS,INT_ENABLE,0x01) == 0xff)
    {
        printf("error 1I\r\n");
        return 0xff;
    }
    
    //MPU6050_Interrupt_Configuration(); //MPU6050 interrupt initialization
    
    return 0;
}

//MPU6050  read data out, return 0 if success, return 0xff if fail
int MPU6050_ReadData(void)
{
    u8 buf[14];
    
    if(IIC_ReadData(MPU6050_DEVICE_ADDRESS,MPU6050_DATA_START,buf,14) == 0xff)
    {
        printf("error 1J\r\n");
        return 0xff;
    }
    else
    {
        // Read registers original data
           
        MPU6050_Raw_Data.Accel_X = (buf[0]<<8 | buf[1]);
        MPU6050_Raw_Data.Accel_Y = (buf[2]<<8 | buf[3]);
        MPU6050_Raw_Data.Accel_Z = (buf[4]<<8 | buf[5]); 
        MPU6050_Raw_Data.Temp =    (buf[6]<<8 | buf[7]);  
        MPU6050_Raw_Data.Gyro_X = (buf[8]<<8 | buf[9]);
        MPU6050_Raw_Data.Gyro_Y = (buf[10]<<8 | buf[11]);
        MPU6050_Raw_Data.Gyro_Z = (buf[12]<<8 | buf[13]);
				
       
        // Convert origianl data to actual acceleration and gyro data, calculation formula related to the register settings
        MPU6050_Real_Data.Accel_X = -(float)(MPU6050_Raw_Data.Accel_X)/8192.0; // read datasheet 30 of 47
        MPU6050_Real_Data.Accel_Y = -(float)(MPU6050_Raw_Data.Accel_Y)/8192.0; // read datasheet 30 of 47
        MPU6050_Real_Data.Accel_Z = (float)(MPU6050_Raw_Data.Accel_Z)/8192.0;  // read datasheet 30 of 47
        MPU6050_Real_Data.Temp =   (float)(MPU6050_Raw_Data.Temp)/340.0+36.53; // read datasheet 31 of 47
        MPU6050_Real_Data.Gyro_X = -(float)(MPU6050_Raw_Data.Gyro_X - gyroADC_X_offset)/65.5;     // read datasheet 32 of 47
        MPU6050_Real_Data.Gyro_Y = -(float)(MPU6050_Raw_Data.Gyro_Y - gyroADC_Y_offset)/65.5;     // read datasheet 32 of 47
        MPU6050_Real_Data.Gyro_Z = (float)(MPU6050_Raw_Data.Gyro_Z - gyroADC_Z_offset)/65.5;      // read datasheet 32 of 47
				
				float mpu6050_angle_x = RADIAN_TO_ANGLE*atan(MPU6050_Real_Data.Accel_X/sqrt(MPU6050_Real_Data.Accel_Y*MPU6050_Real_Data.Accel_Y+MPU6050_Real_Data.Accel_Z*MPU6050_Real_Data.Accel_Z));
				float mpu6050_angle_y = RADIAN_TO_ANGLE*atan(MPU6050_Real_Data.Accel_Y/sqrt(MPU6050_Real_Data.Accel_X*MPU6050_Real_Data.Accel_X+MPU6050_Real_Data.Accel_Z*MPU6050_Real_Data.Accel_Z));
				float mpu6050_angle_z = RADIAN_TO_ANGLE*atan(sqrt(MPU6050_Real_Data.Accel_X*MPU6050_Real_Data.Accel_X+MPU6050_Real_Data.Accel_Y*MPU6050_Real_Data.Accel_Y)/MPU6050_Real_Data.Accel_Z);
				// Complementary Filter 
				float angle_x_filtered = Alpha*(mpu6050_angle_x + MPU6050_Real_Data.Gyro_X * TimeSlice) + (1-Alpha)*MPU6050_Real_Data.Accel_X;
				float angle_y_filtered = Alpha*(mpu6050_angle_y + MPU6050_Real_Data.Gyro_Y * TimeSlice) + (1-Alpha)*MPU6050_Real_Data.Accel_Y;
				float angle_z_filtered = Alpha*(mpu6050_angle_z + MPU6050_Real_Data.Gyro_Z * TimeSlice) + (1-Alpha)*MPU6050_Real_Data.Accel_Z;
		} 
    
    return 0;
}


void MPU6050_Gyro_calibration(void)
{
	u16 i;
	float x_temp=0,y_temp=0,z_temp=0;
	
	for(i=0; i<1000; i++)
	{
		MPU6050_ReadData();
		delay_ms(1);
		x_temp=x_temp+MPU6050_Raw_Data.Gyro_X;
		y_temp=y_temp+MPU6050_Raw_Data.Gyro_Y;
		z_temp=z_temp+MPU6050_Raw_Data.Gyro_Z;
	}			
	
	x_temp=x_temp/1000.00;
	y_temp=y_temp/1000.00;	
	z_temp=z_temp/1000.00;

	gyroADC_X_offset=(int)x_temp;
	gyroADC_Y_offset=(int)y_temp;
	gyroADC_Z_offset=(int)z_temp;
	
}
