#include "main.h"

///Turns on Beck's trying for PID controller
#define PID true

unsigned char USART_BUF[24] = {0};
extern arduino_data data_usart_3;

extern uint16_t measured_yaw_angle;
extern uint16_t measured_yaw_current;
extern int16_t target_yaw_current;
extern int16_t measured_pitch_angle;
extern int16_t measured_pitch_current;
extern int16_t target_pitch_current;

extern int16_t x145;
extern int16_t x245;
extern int16_t x345;
extern int16_t x445;

extern int16_t x123;
extern int16_t x223;
extern int16_t x323;
extern int16_t x423;

extern int32_t measured_yaw_angle_401;

extern MPU6050_RAW_DATA MPU6050_Raw_Data;
extern MPU6050_REAL_DATA MPU6050_Real_Data;

extern int pitch_Position;
extern int yaw_Position;

int count = 100;


int main(void)
{
    int i = 0;
    BSP_Init();
    //delay 500ms, wait MPU6050 for stable signal
    delay_ms(500);
    while(MPU6050_Initialization() == 0xff)
    {
        i++;     // try again if initialization failed
    }
    MPU6050_Gyro_calibration();

    // MPU6050_Interrupt_Configuration();


    PWM_Configuration();
    delay_ms(1000);
    // Set the duty cycle to 1000 to initialize the motor controller for friction wheels
    PWM1 = 1500;
    PWM2 = 1500;

		#if !PID
				printf("Front right; Front left; Back left; Back right");
		#endif
    count = 0;

    Motor_Reset_Can_2();

    while(1)
    {
        // CurrentProtect();
			  Remote_Control();
        MPU6050_ReadData();
        /* Send to Arduino */
       #if PID
            pitch_Position = 90;
            yaw_Position = 0;
       #else
            //Motor_Current_Send(1, -1000);
					  Motor_Current_Send(2, count);
            Motor_Current_Send(1, count);
            count = count + 20;
           // printf("%i, %i, %hd, %hd, %i, %hd, %hd", count, _angle, _current, target_yaw_current, measured_pitch_angle, measured_pitch_current, target_pitch_current);
           // printf("%i, %i, %i, %i", x123, x223, x323, x423);
           printf("%i, %i", measured_yaw_angle, measured_pitch_angle);
          //  printf("%i, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f", count, MPU6050_Real_Data.Gyro_X,
          //  MPU6050_Real_Data.Gyro_Y, MPU6050_Real_Data.Gyro_Z,
          //  MPU6050_Real_Data.Accel_X, MPU6050_Real_Data.Accel_Y, MPU6050_Real_Data.Accel_Z);
           //printf("%i", measured_yaw_angle_401);
           delay_ms(1000);
       #endif
    }
}
