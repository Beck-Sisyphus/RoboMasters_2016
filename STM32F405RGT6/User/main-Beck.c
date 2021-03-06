#include "main.h"

/// Turns on to print the reading
#define DEBUG false

unsigned char USART_BUF[24] = {0};
extern arduino_data data_usart_3;

extern int16_t motor_front_right_cur;
extern int16_t motor_front_left_cur;
extern int16_t motor_back_left_cur;
extern int16_t motor_back_right_cur;

extern int32_t measured_yaw_angle_401;

extern MPU6050_RAW_DATA MPU6050_Raw_Data;
extern MPU6050_REAL_DATA MPU6050_Real_Data;
extern int16_t pitch_Position;
extern int16_t yaw_Position;


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

		#if DEBUG
				printf("Front right; Front left; Back left; Back right");
		#endif

    Motor_Reset_Can_2();

    while(1)
    {
        // CurrentProtect();
        MPU6050_ReadData();
			  Remote_Control();

       #if DEBUG
           // printf("%i, %i", measured_yaw_angle, measured_pitch_angle);
           // printf("%i, %i, %i, %i", motor_front_right_cur, motor_front_left_cur, motor_back_left_cur, motor_back_right_cur);
           printf("%i, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f", count, MPU6050_Real_Data.Gyro_X,
           MPU6050_Real_Data.Gyro_Y, MPU6050_Real_Data.Gyro_Z,
           MPU6050_Real_Data.Accel_X, MPU6050_Real_Data.Accel_Y, MPU6050_Real_Data.Accel_Z);
           //printf("%i", measured_yaw_angle_401);
           delay_ms(1000);
       #endif
    }
}
