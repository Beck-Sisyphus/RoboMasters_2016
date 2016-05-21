#include "main.h"

// #define DEBUG false

unsigned char USART_BUF[24] = {0};
volatile extern arduino_data data_usart_3;

extern int16_t motor_front_right_cur;
extern int16_t motor_front_left_cur;
extern int16_t motor_back_left_cur;
extern int16_t motor_back_right_cur;

extern int32_t measured_yaw_angle_401;

extern MPU6050_RAW_DATA MPU6050_Raw_Data;
extern MPU6050_REAL_DATA MPU6050_Real_Data;
extern int16_t pitch_Position;
extern int16_t yaw_Position;

extern float mpu6050_angle_x;
extern float mpu6050_angle_y;
extern float mpu6050_angle_z;


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

    PWM_Configuration();
    delay_ms(1000);
    // Set the duty cycle to 1000 to initialize the motor controller for friction wheels
    PWM1 = 1500;
    PWM2 = 1500;
    Motor_Reset_Can_2();

    while(1)
    {
    }
}
