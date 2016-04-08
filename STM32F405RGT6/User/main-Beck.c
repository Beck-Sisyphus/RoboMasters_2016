#include "main.h"
#include "mpu6050_driver.h"

///Turns on Beck's trying for PID controller
#define PID true

// Improvements direction: Acceleration filter, increasing the proportion of the acceleration correction in quaternion
// Remote control command smoothing

char id[3];
unsigned char USART_BUF[24] = {0};
// extern RC_Ctl_t RC_Ctl_usart_3;
extern RC_Ctl_t RC_Ctl;
extern arduino_data data_usart_3;

extern MPU6050_RAW_DATA MPU6050_Raw_Data;
extern MPU6050_REAL_DATA MPU6050_Real_Data;

char greeting[6] = {'H', 'E', 'L', 'L', 'O', '\0'};
char ay = 'a';

int count = 0;
int pitch_Position;
int yaw_Position;

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

    MPU6050_Interrupt_Configuration();


    PWM_Configuration();
    // Set the duty cycle to 1000 to initialize the motor controller for friction wheels
    PWM1 = 1500;
    PWM2 = 1500;
    delay_ms(1000);

    // initialize the motor controller for bullets sending motor
    // Motor_Reset(MOTOR_NUM1);
    // delay_ms(30);
    // Motor_Init(MOTOR_NUM1,PWM_MODE);
    // for pitch motor on the red sample rover, the threshold to go up is 128
    // 127 goes down, 128 goes up
    printf("Count; Yaw angle; Measured current; Target current; Pitch angle; Measured current; Target current");
    count = 0;

    // LED1_OFF();
    // LED2_OFF();
    Motor_Reset_Can_2();
    // Motor_ManSet_Can_2();

    while(1)
    {
        // CurrentProtect();
			  Remote_Control();
        /* Send to Arduino */
       #if PID
           pitch_Position = 90;
           yaw_Position = 0;
       #else
           Motor_Current_Send(1, count);
					 Motor_Current_Send(2, -count);
           count = count + 20;
           printf("%i, %i, %hd, %hd, %i, %hd, %hd", count, measured_yaw_angle, measured_yaw_current, target_yaw_current, measured_pitch_angle, measured_pitch_current, target_pitch_current);
           //printf("%i, %f, %f, %f, %f, %f, %f", count, MPU6050_Raw_Data.Gyro_X,
           //MPU6050_Raw_Data.Gyro_Y, MPU6050_Raw_Data.Gyro_Z,
           //MPU6050_Raw_Data.Accel_X, MPU6050_Raw_Data.Accel_Y, MPU6050_Raw_Data.Accel_Z);
           delay_ms(1000);
       #endif
    }
}
