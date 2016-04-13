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

extern MPU6050_RAW_DATA MPU6050_Raw_Data;
extern MPU6050_REAL_DATA MPU6050_Real_Data;

int pitch_Position;
int yaw_Position;

char greeting[6] = {'H', 'E', 'L', 'L', 'O', '\0'};
char ay = 'a';

int count = 0;


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
    // printf("Count; Yaw angle; Measured current; Target current; Pitch angle; Measured current; Target current");
    printf("Front right; Front left; Back left; Back right");
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
            Motor_Current_Send(1, -1000);
					//  Motor_Current_Send(2, -count);
           // count = count + 20;
           printf("%i, %i, %hd, %hd, %i, %hd, %hd", count, measured_yaw_angle, measured_yaw_current, target_yaw_current, measured_pitch_angle, measured_pitch_current, target_pitch_current);
           // printf("%i, %i, %i, %i", x123, x223, x323, x423);

          //  printf("%i, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f", count, MPU6050_Real_Data.Gyro_X,
          //  MPU6050_Real_Data.Gyro_Y, MPU6050_Real_Data.Gyro_Z,
          //  MPU6050_Real_Data.Accel_X, MPU6050_Real_Data.Accel_Y, MPU6050_Real_Data.Accel_Z);
           delay_ms(1000);
       #endif
    }
}
