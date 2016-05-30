#include "main.h"

uint32_t Upload_Speed = 1;   // Data upload speed, unit: Hz
#define upload_time (1000000/Upload_Speed)  //Calculating the upload speed, unit: us

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
    system_micrsecond = Get_Time_Micros();
    while(1)
    {
      if((Get_Time_Micros() - system_micrsecond) > upload_time)
  		{
  			system_micrsecond = Get_Time_Micros();
  			UploadParameter();   //upload data to the PC
  			delay_ms(1);
  		}
    }
}
