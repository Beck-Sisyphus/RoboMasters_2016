#include "main.h"
#include "mpu6050_driver.h"

// Improvements direction: Acceleration filter, increasing the proportion of the acceleration correction in quaternion
// Remote control command smoothing

char id[3];
unsigned char USART_BUF[24] = {0};
// extern RC_Ctl_t RC_Ctl_usart_3;
extern arduino_data data_usart_3;

extern uint16_t temp_yaw_angle;
extern uint16_t temp_pitch_angle;

extern uint16_t temp_yaw_current;
extern uint16_t temp_pitch_current;

extern uint16_t x1data0;
extern uint16_t x1data1;
extern uint16_t x1data2;
extern uint16_t x1data3;
extern uint16_t x1data4;
extern uint16_t x1data5;
extern uint16_t x1data6;
extern uint16_t x1data7;

extern uint16_t x2data0;
extern uint16_t x2data1;
extern uint16_t x2data2;
extern uint16_t x2data3;
extern uint16_t x2data4;
extern uint16_t x2data5;
extern uint16_t x2data6;
extern uint16_t x2data7;

extern uint16_t x3data0;
extern uint16_t x3data1;
extern uint16_t x3data2;
extern uint16_t x3data3;
extern uint16_t x3data4;
extern uint16_t x3data5;
extern uint16_t x3data6;
extern uint16_t x3data7;

extern uint16_t x4data0;
extern uint16_t x4data1;
extern uint16_t x4data2;
extern uint16_t x4data3;
extern uint16_t x4data4;
extern uint16_t x4data5;
extern uint16_t x4data6;
extern uint16_t x4data7;


char greeting[6] = {'H', 'E', 'L', 'L', 'O', '\0'};
char ay = 'a';


int main(void)
{
    int i = 0;
    BSP_Init();    
    //delay 500ms, wait MPU6050 for stable signal  
    delay_ms(500);    
    while(MPU6050_Initialization() == 0xff) 
    {
        i++;     // try again if initialization failed          
        if(i>2) // buzzer starts if initialization fail for more than 10 times and give up
        {
            /*while(i <= 11) 
            {
                                i++;
                LED1_TOGGLE();
                delay_ms(50);
                                
                        
                
            }*/
        }  
    }    
    MPU6050_Gyro_calibration();
    
    MPU6050_Interrupt_Configuration(); 
    MPU6050_ReadData();

    PWM_Configuration();
    delay_ms(1000);
    // Set the duty cycle to 1000 to initialize the motor controller for friction wheels
    PWM1 = 1500;
    PWM2 = 1500;
    
    Motor_Reset_Can_2();
    // Motor_ManSet_Can_2();
    while(1)
    {
        CurrentProtect(); // Overcurrent protection for motor controller

        Remote_Control(); 

        printf("201 %i %i %i %i %i %i %i \n", x1data0, x1data1, x1data2, x1data3, x1data4, x1data5, x1data6, x1data7);
        printf("202 %i %i %i %i %i %i %i \n", x2data0, x2data1, x2data2, x2data3, x2data4, x2data5, x2data6, x2data7);
        printf("203 %i %i %i %i %i %i %i \n", x3data0, x3data1, x3data2, x3data3, x3data4, x3data5, x3data6, x3data7);
        printf("204 %i %i %i %i %i %i %i \n", x4data0, x4data1, x4data2, x4data3, x4data4, x4data5, x4data6, x4data7);

        delay_ms(1000);
    }
}
