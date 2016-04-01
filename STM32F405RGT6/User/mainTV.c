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

extern int16_t x101;
extern int16_t x123;
extern int16_t x145;
extern int16_t x167;

extern int16_t x201;
extern int16_t x223;
extern int16_t x245;
extern int16_t x267;

extern int16_t x301;
extern int16_t x323;
extern int16_t x345;
extern int16_t x367;

extern int16_t x401;
extern int16_t x423;
extern int16_t x445;
extern int16_t x467;

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

        /////////// For testing
        printf("201 %i %i %i %i \n", x101, x123, x145, x167);
        printf("202 %i %i %i %i \n", x201, x223, x245, x267);
        printf("203 %i %i %i %i \n", x301, x323, x345, x367);
        printf("204 %i %i %i %i \n", x401, x423, x445, x467);
        delay_ms(1000);
        ///////////
    }
}


// For testing
// extern uint16_t x1data0;
// extern uint16_t x1data1;
// extern uint16_t x1data2;
// extern uint16_t x1data3;
// extern uint16_t x1data4;
// extern uint16_t x1data5;
// extern uint16_t x1data6;
// extern uint16_t x1data7;

// extern uint16_t x2data0;
// extern uint16_t x2data1;
// extern uint16_t x2data2;
// extern uint16_t x2data3;
// extern uint16_t x2data4;
// extern uint16_t x2data5;
// extern uint16_t x2data6;
// extern uint16_t x2data7;

// extern uint16_t x3data0;
// extern uint16_t x3data1;
// extern uint16_t x3data2;
// extern uint16_t x3data3;
// extern uint16_t x3data4;
// extern uint16_t x3data5;
// extern uint16_t x3data6;
// extern uint16_t x3data7;

// extern uint16_t x4data0;
// extern uint16_t x4data1;
// extern uint16_t x4data2;
// extern uint16_t x4data3;
// extern uint16_t x4data4;
// extern uint16_t x4data5;
// extern uint16_t x4data6;
// extern uint16_t x4data7;
