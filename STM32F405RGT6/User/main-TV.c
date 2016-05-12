#include "main.h"
#include "mpu6050_driver.h"


// Improvements direction: Acceleration filter, increasing the proportion of the acceleration correction in quaternion
// Remote control command smoothing

char id[3];
unsigned char USART_BUF[24] = {0};
// extern RC_Ctl_t RC_Ctl_usart_3;
volatile extern arduino_data data_usart_3;
//MPU6050_RAW_DATA    MPU6050_Raw_Data;
extern MPU6050_REAL_DATA   MPU6050_Real_Data;



char greeting[6] = {'H', 'E', 'L', 'L', 'O', '\0'};
char ay = 'A';


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

    // delay_ms(1000);
    PWM1 = 1500;
    PWM2 = 1500;
    PWM3 = 1;

    // Motor_Reset_Can_2();
    wheel_control(0, 0, 0);

    // Motor_ManSet_Can_2();
    while(1)
    {
        // CurrentProtect(); // Overcurrent protection for motor controller
        MPU6050_ReadData();
        Remote_Control();
        // usart3_receive();
        // pitchyaw_control(data_usart_3.packet.yaw_req, data_usart_3.packet.pitch_req);


        //printf("Pitch: %i     Yaw: %i", measured_pitch_angle, measured_yaw_angle);

        /////////// For testing
        // MPU6050_ReadData();
        // printf("%.2f %.2f %.2f \n", MPU6050_Real_Data.Gyro_X, MPU6050_Real_Data.Gyro_Y, MPU6050_Real_Data.Gyro_Z);
        // printf("%i %i %i %i \n", 1000, 2000, 3000, 4000);
        // printf("201 %i %i %i %i \n", x101, x123, x145, x167);
        // printf("202 %i %i %i %i \n", x201, x223, x245, x267);
        // printf("203 %i %i %i %i \n", x301, x323, x345, x367);
        // printf("204 %i %i %i %i \n", x401, x423, x445, x467);
        ///////////
        // delay_ms(1000);
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
