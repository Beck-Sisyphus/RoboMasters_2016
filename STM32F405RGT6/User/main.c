#include "main.h"
#include "mpu6050_driver.h"

<<<<<<< HEAD
///Turns on Beck's trying for PID controller
#define PID false

// Improvements direction: Acceleration filter, increasing the proportion of the acceleration correction in quaternion
// Remote control command smoothing

char id[3];
unsigned char USART_BUF[24] = {0};
// extern RC_Ctl_t RC_Ctl_usart_3;
extern RC_Ctl_t RC_Ctl;
extern arduino_data data_usart_3;

extern uint16_t temp_yaw_angle;
extern uint16_t measured_pitch_angle;

extern uint16_t temp_yaw_current;
extern uint16_t temp_pitch_current;

extern uint16_t measure_pitch_current;

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
    // Set the duty cycle to 1000 to initialize the motor controller for friction wheels
    PWM1 = 1000;
    PWM2 = 1000;
    delay_ms(1000);

    // initialize the motor controller for bullets sending motor
    // Motor_Reset(MOTOR_NUM1);
    // delay_ms(30);
    // Motor_Init(MOTOR_NUM1,PWM_MODE);
    // for pitch motor on the red sample rover, the threshold to go up is 128
    // 127 goes down, 128 goes up
    printf("Count; Pitch angle; Measured current; Target current");
    // delay_ms(30);

    // LED1_OFF();
    // LED2_OFF();
    Motor_Reset_Can_2();
    // Motor_ManSet_Can_2();

    while(1)
    {
        CurrentProtect(); // Overcurrent protection for motor controller
        // Motor_Current_Send(2, -1000);
        // Motor_ManSet_Can_2();
        // Motor_Current_Send(2, -1000);

        /************ For Red C Motor Switch
        if (RC_Ctl.rc.s2 == RC_SW_MID && RC_Ctl.rc.s1 == RC_SW_MID) {
            // LED1_OFF();
            // LED2_OFF();
            Motor_Reset_Can_2();
        } else if(RC_Ctl.rc.s2 == RC_SW_UP) {
            // LED1_ON();
            // Forward
            Motor_Current_Send(3, -10);
            Motor_Current_Send(4, -10);
            Motor_Current_Send(5, 10);
            Motor_Current_Send(6, 10);
        } else if (RC_Ctl.rc.s2 == RC_SW_DOWN) {
            // LED1_ON();
            //Backward
            Motor_Current_Send(3, 10);
            Motor_Current_Send(4, 10);
            Motor_Current_Send(5, -10);
            Motor_Current_Send(6, -10);
        } else if (RC_Ctl.rc.s1 == RC_SW_UP) {
            // LED2_ON();
            //Right
            Motor_Current_Send(3, -10);
            Motor_Current_Send(4, -10);
            Motor_Current_Send(5, -10);
            Motor_Current_Send(6, -10);
        } else if (RC_Ctl.rc.s1 == RC_SW_DOWN) {
            // LED2_ON();
            //Left
            Motor_Current_Send(3, 10);
            Motor_Current_Send(4, 10);
            Motor_Current_Send(5, 10);
            Motor_Current_Send(6, 10);
        }
        ************/
    // Read Stick values
    // printf("ch0:%i ch1:%i ch2:%i ch3:%i", RC_Ctl.rc.ch0, RC_Ctl.rc.ch1, RC_Ctl.rc.ch2, RC_Ctl.rc.ch3);
    // delay_ms(1000);

        // For Blue Motor Stick
        if (RC_Ctl.rc.ch0 == RC_CH_VALUE_OFFSET
            && RC_Ctl.rc.ch1 == RC_CH_VALUE_OFFSET
            && RC_Ctl.rc.ch2 == RC_CH_VALUE_OFFSET
            && RC_Ctl.rc.ch3 == RC_CH_VALUE_OFFSET) {
            // LED1_OFF();
            // LED2_OFF();
            Motor_Reset_Can_2();
        } else if(RC_Ctl.rc.ch3 > 1600 || RC_Ctl.rc.ch1 > 1600) {
            // LED1_ON();
            // Forward
            Motor_Current_Send(3, -10);
            Motor_Current_Send(4, 10);
            Motor_Current_Send(5, 10);
            Motor_Current_Send(6, -10);
        } else if (RC_Ctl.rc.ch3 < 400 || RC_Ctl.rc.ch1 < 400) {
            // LED1_ON();
            //Backward
            Motor_Current_Send(3, 10);
            Motor_Current_Send(4, -10);
            Motor_Current_Send(5, -10);
            Motor_Current_Send(6, 10);
        } else if (RC_Ctl.rc.ch2 > 1600 || RC_Ctl.rc.ch0 > 1600) {
            // LED2_ON();
            //Right
            Motor_Current_Send(3, -10);
            Motor_Current_Send(4, -10);
            Motor_Current_Send(5, -10);
            Motor_Current_Send(6, -10);
        } else if (RC_Ctl.rc.ch2 < 400 || RC_Ctl.rc.ch0 < 400) {
            // LED2_ON();
            //Left
            Motor_Current_Send(3, 10);
            Motor_Current_Send(4, 10);
            Motor_Current_Send(5, 10);
            Motor_Current_Send(6, 10);
        }

/*
        // For Blue Motor Switch
        if (RC_Ctl.rc.s2 == RC_SW_MID && RC_Ctl.rc.s1 == RC_SW_MID) {
            // LED1_OFF();
            // LED2_OFF();
            Motor_Reset_Can_2();
        } else if(RC_Ctl.rc.s2 == RC_SW_UP) {
            // LED1_ON();
            // Forward
            Motor_Current_Send(3, -10);
            Motor_Current_Send(4, 10);
            Motor_Current_Send(5, 10);
            Motor_Current_Send(6, -10);
        } else if (RC_Ctl.rc.s2 == RC_SW_DOWN) {
            // LED1_ON();
            //Backward
            Motor_Current_Send(3, 10);
            Motor_Current_Send(4, -10);
            Motor_Current_Send(5, -10);
            Motor_Current_Send(6, 10);
        } else if (RC_Ctl.rc.s1 == RC_SW_UP) {
            // LED2_ON();
            //Right
            Motor_Current_Send(3, -10);
            Motor_Current_Send(4, -10);
            Motor_Current_Send(5, -10);
            Motor_Current_Send(6, -10);
        } else if (RC_Ctl.rc.s1 == RC_SW_DOWN) {
            // LED2_ON();
            //Left
            Motor_Current_Send(3, 10);
            Motor_Current_Send(4, 10);
            Motor_Current_Send(5, 10);
            Motor_Current_Send(6, 10);
        }

*/

        Motor_Current_Send(2, count);
        printf("%i, %i, %i, %i", count, measured_pitch_angle, measure_pitch_current, temp_pitch_current);
        count--;
        delay_ms(1000);

        /* Send to Arduino */
        // printf("Pitch angle: %i", measured_pitch_angle);

        // printf("Pitch angle: %i", temp_pitch_angle);
        // delay_ms(1000);
        // printf("Pitch current: %i", temp_pitch_current);
        // delay_ms(1000);
        // printf("Yaw angle: %i", temp_yaw_angle);
        // delay_ms(1000);
        // printf("Yaw current: %i", temp_yaw_current);
        // delay_ms(1000);
        #if PID
            set_Pitch_Position(4000);
            printf("Pitch current measured?: %i", measure_pitch_current);
            delay_ms(1000);
        #endif
    }
}
