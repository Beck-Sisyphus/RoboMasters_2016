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
extern uint16_t measured_pitch_angle;

extern int16_t target_yaw_current;
extern int16_t target_pitch_current;
extern int16_t measured_yaw_current;
extern int16_t measured_pitch_current;
// to prevent whels from turning on start-up if remote is off
uint8_t Remote_On = 0;

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


        // To see if remote is off or not
        if (RC_Ctl.rc.ch2 < RC_CH_VALUE_MIN
            || RC_Ctl.rc.ch3 < RC_CH_VALUE_MIN
            ) {
            Remote_On = 0;
        } else {
            Remote_On = 1;
        }


        // For Blue Motor Stick
        if ((RC_Ctl.rc.ch2 == RC_CH_VALUE_OFFSET
            && RC_Ctl.rc.ch3 == RC_CH_VALUE_OFFSET)
            || Remote_On == 0) {
            // LED1_OFF();
            // LED2_OFF();
            Motor_Reset_Can_2();
        } else if(RC_Ctl.rc.ch3 > RC_CH_VALUE_OFFSET) {
            // LED1_ON();
            // Forward
            Motor_Current_Send(3, -1 * Pos_Curr_Eqn(RC_Ctl.rc.ch3));
            Motor_Current_Send(4, Pos_Curr_Eqn(RC_Ctl.rc.ch3));
            Motor_Current_Send(5, Pos_Curr_Eqn(RC_Ctl.rc.ch3));
            Motor_Current_Send(6, -1 * Pos_Curr_Eqn(RC_Ctl.rc.ch3));
        } else if (RC_Ctl.rc.ch3 < RC_CH_VALUE_OFFSET) {
            // LED1_ON();
            //Backward
            Motor_Current_Send(3, -1 * Neg_Curr_Eqn(RC_Ctl.rc.ch3));
            Motor_Current_Send(4, Neg_Curr_Eqn(RC_Ctl.rc.ch3));
            Motor_Current_Send(5, Neg_Curr_Eqn(RC_Ctl.rc.ch3));
            Motor_Current_Send(6, -1 * Neg_Curr_Eqn(RC_Ctl.rc.ch3));
        } else if (RC_Ctl.rc.ch2 > RC_CH_VALUE_OFFSET) {
            // LED2_ON();
            //Right
            Motor_Current_Send(3, -1 * Pos_Curr_Eqn(RC_Ctl.rc.ch2));
            Motor_Current_Send(4, -1 * Pos_Curr_Eqn(RC_Ctl.rc.ch2));
            Motor_Current_Send(5, -1 * Pos_Curr_Eqn(RC_Ctl.rc.ch2));
            Motor_Current_Send(6, -1 * Pos_Curr_Eqn(RC_Ctl.rc.ch2));
        } else if (RC_Ctl.rc.ch2 < RC_CH_VALUE_OFFSET) {
            // LED2_ON();
            //Left
            Motor_Current_Send(3, -1 * Neg_Curr_Eqn(RC_Ctl.rc.ch2));
            Motor_Current_Send(4, -1 * Neg_Curr_Eqn(RC_Ctl.rc.ch2));
            Motor_Current_Send(5, -1 * Neg_Curr_Eqn(RC_Ctl.rc.ch2));
            Motor_Current_Send(6, -1 * Neg_Curr_Eqn(RC_Ctl.rc.ch2));
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




        /* Send to Arduino */
        // printf("Pitch angle: %i", measured_pitch_angle);

        // printf("Pitch angle: %i", temp_pitch_angle);
        // delay_ms(1000);
        // printf("Pitch current: %i", target_pitch_current);
        // delay_ms(1000);
        // printf("Yaw angle: %i", measured_yaw_angle);
        // delay_ms(1000);
        // printf("Yaw current: %i", target_yaw_current);
        // delay_ms(1000);
        #if PID
            set_Pitch_Position(4000);
        #else
            Motor_Current_Send(2, count);
            count = count + 50;
        #endif

        printf("%i, %i, %hd, %hd", count, measured_pitch_angle, measured_pitch_current, target_pitch_current);
        delay_ms(1000);
    }
}
