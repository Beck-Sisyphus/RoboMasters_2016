#include "main.h"

/// Turns on to print the reading
#define DEBUG false

unsigned char USART_BUF[24] = {0};
extern arduino_data data_usart_3;

extern uint16_t measured_yaw_angle;
extern int16_t measured_pitch_angle;

extern int16_t motor_yaw_cur;
extern int16_t motor_pitch_cur;
extern int16_t motor_front_right_cur;
extern int16_t motor_front_left_cur;
extern int16_t motor_back_left_cur;
extern int16_t motor_back_right_cur;

extern MPU6050_RAW_DATA MPU6050_Raw_Data;
extern MPU6050_REAL_DATA MPU6050_Real_Data;
extern int16_t pitch_Position;

extern int16_t yaw_Position;
extern float measured_yaw_angle_401;

extern int16_t drive;
extern int16_t strafe;
extern int16_t rotate;
extern volatile Encoder CM1Encoder;
extern volatile Encoder CM2Encoder;
extern volatile Encoder CM3Encoder;
extern volatile Encoder CM4Encoder;
extern volatile Encoder GMYawEncoder;
extern volatile Encoder GMPitchEncoder;

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
		// delay_ms(1000);
    PWM_Configuration();
    delay_ms(1000);
    // Set the duty cycle to 1000 to initialize the motor controller for friction wheels
    PWM1 = 1500;
    // delay_ms(1000);
    PWM2 = 1500;
    delay_ms(1000);
    PWM1 = 1550;
    // delay_ms(1000);
    PWM2 = 1550;
    Motor_Reset_Can_2();
		pitch_Position = 0;
    yaw_Position = 0;

    while(1)
    {
        // CurrentProtect();
        MPU6050_ReadData();
        //Remote_Control();
        #if DEBUG
            printf("%i\t\t%i\t\t%f\t\t%f\t\t\n", measured_yaw_angle, measured_pitch_angle, GMYawEncoder.ecd_angle, GMPitchEncoder.ecd_angle);
            // printf("%i\t\t%i\t\t\n", (int16_t) yaw_velocity_change_205, (int16_t) pitch_velocity_change_206);
						// printf("%i\t\t%i\t\t%f\t\t%f\t\t\n", CM1Encoder.velocity_raw, CM2Encoder.velocity_raw, (float)CM3Encoder.velocity_raw, (float)CM4Encoder.velocity_raw);
            // printf("%i, %i, %i, %i", motor_front_right_cur, motor_front_left_cur, motor_back_left_cur, motor_back_right_cur);
            //  printf("%i, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f", count, MPU6050_Real_Data.Gyro_X,
            //  MPU6050_Real_Data.Gyro_Y, MPU6050_Real_Data.Gyro_Z,
            //  MPU6050_Real_Data.Accel_X, MPU6050_Real_Data.Accel_Y, MPU6050_Real_Data.Accel_Z);
            printf("%f", measured_yaw_angle_401);
            //  printf("%i\t\t%i\t\t%i\t\t\n", drive, strafe, rotate);
            delay_ms(1000);
        #endif
    }
}
