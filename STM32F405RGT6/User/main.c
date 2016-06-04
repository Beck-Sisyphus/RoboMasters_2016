#include "main.h"

/// Turns on to print the reading
#define DEBUG true

unsigned char USART_BUF[24] = {0};
extern arduino_data data_usart_3;

extern uint16_t measured_yaw_angle;
extern uint16_t measured_yaw_current;
extern int16_t target_yaw_current;
extern int16_t measured_pitch_angle;
extern int16_t measured_pitch_current;
extern int16_t target_pitch_current;

extern int16_t motor_front_right_cur;
extern int16_t motor_front_left_cur;
extern int16_t motor_back_left_cur;
extern int16_t motor_back_right_cur;

extern int32_t measured_yaw_angle_401;

extern MPU6050_RAW_DATA MPU6050_Raw_Data;
extern MPU6050_REAL_DATA MPU6050_Real_Data;
extern int16_t pitch_Position;
extern int16_t yaw_Position;

extern volatile int16_t measured_201_angle;
extern volatile int16_t measured_201_speed;



extern volatile int16_t measured_202_angle;
extern volatile int16_t measured_202_speed;


extern volatile int16_t measured_203_angle;
extern volatile int16_t measured_203_speed;


extern volatile int16_t measured_204_angle;
extern volatile int16_t measured_204_speed;


int count = 0;


int main(void)
{
    int i = 0;
    pitch_Position = 1571; // 90 degree in radian
    yaw_Position = 0;
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
    delay_ms(1000);
    // Set the duty cycle to 1000 to initialize the motor controller for friction wheels
    PWM1 = 1500;
    PWM2 = 1500;

		#if DEBUG
				printf("Front right; Front left; Back left; Back right");
		#endif
    count = 0;

    Motor_Reset_Can_2();

    while(1)
    {
      //   // CurrentProtect();
        MPU6050_ReadData();
			// Remote_Control();
			//  // wheel_control(2000 , 0, 0);
      //  #if DEBUG
      //       pitchyaw_control(0, -1500);
      //       count = count + 20;
      //      // printf("%i, %i", measured_yaw_angle, measured_pitch_angle);
      //      // printf("%i, %i, %i, %i", motor_front_right_cur, motor_front_left_cur, motor_back_left_cur, motor_back_right_cur);
      //      printf("%i, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f", count, MPU6050_Real_Data.Gyro_X,
      //      MPU6050_Real_Data.Gyro_Y, MPU6050_Real_Data.Gyro_Z,
      //      MPU6050_Real_Data.Accel_X, MPU6050_Real_Data.Accel_Y, MPU6050_Real_Data.Accel_Z);
      //      //printf("%i", measured_yaw_angle_401);
      //      delay_ms(1000);
      //  #endif
      // printf("%i\t\t%i\t\t%i\t\t%i\t\t", measured_201_speed, measured_202_speed, measured_203_speed, measured_204_speed);
        //printf("HI");
        //delay_ms(1000);
			  //float pitch_velocity_change = Velocity_Control_205((float)MPU6050_Real_Data.Gyro_Y, 0);
        //pitchyaw_control(0, (int16_t)pitch_velocity_change);
        set_Pitch_Yaw_Position(pitch_Position, yaw_Position);
        delay_ms(1);
    }
}
