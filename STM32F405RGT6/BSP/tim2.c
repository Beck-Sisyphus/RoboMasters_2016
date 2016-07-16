#include "main.h"

volatile extern int16_t pitch_Position;
volatile extern int16_t yaw_Position;
volatile extern int16_t drive;
volatile extern int16_t strafe;
volatile extern int16_t rotate;
// volatile extern int16_t pitch_Velocity;
// volatile extern int16_t yaw_Velocity;

// for velocity controlling pitch and yaw with remote
// volatile extern int16_t remote_pitch_change;
// volatile extern int16_t remote_yaw_change;
volatile extern arduino_data data_usart_3;

extern RC_Ctl_t RC_Ctl;

void TIM2_Configuration(void)
{
    TIM_TimeBaseInitTypeDef  tim;
    NVIC_InitTypeDef         nvic;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);

    nvic.NVIC_IRQChannel = TIM2_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 2;
    nvic.NVIC_IRQChannelSubPriority = 2;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    tim.TIM_Prescaler = 42-1;//cyq:APB1=42MHz,APB2=84MHz
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_Period = 1000;// 1 MHz down to 1 KHz (1 ms)
    TIM_TimeBaseInit(TIM2,&tim);
    /* TIM IT enable */
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    /* TIM2 enable counter */
    TIM_Cmd(TIM2, ENABLE);
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);

}

void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        Remote_Control();
        set_Pitch_Yaw_Position(pitch_Position, yaw_Position);
        int16_t target_velocity_201 = (-1*drive + strafe + rotate);
        int16_t target_velocity_202 = (drive + strafe + rotate);
        int16_t target_velocity_203 = (drive - strafe + rotate);
        int16_t target_velocity_204 = (-1*drive - strafe + rotate);
        // #if PID_CHASSIS
            int16_t motor_201_vel = set_chassis_motor_velocity(201, target_velocity_201);
            int16_t motor_202_vel = set_chassis_motor_velocity(202, target_velocity_202);
            int16_t motor_203_vel = set_chassis_motor_velocity(203, target_velocity_203);
            int16_t motor_204_vel = set_chassis_motor_velocity(204, target_velocity_204);
        // #else
            // int16_t motor_201_vel = 11 * target_velocity_201;
            // int16_t motor_202_vel = 11 * target_velocity_202;
            // int16_t motor_203_vel = 11 * target_velocity_203;
            // int16_t motor_204_vel = 11 * target_velocity_204;
        // #endif
        wheel_control(motor_201_vel, motor_202_vel, motor_203_vel, motor_204_vel);
				// wheel_control(-500, 500, 500, -500);

    }
}
