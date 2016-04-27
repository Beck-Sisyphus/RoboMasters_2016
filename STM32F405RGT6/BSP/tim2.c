#include "main.h"

int16_t pitch_Position;
int16_t yaw_Position;
int16_t pitch_Velocity;
int16_t yaw_Velocity;


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
			  // MPU6050_ReadData();
        // set_Pitch_Position(pitch_Position);
        // set_Yaw_Position(yaw_Position);
        set_Pitch_Yaw_Position(pitch_Position, yaw_Position);
        // float pitch_velocity_change = Velocity_Control_205((float)MPU6050_Real_Data.Gyro_Y, pitch_Velocity);
        // float yaw_velocity_change = Velocity_Control_206((float)MPU6050_Real_Data.Gyro_Z, yaw_Velocity);
        // pitchyaw_control((int16_t) yaw_velocity_change, (int16_t)pitch_velocity_change);
    }
}
