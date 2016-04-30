#include "main.h"

extern int16_t pitch_Position;
extern int16_t yaw_Position;
extern int16_t pitch_Velocity;
extern int16_t yaw_Velocity;

// for velocity controlling pitch and yaw with remote
extern int16_t remote_pitch_change;
extern int16_t remote_yaw_change;


void TIM2_Configuration(void)
{
    TIM_TimeBaseInitTypeDef  tim;
    NVIC_InitTypeDef         nvic;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);

    // initialize starting position
    // pitch_Position = 78;
    // yaw_Position = 0;

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
        set_Pitch_Yaw_Position(remote_pitch_change, remote_yaw_change);
        // float pitch_velocity_change = Velocity_Control_205((float)MPU6050_Real_Data.Gyro_Y, pitch_Velocity);
        // float yaw_velocity_change = Velocity_Control_206((float)MPU6050_Real_Data.Gyro_Z, yaw_Velocity);
        // pitchyaw_control((int16_t) yaw_velocity_change, (int16_t)pitch_velocity_change);
    }
}
