#include "main.h"

volatile extern int16_t pitch_Position;
volatile extern int16_t yaw_Position;
// volatile extern int16_t pitch_Velocity;
// volatile extern int16_t yaw_Velocity;

// for velocity controlling pitch and yaw with remote
volatile extern int16_t remote_pitch_change;
volatile extern int16_t remote_yaw_change;

extern RC_Ctl_t RC_Ctl;

void TIM6_Configuration(void)
{
    pitch_Position = 1571; // 90 degree
    yaw_Position = 0;

    TIM_TimeBaseInitTypeDef  tim;
    NVIC_InitTypeDef         nvic;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE);

    nvic.NVIC_IRQChannel = TIM6_DAC_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    tim.TIM_Prescaler = 42-1;        //42M internal clock
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_Period = 1000;  // 1 MHz down to 1 KHz (1 ms)
    TIM_TimeBaseInit(TIM6,&tim);
    TIM6_Start();
}

void TIM6_Start(void)
{
    TIM_ITConfig(TIM6, TIM_IT_Update,ENABLE);
    TIM_Cmd(TIM6, ENABLE);
    TIM_ClearFlag(TIM6, TIM_FLAG_Update);
}

void TIM6_DAC_IRQHandler(void)
{

    if (TIM_GetITStatus(TIM6,TIM_IT_Update)!= RESET)
	  {
    		TIM_ClearITPendingBit(TIM6,TIM_IT_Update);
        TIM_ClearFlag(TIM6, TIM_FLAG_Update);
        // set_Pitch_Yaw_Position(pitch_Position, yaw_Position);
        Remote_Control();
        Control_Task();
    }
}
