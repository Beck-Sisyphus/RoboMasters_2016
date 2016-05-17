#include "timer2.h"

//Timer 2 32-bit counter
//Timer Clock is 168MHz / 4 * 2 = 84M

void TIM2_Configuration(void)
{
    TIM_TimeBaseInitTypeDef tim;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
    tim.TIM_Period = 0xFFFFFFFF;
    tim.TIM_Prescaler = 84 - 1;	 //1M internal clock
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_ARRPreloadConfig(TIM2, ENABLE);
    TIM_TimeBaseInit(TIM2, &tim);

    TIM_Cmd(TIM2,ENABLE);
}

void TIM2_IRQHandler(void)
{
	  if (TIM_GetITStatus(TIM2,TIM_IT_Update)!= RESET)
		{
			  TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
        TIM_ClearFlag(TIM2, TIM_FLAG_Update);
			  BOTH_LED_TOGGLE();
		}
}

int32_t ms_count = 0;
uint32_t Get_Time_Micros(void)
{
	return TIM2->CNT;
}

/* 2015 old code */
// void TIM2_Configuration(void)
// {
//     TIM_TimeBaseInitTypeDef  tim;
//     NVIC_InitTypeDef         nvic;
//
//     RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
//
//     nvic.NVIC_IRQChannel = TIM2_DAC_IRQn;
//     nvic.NVIC_IRQChannelPreemptionPriority = 1;
//     nvic.NVIC_IRQChannelSubPriority = 0;
//     nvic.NVIC_IRQChannelCmd = ENABLE;
//     NVIC_Init(&nvic);
//
//     tim.TIM_Prescaler = 84-1;
//     tim.TIM_CounterMode = TIM_CounterMode_Up;
//     tim.TIM_ClockDivision = TIM_CKD_DIV1;
//     tim.TIM_Period = 50000;
//     TIM_TimeBaseInit(TIM2,&tim);
// }

// void TIM2_Start(void)
// {
//     TIM_Cmd(TIM2, ENABLE);
//     TIM_ITConfig(TIM2, TIM_IT_Update,ENABLE);
//     TIM_ClearFlag(TIM2, TIM_FLAG_Update);
// }
