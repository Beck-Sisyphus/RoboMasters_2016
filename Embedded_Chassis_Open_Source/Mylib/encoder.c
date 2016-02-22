#include "main.h"

//TIM3 TIM5 connect to encoder,a-b
//TIM3_CH1 ----- PC6
//TIM3_CH2 ----- PC7

//TIM5_CH1 ----- PA0
//TIM5_CH2 ----- PA1

void Encoder_Configuration(void)
{
    GPIO_InitTypeDef gpio;
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3  | RCC_APB1Periph_TIM5 ,ENABLE);
    
    gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOC,&gpio);
    
    gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_Init(GPIOA,&gpio);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource0,  GPIO_AF_TIM5);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1,  GPIO_AF_TIM5);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6,  GPIO_AF_TIM3);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7,  GPIO_AF_TIM3);
    
    TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    TIM_EncoderInterfaceConfig(TIM5, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    
    TIM_Cmd(TIM3, ENABLE);
    TIM_Cmd(TIM5, ENABLE);
}
