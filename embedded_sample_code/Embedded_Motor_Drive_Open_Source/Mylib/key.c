#include "main.h"

void KEY_Configuration(void)
{
    GPIO_InitTypeDef gpio;  
    EXTI_InitTypeDef exti;
    NVIC_InitTypeDef nvic;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
    
	gpio.GPIO_Pin = GPIO_Pin_15;	
	gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	gpio.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOA, &gpio);
    
    exti.EXTI_Line = EXTI_Line15;
    exti.EXTI_Mode = EXTI_Mode_Interrupt;
    exti.EXTI_Trigger = EXTI_Trigger_Falling;
    exti.EXTI_LineCmd = ENABLE;
    EXTI_Init(&exti);
    
    nvic.NVIC_IRQChannel = EXTI15_10_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource15);
}

void EXTI15_10_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line15)!=RESET)
    {
        EXTI_ClearITPendingBit(EXTI_Line15);

    }
}
