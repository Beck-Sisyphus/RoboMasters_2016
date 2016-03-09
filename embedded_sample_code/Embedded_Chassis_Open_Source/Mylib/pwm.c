#include "main.h"

/*-PWM1---(PA9---TIM1_CH2)--*/
/*-PWM2---(PA8---TIM1_CH1)--*/

void PWM_Configuration(void)
{
    GPIO_InitTypeDef          gpio;
    TIM_TimeBaseInitTypeDef   tim;
    TIM_OCInitTypeDef         oc;
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOA,&gpio);
    
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource8, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource9, GPIO_AF_TIM1);    
    
    tim.TIM_Prescaler = 168-1;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_Period = 2500;   //2.5ms
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM1,&tim);
    
    oc.TIM_OCMode = TIM_OCMode_PWM2;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OutputNState = TIM_OutputState_Disable;
    oc.TIM_Pulse = 0;
    oc.TIM_OCPolarity = TIM_OCPolarity_Low;
    oc.TIM_OCNPolarity = TIM_OCPolarity_High;
    oc.TIM_OCIdleState = TIM_OCIdleState_Reset;
    oc.TIM_OCNIdleState = TIM_OCIdleState_Set;
    TIM_OC1Init(TIM1,&oc);
    TIM_OC2Init(TIM1,&oc);
    
    TIM_OC1PreloadConfig(TIM1,TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM1,TIM_OCPreload_Enable);
           
    TIM_ARRPreloadConfig(TIM1,ENABLE);
    
    TIM_CtrlPWMOutputs(TIM1,ENABLE);
    
    TIM_Cmd(TIM1,ENABLE);
}
