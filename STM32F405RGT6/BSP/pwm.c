#include "main.h"

/*-LEFT---(PA1---TIM5_CH2)--*/
/*-RIGHT--(PA0--TIM5_CH1)--*/
TIM_OCInitTypeDef         oc;

void PWM_Configuration(void)
{
    GPIO_InitTypeDef          gpio;
	 
    TIM_TimeBaseInitTypeDef   tim;
    TIM_OCInitTypeDef         oc;
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA ,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_0;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOA,&gpio);

    GPIO_PinAFConfig(GPIOA,GPIO_PinSource1, GPIO_AF_TIM5);
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_TIM5);       

    tim.TIM_Prescaler = 84-1;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_Period = 2500;   // Sets period of 2.5ms
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM5,&tim);
    
    oc.TIM_OCMode = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OutputNState = TIM_OutputState_Disable;
    oc.TIM_Pulse = 625; // output high 625 us per cycle
    oc.TIM_OCPolarity = TIM_OCPolarity_High;
    oc.TIM_OCNPolarity = TIM_OCPolarity_Low;
    oc.TIM_OCIdleState = TIM_OCIdleState_Reset;
    oc.TIM_OCNIdleState = TIM_OCIdleState_Set;

    // TIM5 channel 1, 2 intialize
    TIM_OC1Init(TIM5,&oc);
    TIM_OC2Init(TIM5,&oc);

    TIM_OC1PreloadConfig(TIM5,TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM5,TIM_OCPreload_Enable);  
           
    TIM_ARRPreloadConfig(TIM5,ENABLE);

    TIM_Cmd(TIM5,ENABLE);
    

}
void set(int pulse){
    oc.TIM_Pulse = pulse;
}

// DIfferent GPIO modes
// typedef enum
// { 
//   GPIO_Mode_IN   = 0x00, /*!< GPIO Input Mode */
//   GPIO_Mode_OUT  = 0x01, /*!< GPIO Output Mode */
//   GPIO_Mode_AF   = 0x02, /*!< GPIO Alternate function Mode */
//   GPIO_Mode_AN   = 0x03  /*!< GPIO Analog Mode */
// }GPIOMode_TypeDef;



