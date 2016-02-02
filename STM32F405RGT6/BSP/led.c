#include "led.h"

void LED_Configuration(void)
{
	GPIO_InitTypeDef gpio;   

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
		
	gpio.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_1;	
	gpio.GPIO_Mode = GPIO_Mode_OUT;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOC, &gpio);
	LED1_OFF();
	LED2_OFF();
}

