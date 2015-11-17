#ifndef __LED_H__
#define __LED_H__

#include <stm32f4xx.h>

#define LED1_ON()       GPIO_ResetBits(GPIOB,GPIO_Pin_14)
#define LED1_OFF()      GPIO_SetBits(GPIOB,GPIO_Pin_14)
#define LED1_TOGGLE()   (GPIOB->ODR) ^= GPIO_Pin_14//RED

#define LED2_ON()       GPIO_ResetBits(GPIOB,GPIO_Pin_15)
#define LED2_OFF()      GPIO_SetBits(GPIOB,GPIO_Pin_15)
#define LED2_TOGGLE()   (GPIOB->ODR) ^= GPIO_Pin_15//BLUE


void LED_Configuration(void);


#endif 
