#ifndef __LED_H__
#define __LED_H__

#include <stm32f4xx.h>

#define LED1_ON()       GPIO_ResetBits(GPIOC,GPIO_Pin_2)
#define LED1_OFF()      GPIO_SetBits(GPIOC,GPIO_Pin_2)
#define LED1_TOGGLE()   (GPIOC->ODR) ^= GPIO_Pin_2//RED (2015: red)

#define LED2_ON()       GPIO_ResetBits(GPIOC,GPIO_Pin_1)
#define LED2_OFF()      GPIO_SetBits(GPIOC,GPIO_Pin_1)
#define LED2_TOGGLE()   (GPIOC->ODR) ^= GPIO_Pin_1//BLUE (2015: green)


void LED_Configuration(void);


#endif
