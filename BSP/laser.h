#ifndef __LASER_H__
#define __LASER_H__

#include <stm32f4xx.h>

#define SHOOT() GPIO_SetBits(GPIOB,GPIO_Pin_8);GPIO_ResetBits(GPIOB,GPIO_Pin_9) 
#define HALT()  GPIO_ResetBits(GPIOB,GPIO_Pin_8);GPIO_SetBits(GPIOB,GPIO_Pin_9)

#define L_PWM  TIM4->CCR4
#define H_PWM  TIM4->CCR3

void RELAY_Configuration(void);
void LASER_Configuration(void);
void PWM2_Configuration(void);

#endif 
