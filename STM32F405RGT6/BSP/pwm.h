#ifndef __PWM_H__
#define __PWM_H__

void PWM_Configuration(void);
void feeder_motor_on(void);
void feeder_motor_off(void);
void friction_motor_on(void);
void friction_motor_off(void);
void usart3_receive(void);

#define PWM1  TIM5->CCR1
#define PWM2  TIM5->CCR2
#define PWM3  TIM5->CCR3

#endif


