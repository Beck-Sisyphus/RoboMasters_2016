#ifndef __PWM_H__
#define __PWM_H__

void PWM_Configuration(void);

#define PWM1  TIM1->CCR2
#define PWM2  TIM1->CCR1

#endif
