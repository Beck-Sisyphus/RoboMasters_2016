#ifndef __BEEP_H__
#define __BEEP_H__

#define BEEP_ON()        GPIO_SetBits(GPIOB,GPIO_Pin_14)
#define BEEP_OFF()       GPIO_ResetBits(GPIOB,GPIO_Pin_14)
#define BEEP_TOGGLE()    GPIOB->ODR ^= GPIO_Pin_14

void BEEP_Configuration(void);

#endif
