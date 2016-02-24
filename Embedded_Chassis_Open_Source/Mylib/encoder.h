#ifndef __ENCODER_H__
#define __ENCODER_H__

void Encoder_Configuration(void);

#define Encoder_Start()   TIM3->CNT = 0x7fff
#define Encoder_Reset()   TIM3->CNT = 0x7fff
#define Encoder_Get_Cnt() ((TIM3->CNT)-0x7fff)

#endif 
