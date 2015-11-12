#include "main.h"

void TIM6_Configuration(void)
{
    TIM_TimeBaseInitTypeDef  tim;
    NVIC_InitTypeDef         nvic;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE);

    nvic.NVIC_IRQChannel = TIM6_DAC_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 2;
    nvic.NVIC_IRQChannelSubPriority = 2;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    tim.TIM_Prescaler = 42-1;//cyq:APB1=42MHz,APB2=84MHz
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_Period = 1000;//cyq:1Khz
    TIM_TimeBaseInit(TIM6,&tim);
}

void TIM6_Start(void)
{
    TIM_Cmd(TIM6, ENABLE);	 
    TIM_ITConfig(TIM6, TIM_IT_Update,ENABLE);
    TIM_ClearFlag(TIM6, TIM_FLAG_Update);	
}


int encoder_cnt=0;
void TIM6_DAC_IRQHandler(void)  
{
    CanTxMsg tx_message;
    if (TIM_GetITStatus(TIM6,TIM_IT_Update) != RESET) 
	{
        LED_TOGGLE();//cyq
        TIM_ClearITPendingBit(TIM6,TIM_IT_Update);
        TIM_ClearFlag(TIM6, TIM_FLAG_Update);
        
        encoder_cnt = Encoder_Get_Cnt();
        
       tx_message.StdId = MY_ID;//cyq
       tx_message.IDE = CAN_Id_Standard;
       tx_message.RTR = CAN_RTR_Data;
       tx_message.DLC = 0x08;  
       
       tx_message.Data[0] = (unsigned char)(encoder_cnt>>8);
       tx_message.Data[1] = (unsigned char)(encoder_cnt&0xff);
       tx_message.Data[2] = 0x11;
       tx_message.Data[3] = 0x11;
       tx_message.Data[4] = 0x11;
       tx_message.Data[5] = 0x11;
       tx_message.Data[6] = 0x11;
       tx_message.Data[7] = 0x11;     
 
       CAN_Transmit(CAN1,&tx_message);
    }
}
