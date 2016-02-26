#include "usart3.h"
#include "laser.h"
//#include "main.h"
#include "usart1.h" // ONLY for the RC_Ctl_t, remove this when we use diff. type

volatile unsigned char sbus_rx_buffer_usart_3[25]; 
DMA_InitTypeDef dma_usart_3; 
RC_Ctl_t RC_Ctl_usart_3;

/*-----USART3_TX-----PB10---*/
/*-----USART3_RX-----PB11---*/

void USART3_Configuration(void)
{
    USART_InitTypeDef usart3;
    GPIO_InitTypeDef  gpio;
    NVIC_InitTypeDef  nvic;

    /* -------------- Enable Module Clock Source ----------------------------*/
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_DMA1,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3); 

    /* -------------- Configure GPIO ---------------------------------------*/
    gpio.GPIO_Pin = GPIO_Pin_11;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB,&gpio);

    usart3.USART_BaudRate = 100000;
    usart3.USART_WordLength = USART_WordLength_8b;
    usart3.USART_StopBits = USART_StopBits_1;
    usart3.USART_Parity = USART_Parity_Even;
    usart3.USART_Mode = USART_Mode_Rx;
    usart3.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART3,&usart3);

    USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);
    USART_Cmd(USART3,ENABLE);
    USART_DMACmd (USART3, USART_DMAReq_Rx, ENABLE);

    /* -------------- Configure NVIC ---------------------------------------*/
    nvic.NVIC_IRQChannel = DMA1_Stream1_IRQn;

    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    /* -------------- Configure DMA -----------------------------------------*/ 
    DMA_DeInit(DMA1_Stream1); 
    dma_usart_3.DMA_Channel = DMA_Channel_4; 
    dma_usart_3.DMA_PeripheralBaseAddr = (uint32_t)&(USART3->DR);
    dma_usart_3.DMA_Memory0BaseAddr = (uint32_t)sbus_rx_buffer_usart_3; 
    dma_usart_3.DMA_DIR = DMA_DIR_PeripheralToMemory; 
    dma_usart_3.DMA_BufferSize = 18; 
    dma_usart_3.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 
    dma_usart_3.DMA_MemoryInc = DMA_MemoryInc_Enable; 
    dma_usart_3.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; 
    dma_usart_3.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    dma_usart_3.DMA_Mode = DMA_Mode_Circular; 
    dma_usart_3.DMA_Priority = DMA_Priority_VeryHigh; 
    dma_usart_3.DMA_FIFOMode = DMA_FIFOMode_Disable; 
    dma_usart_3.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull; 
    dma_usart_3.DMA_MemoryBurst = DMA_Mode_Normal; 
    dma_usart_3.DMA_PeripheralBurst = DMA_PeripheralBurst_Single; 
    DMA_Init(DMA1_Stream1,&dma_usart_3); 
    DMA_ITConfig(DMA1_Stream1,DMA_IT_TC,ENABLE); 
    DMA_Cmd(DMA1_Stream1,ENABLE); 
}

void USART3_SendChar(unsigned char b)
{
    while (USART_GetFlagStatus(USART3,USART_FLAG_TC) == RESET);
    USART_SendData(USART3,b);
}

// int fputc(int ch, FILE *f)
// {
//     while (USART_GetFlagStatus(USART3,USART_FLAG_TC) == RESET);
//     USART_SendData(USART3, (uint8_t)ch);    
//     return ch;
// }


// static u16 RS232_VisualScope_CRC16( u8 *Array, u16 Len )
// {
// 	u16 USART_IX, USART_IY, USART_CRC;

// 	USART_CRC = 0xffff;
// 	for(USART_IX=0; USART_IX<Len; USART_IX++) {
// 		USART_CRC = USART_CRC^(u16)(Array[USART_IX]);
// 		for(USART_IY=0; USART_IY<=7; USART_IY++) {
// 			if((USART_CRC&1)!=0)
// 				USART_CRC = (USART_CRC>>1)^0xA001;
// 			else
// 				USART_CRC = USART_CRC>>1;
// 		}
// 	}
// 	return(USART_CRC);
// }



// void RS232_VisualScope( USART_TypeDef* USARTx, u8 *pWord, u16 Len )
// {
// 	u8 i = 0;
// 	u16 Temp = 0;

// 	Temp = RS232_VisualScope_CRC16(pWord, Len);
// 	pWord[8] = Temp&0x00ff;
// 	pWord[9] = (Temp&0xff00)>>8;

// 	for(i=0; i<10; i++) {
// 		USART_SendData(USARTx, (uint8_t)*pWord);
// 		while(USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET);
// 		pWord++;
// 	}
// }



void USART3_IRQHandler(void)
{
    if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        USART_ClearITPendingBit(USART3,USART_IT_RXNE);
        
    }
}

void DMA1_Stream1_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA1_Stream1, DMA_IT_TCIF1)) // DMA_IT_TCI_F1 because we are clearing stream 1
    {
        DMA_ClearFlag(DMA1_Stream1, DMA_FLAG_TCIF1); 
        DMA_ClearITPendingBit(DMA1_Stream1, DMA_IT_TCIF1); 

        RC_Ctl_usart_3.rc.ch0 = (sbus_rx_buffer_usart_3[0] | (sbus_rx_buffer_usart_3[1] << 8)) & 0x07ff;         //!< Channel 0 
        RC_Ctl_usart_3.rc.ch1 = ((sbus_rx_buffer_usart_3[1] >> 3) | (sbus_rx_buffer_usart_3[2] << 5)) & 0x07ff;  //!< Channel 1 
        RC_Ctl_usart_3.rc.ch2 = ((sbus_rx_buffer_usart_3[2] >> 6) | (sbus_rx_buffer_usart_3[3] << 2) |           //!< Channel 2 
                          (sbus_rx_buffer_usart_3[4] << 10)) & 0x07ff; 
        RC_Ctl_usart_3.rc.ch3 = ((sbus_rx_buffer_usart_3[4] >> 1) | (sbus_rx_buffer_usart_3[5] << 7)) & 0x07ff;  //!< Channel 3 
        RC_Ctl_usart_3.rc.s1  = ((sbus_rx_buffer_usart_3[5] >> 4)& 0x000C) >> 2;                         //!< Switch left 
        RC_Ctl_usart_3.rc.s2  = ((sbus_rx_buffer_usart_3[5] >> 4)& 0x0003);                              //!< Switch right     
        RC_Ctl_usart_3.mouse.x = sbus_rx_buffer_usart_3[6] | (sbus_rx_buffer_usart_3[7] << 8);                   //!< Mouse X axis 
        RC_Ctl_usart_3.mouse.y = sbus_rx_buffer_usart_3[8] | (sbus_rx_buffer_usart_3[9] << 8);                   //!< Mouse Y axis 
        RC_Ctl_usart_3.mouse.z = sbus_rx_buffer_usart_3[10] | (sbus_rx_buffer_usart_3[11] << 8);                 //!< Mouse Z axis 
        RC_Ctl_usart_3.mouse.press_l = sbus_rx_buffer_usart_3[12];                                       //!< Mouse Left Is Press ? 
        RC_Ctl_usart_3.mouse.press_r = sbus_rx_buffer_usart_3[13];                                       //!< Mouse Right Is Press ? 
        RC_Ctl_usart_3.key.v = sbus_rx_buffer_usart_3[14] | (sbus_rx_buffer_usart_3[15] << 8);                   //!< KeyBoard value 
    }
}
