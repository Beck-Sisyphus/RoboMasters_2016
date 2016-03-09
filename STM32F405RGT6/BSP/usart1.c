#include "usart1.h"
#include "laser.h"

volatile unsigned char sbus_rx_buffer[25]; 
DMA_InitTypeDef dma; 
RC_Ctl_t RC_Ctl;

/*-----USART1_TX-----NC-----*/
/*-----USART1_RX-----PB7----*/

void USART1_Configuration(void)
{
    USART_InitTypeDef usart1;
    GPIO_InitTypeDef  gpio;
    NVIC_InitTypeDef  nvic;

    /* -------------- Enable Module Clock Source ----------------------------*/
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_DMA2,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_USART1); 

    /* -------------- Configure GPIO ---------------------------------------*/
    gpio.GPIO_Pin = GPIO_Pin_7;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB,&gpio);

    usart1.USART_BaudRate = 100000;
    usart1.USART_WordLength = USART_WordLength_8b;
    usart1.USART_StopBits = USART_StopBits_1;
    usart1.USART_Parity = USART_Parity_Even;
    usart1.USART_Mode = USART_Mode_Rx;
    usart1.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART1,&usart1);

    USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
    USART_Cmd(USART1,ENABLE);
    USART_DMACmd (USART1, USART_DMAReq_Rx, ENABLE);

    /* -------------- Configure NVIC ---------------------------------------*/
    nvic.NVIC_IRQChannel = DMA2_Stream2_IRQn;

    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    /* -------------- Configure DMA -----------------------------------------*/ 
    DMA_DeInit(DMA2_Stream2); 
    dma.DMA_Channel = DMA_Channel_4; 
    dma.DMA_PeripheralBaseAddr = (uint32_t)&(USART1->DR);
    dma.DMA_Memory0BaseAddr = (uint32_t)sbus_rx_buffer; 
    dma.DMA_DIR = DMA_DIR_PeripheralToMemory; 
    dma.DMA_BufferSize = 18; 
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable; 
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; 
    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    dma.DMA_Mode = DMA_Mode_Circular; 
    dma.DMA_Priority = DMA_Priority_VeryHigh; 
    dma.DMA_FIFOMode = DMA_FIFOMode_Disable; 
    dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull; 
    dma.DMA_MemoryBurst = DMA_Mode_Normal; 
    dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single; 
    DMA_Init(DMA2_Stream2,&dma); 
    DMA_ITConfig(DMA2_Stream2,DMA_IT_TC,ENABLE); 
    DMA_Cmd(DMA2_Stream2,ENABLE); 
}

void USART1_SendChar(unsigned char b)
{
    while (USART_GetFlagStatus(USART1,USART_FLAG_TC) == RESET);
    USART_SendData(USART1,b);
}

int fputc(int ch, FILE *f)
{
    while (USART_GetFlagStatus(USART1,USART_FLAG_TC) == RESET);
    USART_SendData(USART1, (uint8_t)ch);    
    return ch;
}


static u16 RS232_VisualScope_CRC16( u8 *Array, u16 Len )
{
	u16 USART_IX, USART_IY, USART_CRC;

	USART_CRC = 0xffff;
	for(USART_IX=0; USART_IX<Len; USART_IX++) {
		USART_CRC = USART_CRC^(u16)(Array[USART_IX]);
		for(USART_IY=0; USART_IY<=7; USART_IY++) {
			if((USART_CRC&1)!=0)
				USART_CRC = (USART_CRC>>1)^0xA001;
			else
				USART_CRC = USART_CRC>>1;
		}
	}
	return(USART_CRC);
}



void RS232_VisualScope( USART_TypeDef* USARTx, u8 *pWord, u16 Len )
{
	u8 i = 0;
	u16 Temp = 0;

	Temp = RS232_VisualScope_CRC16(pWord, Len);
	pWord[8] = Temp&0x00ff;
	pWord[9] = (Temp&0xff00)>>8;

	for(i=0; i<10; i++) {
		USART_SendData(USARTx, (uint8_t)*pWord);
		while(USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET);
		pWord++;
	}
}



void USART1_IRQHandler(void)
{
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        USART_ClearITPendingBit(USART1,USART_IT_RXNE);
        
    }
}

void DMA2_Stream2_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA2_Stream2, DMA_IT_TCIF2)) 
    {
        DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2); 
        DMA_ClearITPendingBit(DMA2_Stream2, DMA_IT_TCIF2); 

        RC_Ctl.rc.ch0 = (sbus_rx_buffer[0] | (sbus_rx_buffer[1] << 8)) & 0x07ff;         //!< Channel 0 
        RC_Ctl.rc.ch1 = ((sbus_rx_buffer[1] >> 3) | (sbus_rx_buffer[2] << 5)) & 0x07ff;  //!< Channel 1 
        RC_Ctl.rc.ch2 = ((sbus_rx_buffer[2] >> 6) | (sbus_rx_buffer[3] << 2) |           //!< Channel 2 
                          (sbus_rx_buffer[4] << 10)) & 0x07ff; 
        RC_Ctl.rc.ch3 = ((sbus_rx_buffer[4] >> 1) | (sbus_rx_buffer[5] << 7)) & 0x07ff;  //!< Channel 3 
        RC_Ctl.rc.s1  = ((sbus_rx_buffer[5] >> 4)& 0x000C) >> 2;                         //!< Switch left 
        RC_Ctl.rc.s2  = ((sbus_rx_buffer[5] >> 4)& 0x0003);                              //!< Switch right     
        RC_Ctl.mouse.x = sbus_rx_buffer[6] | (sbus_rx_buffer[7] << 8);                   //!< Mouse X axis 
        RC_Ctl.mouse.y = sbus_rx_buffer[8] | (sbus_rx_buffer[9] << 8);                   //!< Mouse Y axis 
        RC_Ctl.mouse.z = sbus_rx_buffer[10] | (sbus_rx_buffer[11] << 8);                 //!< Mouse Z axis 
        RC_Ctl.mouse.press_l = sbus_rx_buffer[12];                                       //!< Mouse Left Is Press ? 
        RC_Ctl.mouse.press_r = sbus_rx_buffer[13];                                       //!< Mouse Right Is Press ? 
        RC_Ctl.key.v = sbus_rx_buffer[14] | (sbus_rx_buffer[15] << 8);                   //!< KeyBoard value 
    }
}
