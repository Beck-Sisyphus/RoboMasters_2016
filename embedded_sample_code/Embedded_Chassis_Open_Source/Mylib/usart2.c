#include "main.h"

/*-----USART2_RX-----PA3----*/ 
//for D-BUS

unsigned char sbus_rx_buffer[18];

void USART2_Configuration(void)
{
    USART_InitTypeDef usart2;
	GPIO_InitTypeDef  gpio;
    NVIC_InitTypeDef  nvic;
    DMA_InitTypeDef   dma;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_DMA1,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3 ,GPIO_AF_USART2);
	
	gpio.GPIO_Pin = GPIO_Pin_3 ;
	gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA,&gpio);
    
    USART_DeInit(USART2);
	usart2.USART_BaudRate = 100000;   //SBUS 100K baudrate
	usart2.USART_WordLength = USART_WordLength_8b;
	usart2.USART_StopBits = USART_StopBits_1;
	usart2.USART_Parity = USART_Parity_Even;
	usart2.USART_Mode = USART_Mode_Rx;
    usart2.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART2,&usart2);
    
	USART_Cmd(USART2,ENABLE);
    USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);
    
    nvic.NVIC_IRQChannel = DMA1_Stream5_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    
    DMA_DeInit(DMA1_Stream5);
    dma.DMA_Channel= DMA_Channel_4;
    dma.DMA_PeripheralBaseAddr = (uint32_t)&(USART2->DR);
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
    DMA_Init(DMA1_Stream5,&dma);

    DMA_ITConfig(DMA1_Stream5,DMA_IT_TC,ENABLE);
    DMA_Cmd(DMA1_Stream5,ENABLE);
}

void DMA1_Stream5_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA1_Stream5, DMA_IT_TCIF5))
    {
        DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF5);
        DMA_ClearITPendingBit(DMA1_Stream5, DMA_IT_TCIF5);
    }
}
