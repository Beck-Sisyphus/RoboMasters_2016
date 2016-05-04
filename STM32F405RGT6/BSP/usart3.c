#include "usart3.h"
#include "laser.h"
//#include "main.h"
#include "usart1.h" // ONLY for the RC_Ctl_t, remove this when we use diff. type
#include "led.h"


volatile unsigned char arduino_rx_buffer_usart_3[16];
DMA_InitTypeDef dma_usart_3;
// RC_Ctl_t RC_Ctl_usart_3;
arduino_data data_usart_3;
int16_t testData;

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
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3);

    /* -------------- Configure GPIO ---------------------------------------*/
    gpio.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_10;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB,&gpio);

    usart3.USART_BaudRate = 115200;
    usart3.USART_WordLength = USART_WordLength_8b;
    usart3.USART_StopBits = USART_StopBits_1;
    usart3.USART_Parity = USART_Parity_Even;
    usart3.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
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
    dma_usart_3.DMA_Memory0BaseAddr = (uint32_t)arduino_rx_buffer_usart_3;
    dma_usart_3.DMA_DIR = DMA_DIR_PeripheralToMemory;
    dma_usart_3.DMA_BufferSize = 16;
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


int fputc(int ch, FILE *f)
{
    while (USART_GetFlagStatus(USART3,USART_FLAG_TC) == RESET);
    USART_SendData(USART3, (uint8_t)ch);
    return ch;
}

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

        data_usart_3.packet.header = (((int16_t) arduino_rx_buffer_usart_3[0] << 8)) | (arduino_rx_buffer_usart_3[1] & 255);
        data_usart_3.packet.feeder_motor_state = arduino_rx_buffer_usart_3[2] & 255;
        data_usart_3.packet.friction_motor_state = arduino_rx_buffer_usart_3[3] & 255;
        data_usart_3.packet.pitch_req = (((int16_t) arduino_rx_buffer_usart_3[4] << 8)) | (arduino_rx_buffer_usart_3[5] & 255);
        data_usart_3.packet.yaw_req = (((int16_t) arduino_rx_buffer_usart_3[6] << 8)) | (arduino_rx_buffer_usart_3[7] & 255);
        data_usart_3.packet.feeder_motor_pwm = (((int16_t) arduino_rx_buffer_usart_3[8] << 8)) | (arduino_rx_buffer_usart_3[9] & 255);
        data_usart_3.packet.friction_motor_pwm = (((int16_t) arduino_rx_buffer_usart_3[10] << 8)) | (arduino_rx_buffer_usart_3[11] & 255);
    }
}
