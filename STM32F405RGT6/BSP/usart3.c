#include "usart3.h"
#include "laser.h"
//#include "main.h"
#include "usart1.h" // ONLY for the RC_Ctl_t, remove this when we use diff. type

volatile unsigned char arduino_rx_buffer_usart_3[18]; 
DMA_InitTypeDef dma_usart_3; 
// RC_Ctl_t RC_Ctl_usart_3;
arduino_data data_usart_3;

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


int fputc(int ch, FILE *f)
{
    while (USART_GetFlagStatus(USART3,USART_FLAG_TC) == RESET);
    USART_SendData(USART3, (uint8_t)ch);    
    return ch;
}


// static u16 RS232_VisualScope_CRC16_USART3( u8 *Array, u16 Len )
// {
//  u16 USART_IX, USART_IY, USART_CRC;

//  USART_CRC = 0xffff;
//  for(USART_IX=0; USART_IX<Len; USART_IX++) {
//    USART_CRC = USART_CRC^(u16)(Array[USART_IX]);
//    for(USART_IY=0; USART_IY<=7; USART_IY++) {
//      if((USART_CRC&1)!=0)
//        USART_CRC = (USART_CRC>>1)^0xA001;
//      else
//        USART_CRC = USART_CRC>>1;
//    }
//  }
//  return(USART_CRC);
// }



// void RS232_VisualScope_USART3( USART_TypeDef* USARTx, u8 *pWord, u16 Len )
// {
//  u8 i = 0;
//  u16 Temp = 0;

//  Temp = RS232_VisualScope_CRC16(pWord, Len);
//  pWord[8] = Temp&0x00ff;
//  pWord[9] = (Temp&0xff00)>>8;

//  for(i=0; i<10; i++) {
//    USART_SendData(USARTx, (uint8_t)*pWord);
//    while(USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET);
//    pWord++;
//  }
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

        // data_usart_3.packet.p1 = arduino_rx_buffer_usart_3[0];
        data_usart_3.packet.p1 = arduino_rx_buffer_usart_3[17];
    }
}


// ===============================================================================
//                             Data transfers functions
//  ===============================================================================  

//   This subsection provides a set of functions allowing to manage the USART data 
//   transfers.
  
//   During an USART reception, data shifts in least significant bit first through 
//   the RX pin. In this mode, the USART_DR register consists of a buffer (RDR) 
//   between the internal bus and the received shift register.

//   When a transmission is taking place, a write instruction to the USART_DR register 
//   stores the data in the TDR register and which is copied in the shift register 
//   at the end of the current transmission.

//   The read access of the USART_DR register can be done using the USART_ReceiveData()
//   function and returns the RDR buffered value. Whereas a write access to the USART_DR 
//   can be done using USART_SendData() function and stores the written data into 
//   TDR buffer.

// @endverbatim
//   * @{
//   */

// /**
//   * @brief  Transmits single data through the USARTx peripheral.
//   * @param  USARTx: where x can be 1, 2, 3, 4, 5 or 6 to select the USART or 
//   *         UART peripheral.
//   * @param  Data: the data to transmit.
//   * @retval None
//   */
// void USART_SendData(USART_TypeDef* USARTx, uint16_t Data)
// {
//   /* Check the parameters */
//   assert_param(IS_USART_ALL_PERIPH(USARTx));
//   assert_param(IS_USART_DATA(Data)); 
    
//   /* Transmit Data */
//   USARTx->DR = (Data & (uint16_t)0x01FF);
// }

// /**
//   * @brief  Returns the most recent received data by the USARTx peripheral.
//   * @param  USARTx: where x can be 1, 2, 3, 4, 5 or 6 to select the USART or 
//   *         UART peripheral.
//   * @retval The received data.
//   */
// uint16_t USART_ReceiveData(USART_TypeDef* USARTx)
// {
//   /* Check the parameters */
//   assert_param(IS_USART_ALL_PERIPH(USARTx));
  
//   /* Receive Data */
//   return (uint16_t)(USARTx->DR & (uint16_t)0x01FF);
// }
