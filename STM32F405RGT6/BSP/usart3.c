#include "usart3.h"
#include "laser.h"
#include "main.h"
#include "usart1.h" // ONLY for the RC_Ctl_t, remove this when we use diff. type
#include "led.h"

volatile unsigned char arduino_rx_buffer_usart_3[32];
volatile uint16_t arduino_tx_buffer_usart_3[32];
DMA_InitTypeDef dma_usart_3;
volatile arduino_data data_usart_3;

volatile extern int16_t friction_motor_state;
volatile extern int16_t feeder_motor_state;
volatile extern RC_Ctl_t RC_Ctl;

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
    //usart3.USART_Parity = USART_Parity_Even;
    usart3.USART_Parity = USART_Parity_No;
    usart3.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    usart3.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART3,&usart3);

    USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);
    USART_Cmd(USART3,ENABLE);
    //USART_DMACmd (USART3, USART_DMAReq_Rx, ENABLE);

    /* -------------- Configure NVIC ---------------------------------------*/
    nvic.NVIC_IRQChannel = USART3_IRQn;//DMA1_Stream1_IRQn;

    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    /* -------------- Configure DMA -----------------------------------------*/
    // DMA_DeInit(DMA1_Stream1);
    // dma_usart_3.DMA_Channel = DMA_Channel_4;
    // dma_usart_3.DMA_PeripheralBaseAddr = (uint32_t)&(USART3->DR);
    // dma_usart_3.DMA_Memory0BaseAddr = (uint32_t)arduino_rx_buffer_usart_3;
    // dma_usart_3.DMA_DIR = DMA_DIR_PeripheralToMemory;
    // dma_usart_3.DMA_BufferSize = 32;
    // dma_usart_3.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    // dma_usart_3.DMA_MemoryInc = DMA_MemoryInc_Enable;
    // dma_usart_3.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    // dma_usart_3.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    // dma_usart_3.DMA_Mode = DMA_Mode_Circular;
    // dma_usart_3.DMA_Priority = DMA_Priority_VeryHigh;
    // dma_usart_3.DMA_FIFOMode = DMA_FIFOMode_Disable;
    // dma_usart_3.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    // dma_usart_3.DMA_MemoryBurst = DMA_Mode_Normal;
    // dma_usart_3.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    // DMA_Init(DMA1_Stream1,&dma_usart_3);
    // DMA_ITConfig(DMA1_Stream1,DMA_IT_TC,ENABLE);
    // DMA_Cmd(DMA1_Stream1,ENABLE);
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
        //USART_ClearITPendingBit(USART3,USART_IT_RXNE);
        if ((char) USART_ReceiveData(USART3) == 0xFA) { // 1st header byte
            while (USART_GetFlagStatus(USART3, USART_FLAG_RXNE) == RESET);
            if ((char) USART_ReceiveData(USART3) == 0x00) { // 2nd header byte
                LED1_TOGGLE();
                // (1) store data into receive buffer
                arduino_rx_buffer_usart_3[0] = 0xFA;
                arduino_rx_buffer_usart_3[1] = 0x00;
                for (int i = 2; i < 32; i++) {
                    while (USART_GetFlagStatus(USART3, USART_FLAG_RXNE) == RESET);
                    arduino_rx_buffer_usart_3[i] = (char) USART_ReceiveData(USART3);
                }

                // store received data into
                if (ROBOT_SERIAL_NUMBER != HERO_ROBOT_CANNON_7) {
                    data_usart_3.packet.header = MAKE_INT16(arduino_rx_buffer_usart_3[0], arduino_rx_buffer_usart_3[1]);
                    data_usart_3.packet.feeder_motor_state = arduino_rx_buffer_usart_3[2] & 255;
                    data_usart_3.packet.friction_motor_state = arduino_rx_buffer_usart_3[3] & 255;
                    data_usart_3.packet.pitch_req = MAKE_INT16(arduino_rx_buffer_usart_3[4], arduino_rx_buffer_usart_3[5]);
                    data_usart_3.packet.yaw_req = MAKE_INT16(arduino_rx_buffer_usart_3[6], arduino_rx_buffer_usart_3[7]);
                    data_usart_3.packet.feeder_motor_pwm = MAKE_INT16(arduino_rx_buffer_usart_3[8], arduino_rx_buffer_usart_3[9]);
                    data_usart_3.packet.friction_motor_pwm = MAKE_INT16(arduino_rx_buffer_usart_3[10], arduino_rx_buffer_usart_3[11]);
                    data_usart_3.packet.drive_req = MAKE_INT16(arduino_rx_buffer_usart_3[12], arduino_rx_buffer_usart_3[13]);
                    data_usart_3.packet.strafe_req = MAKE_INT16(arduino_rx_buffer_usart_3[14], arduino_rx_buffer_usart_3[15]);
                    data_usart_3.packet.rotate_req = MAKE_INT16(arduino_rx_buffer_usart_3[16], arduino_rx_buffer_usart_3[17]);
                    data_usart_3.packet.mpu_x = MAKE_INT16(arduino_rx_buffer_usart_3[20], arduino_rx_buffer_usart_3[21]);
                    data_usart_3.packet.mpu_y = MAKE_INT16(arduino_rx_buffer_usart_3[22], arduino_rx_buffer_usart_3[23]);
                    data_usart_3.packet.mpu_z = MAKE_INT16(arduino_rx_buffer_usart_3[24], arduino_rx_buffer_usart_3[25]);
                    data_usart_3.packet.js_real_chassis_out_power = MAKE_INT16(arduino_rx_buffer_usart_3[26], arduino_rx_buffer_usart_3[27]);

                    // (2) reply with new packet, debug must be false
                    for (int i = 0; i < 32; i++) {
                        arduino_tx_buffer_usart_3[i] = 0x00;
                    }
                    arduino_tx_buffer_usart_3[0] = 0xCE;
                    arduino_tx_buffer_usart_3[2] = feeder_motor_state & 255;
                    arduino_tx_buffer_usart_3[10] = TX_MSB(RC_Ctl.rc.ch0);
                    arduino_tx_buffer_usart_3[11] = TX_LSB(RC_Ctl.rc.ch0);
                    arduino_tx_buffer_usart_3[12] = TX_MSB(RC_Ctl.rc.ch1);
                    arduino_tx_buffer_usart_3[13] = TX_LSB(RC_Ctl.rc.ch1);
                    arduino_tx_buffer_usart_3[14] = TX_MSB(RC_Ctl.rc.ch2);
                    arduino_tx_buffer_usart_3[15] = TX_LSB(RC_Ctl.rc.ch2);
                    arduino_tx_buffer_usart_3[16] = TX_MSB(RC_Ctl.rc.ch3);
                    arduino_tx_buffer_usart_3[17] = TX_LSB(RC_Ctl.rc.ch3);
                    arduino_tx_buffer_usart_3[18] = RC_Ctl.rc.s1;
                    arduino_tx_buffer_usart_3[19] = RC_Ctl.rc.s2;
                    arduino_tx_buffer_usart_3[20] = TX_MSB(RC_Ctl.mouse.x);
                    arduino_tx_buffer_usart_3[21] = TX_LSB(RC_Ctl.mouse.x);
                    arduino_tx_buffer_usart_3[22] = TX_MSB(RC_Ctl.mouse.y);
                    arduino_tx_buffer_usart_3[23] = TX_LSB(RC_Ctl.mouse.y);
                    arduino_tx_buffer_usart_3[24] = TX_MSB(RC_Ctl.mouse.z);
                    arduino_tx_buffer_usart_3[25] = TX_LSB(RC_Ctl.mouse.z);
                    arduino_tx_buffer_usart_3[26] = RC_Ctl.mouse.press_l;
                    arduino_tx_buffer_usart_3[27] = RC_Ctl.mouse.press_r;
                    arduino_tx_buffer_usart_3[28] = TX_MSB(RC_Ctl.key.v);
                    arduino_tx_buffer_usart_3[29] = TX_LSB(RC_Ctl.key.v);

                    // send the packet
                    for (int i = 0; i < 32; i++) {
                        USART3_SendChar(arduino_tx_buffer_usart_3[i]);
                    }
                } else {
                    RC_Ctl.rc.ch0 = MAKE_INT16(arduino_rx_buffer_usart_3[10], arduino_rx_buffer_usart_3[11]);
                    RC_Ctl.rc.ch1 = MAKE_INT16(arduino_rx_buffer_usart_3[12], arduino_rx_buffer_usart_3[13]);
                    RC_Ctl.rc.ch2 = MAKE_INT16(arduino_rx_buffer_usart_3[14], arduino_rx_buffer_usart_3[15]);
                    RC_Ctl.rc.ch3 = MAKE_INT16(arduino_rx_buffer_usart_3[16], arduino_rx_buffer_usart_3[17]);
                    RC_Ctl.rc.s1 = arduino_rx_buffer_usart_3[18];
                    RC_Ctl.rc.s2 = arduino_rx_buffer_usart_3[19];
                    RC_Ctl.mouse.x = MAKE_INT16(arduino_rx_buffer_usart_3[20], arduino_rx_buffer_usart_3[21]);
                    RC_Ctl.mouse.y = MAKE_INT16(arduino_rx_buffer_usart_3[22], arduino_rx_buffer_usart_3[23]);
                    RC_Ctl.mouse.z = MAKE_INT16(arduino_rx_buffer_usart_3[24], arduino_rx_buffer_usart_3[25]);
                    RC_Ctl.mouse.press_l = arduino_rx_buffer_usart_3[26];
                    RC_Ctl.mouse.press_r = arduino_rx_buffer_usart_3[27];
                    RC_Ctl.key.v = MAKE_INT16(arduino_rx_buffer_usart_3[28], arduino_rx_buffer_usart_3[29]);
                }
            }
        }
    }
}

/*
0  1  header
2     feeder_motor_state
3     friction_motor_state
4  5  pitch_req
6  7  yaw_req
8  9  feeder_motor_pwm
10 11 friction_motor_pwm
12 13 drive_req
14 15 strafe_req
16 17 rotate_req
18 19
20 21 mpu_roll
22 23 mpu_pitch
24 25 mpu_yaw
*/
// void DMA1_Stream1_IRQHandler(void)
// {

//     if(DMA_GetITStatus(DMA1_Stream1, DMA_IT_TCIF1)) // DMA_IT_TCI_F1 because we are clearing stream 1
//     {
//         DMA_ClearFlag(DMA1_Stream1, DMA_FLAG_TCIF1);
//         DMA_ClearITPendingBit(DMA1_Stream1, DMA_IT_TCIF1);

//         // reset dma if header is not right
//         if (arduino_rx_buffer_usart_3[0] ^ 0xFA) {
//             // restart the dma
//             DMA_ClearFlag(DMA1_Stream1, DMA_FLAG_FEIF1|DMA_FLAG_DMEIF1|DMA_FLAG_TEIF1|DMA_FLAG_HTIF1|DMA_FLAG_TCIF1);
//             DMA_Cmd(DMA1_Stream1, DISABLE);
//             while (DMA1_Stream1->CR & DMA_SxCR_EN);
//             DMA_Cmd(DMA1_Stream1, ENABLE);
//         } else {
//             LED1_TOGGLE();
//             // store received data into
//             data_usart_3.packet.header = MAKE_INT16(arduino_rx_buffer_usart_3[0], arduino_rx_buffer_usart_3[1]);
//             data_usart_3.packet.feeder_motor_state = arduino_rx_buffer_usart_3[2] & 255;
//             data_usart_3.packet.friction_motor_state = arduino_rx_buffer_usart_3[3] & 255;
//             data_usart_3.packet.pitch_req = MAKE_INT16(arduino_rx_buffer_usart_3[4], arduino_rx_buffer_usart_3[5]);
//             data_usart_3.packet.yaw_req = MAKE_INT16(arduino_rx_buffer_usart_3[6], arduino_rx_buffer_usart_3[7]);
//             data_usart_3.packet.feeder_motor_pwm = MAKE_INT16(arduino_rx_buffer_usart_3[8], arduino_rx_buffer_usart_3[9]);
//             data_usart_3.packet.friction_motor_pwm = MAKE_INT16(arduino_rx_buffer_usart_3[10], arduino_rx_buffer_usart_3[11]);
//             data_usart_3.packet.drive_req = MAKE_INT16(arduino_rx_buffer_usart_3[12], arduino_rx_buffer_usart_3[13]);
//             data_usart_3.packet.strafe_req = MAKE_INT16(arduino_rx_buffer_usart_3[14], arduino_rx_buffer_usart_3[15]);
//             data_usart_3.packet.rotate_req = MAKE_INT16(arduino_rx_buffer_usart_3[16], arduino_rx_buffer_usart_3[17]);
//             data_usart_3.packet.mpu_x = MAKE_INT16(arduino_rx_buffer_usart_3[20], arduino_rx_buffer_usart_3[21]);
//             data_usart_3.packet.mpu_y = MAKE_INT16(arduino_rx_buffer_usart_3[22], arduino_rx_buffer_usart_3[23]);
//             data_usart_3.packet.mpu_z = MAKE_INT16(arduino_rx_buffer_usart_3[24], arduino_rx_buffer_usart_3[25]);
//             data_usart_3.packet.js_real_chassis_out_power = MAKE_INT16(arduino_rx_buffer_usart_3[26], arduino_rx_buffer_usart_3[27]);

// #if !(DEBUG)
//             // reply with new packet, debug must be false
//             for (int i = 0; i < 32; i++) {
//                 arduino_tx_buffer_usart_3[i] = 0x00;
//             }
//             arduino_tx_buffer_usart_3[2] = feeder_motor_state & 255;
//             arduino_tx_buffer_usart_3[0] = 0xCE;

//             // send the packet
//             for (int i = 0; i < 32; i++) {
//                 USART3_SendChar(arduino_tx_buffer_usart_3[i]);
//             }
// #endif

//             // debug actions
//         }
//     }
// }
