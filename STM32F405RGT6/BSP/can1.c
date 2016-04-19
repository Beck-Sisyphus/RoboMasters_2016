#include "can1.h"

/*----CAN1_TX-----PA12----*/
/*----CAN1_RX-----PA11----*/

/*************************************************************************
                          CAN1_Configuration
Description：Initiate CAN1 configuration to be 1M Baud 初始化CAN1配置为1M波特率
*************************************************************************/
void CAN1_Configuration(void)
{
    CAN_InitTypeDef        can;
    CAN_FilterInitTypeDef  can_filter;
    GPIO_InitTypeDef       gpio;
    NVIC_InitTypeDef       nvic;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_CAN1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_CAN1);

    gpio.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    GPIO_Init(GPIOA, &gpio);

    nvic.NVIC_IRQChannel = CAN1_RX0_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    nvic.NVIC_IRQChannel = CAN1_TX_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    CAN_DeInit(CAN1);
    CAN_StructInit(&can);

    can.CAN_TTCM = DISABLE;
    can.CAN_ABOM = DISABLE;
    can.CAN_AWUM = DISABLE;
    can.CAN_NART = DISABLE;
    can.CAN_RFLM = DISABLE;
    can.CAN_TXFP = ENABLE;
    can.CAN_Mode = CAN_Mode_Normal;
    can.CAN_SJW  = CAN_SJW_1tq;
    can.CAN_BS1 = CAN_BS1_9tq;
    can.CAN_BS2 = CAN_BS2_4tq;
    can.CAN_Prescaler = 3;   //CAN BaudRate 42/(1+9+4)/3=1Mbps
    CAN_Init(CAN1, &can);

    can_filter.CAN_FilterNumber=0;
    can_filter.CAN_FilterMode=CAN_FilterMode_IdMask;
    can_filter.CAN_FilterScale=CAN_FilterScale_32bit;
    can_filter.CAN_FilterIdHigh=0x0000;
    can_filter.CAN_FilterIdLow=0x0000;
    can_filter.CAN_FilterMaskIdHigh=0x0000;
    can_filter.CAN_FilterMaskIdLow=0x0000;
    can_filter.CAN_FilterFIFOAssignment=0;//the message which pass the filter save in fifo0
    can_filter.CAN_FilterActivation=ENABLE;
    CAN_FilterInit(&can_filter);

    CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);
    CAN_ITConfig(CAN1,CAN_IT_TME,ENABLE);
}


//If Break, enter this function one time per minute 如果中断开启，则1ms进入该函数一次
#define CurrTempNUM 100 //integraled time period 积分的时间周期
#define CurrLimit 13000 //Current limited to 12000 for 6025

static uint16_t Can1_CNT = 0;


int32_t measured_yaw_angle_401 = 0;
int32_t last_yaw_angle = 0;
int32_t this_yaw_angle = 0;


unsigned char can_tx_success_flag=0;
void CAN1_TX_IRQHandler(void) //CAN TX
{
    if (CAN_GetITStatus(CAN1,CAN_IT_TME)!= RESET)
	{
	   CAN_ClearITPendingBit(CAN1,CAN_IT_TME);
       can_tx_success_flag=1;
    }
}

/*************************************************************************
                          CAN1_RX0_IRQHandler
Description: Interrupt receive for chassis CAN bus data
*************************************************************************/
void CAN1_RX0_IRQHandler(void)
{
    CanRxMsg rx_message;

    if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET)
	  {
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
        CAN_Receive(CAN1, CAN_FIFO0, &rx_message);
        //Single axis gyroscope data 单轴陀螺仪数据
        if(rx_message.StdId == 0x401)
        {
          measured_yaw_angle_401 = (int32_t)(rx_message.Data[0]<<24)|(int32_t)(rx_message.Data[1]<<16)
          | (int32_t)(rx_message.Data[2]<<8) | (int32_t)(rx_message.Data[3]);

          last_yaw_angle = this_yaw_angle;
          this_yaw_angle = -((float)measured_yaw_angle_401*0.01);
        }
    }
}
