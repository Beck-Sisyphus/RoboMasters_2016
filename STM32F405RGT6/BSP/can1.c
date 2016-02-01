#include "can1.h"

//comment

//VP230---CAN_TX---PA12(CANTX) 
//VP230---CAN_RX---PA11(CANRX) 

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
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
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

static int16_t current_201 = 0;
static int16_t current_202 = 0;
static int16_t CurrTemp_201[CurrTempNUM] = {0};
static int16_t CurrTemp_202[CurrTempNUM] = {0};
static int16_t CurrCnt_201 = 0;
static int16_t CurrCnt_202 = 0;
static int32_t CurrInt_201 =0;
static int32_t CurrInt_202 =0;
volatile unsigned char OverCurr_flag = 0;

void CurrentProtect(void)
{
    unsigned int i;
    unsigned int j;
    int32_t currtemp_201 = 0;
    int32_t currtemp_202 = 0;
    if(Can1_CNT%20 == 0)
    {
        // 201Current protection
        CurrTemp_201[CurrCnt_201] = current_201;
        CurrCnt_201++;
        
        if(CurrCnt_201 >= CurrTempNUM){
            CurrCnt_201 = 0;
        }            
        for(i = 0;i < CurrTempNUM;i++)
        {
            currtemp_201 += CurrTemp_201[i];
            CurrInt_201 = currtemp_201/CurrTempNUM;
        }
        // 202Current protection
        CurrTemp_202[CurrCnt_202] = current_202;
        CurrCnt_202++;
        
        if(CurrCnt_202 >= CurrTempNUM){
            CurrCnt_202 = 0;
        }            
        for(j = 0;j < CurrTempNUM;j++)
        {
            currtemp_202 += CurrTemp_202[j];
            CurrInt_202 = currtemp_202/CurrTempNUM;
        }
    }
        
    if((abs(CurrInt_202) > CurrLimit)||(abs(CurrInt_201) > CurrLimit))
        {
            OverCurr_flag = 1;
        }

}

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
 Description: Receipt of Head Motor's CAN Data is broken
*************************************************************************/
void CAN1_RX0_IRQHandler(void)
{
    CanRxMsg rx_message;    
    
    if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET) 
	{
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
        CAN_Receive(CAN1, CAN_FIFO0, &rx_message);       
                
        if(rx_message.StdId == 0x201)
        {             
             //Acquire Head Motor 0x201's Encoding disk value 获得云台电机0x201的码盘值                 
        }
        if(rx_message.StdId == 0x202)
        { 
             //Acquire Head Motor 0x202's Encoding disk value 获得云台电机0x202的码盘值           
        }		
        if(rx_message.StdId == 0x203)
        { 
             //Acquire Head Motor 0x203's Encoding disk value 获得云台电机0x203的码盘值  
        }
        Can1_CNT++;		
    }
}



/*************************************************************************
                            Motor_Reset
Description：Reset the driver that connected to the main control board 
Input parameter： int Motor_ID     Optional parameter： MOTOR_NUM1   MOTOR_NUM2  
                                       MOTOR_NUM3   MOTOR_NUM4
                                       MOTOR_NUM5   MOTOR_NUM6
                                       MOTOR_NUM7   MOTOR_NUM8
*************************************************************************/
void Motor_Reset(int Motor_ID)
{
    CanTxMsg tx_message;
    tx_message.IDE = CAN_ID_STD;    //Standard Frame
    tx_message.RTR = CAN_RTR_DATA;  //Data Frame
    tx_message.DLC = 0x01;          //Frame length is 1
    tx_message.StdId = 0x00;        //Fram ID is 0x00

    switch (Motor_ID)
    {
        case MOTOR_NUM1: tx_message.Data[0] = ID_MOTOR1_CHOOSE_MODE; break;  //If Motor1 is chosen, Data0 is 0x10
        case MOTOR_NUM2: tx_message.Data[0] = ID_MOTOR2_CHOOSE_MODE; break;  //If Motor2 is chosen, Data0 is 0x20
        case MOTOR_NUM3: tx_message.Data[0] = ID_MOTOR3_CHOOSE_MODE; break;  //If Motor3 is chosen, Data0 is 0x30
        case MOTOR_NUM4: tx_message.Data[0] = ID_MOTOR4_CHOOSE_MODE; break;  //If Motor4 is chosen, Data0 is 0x40
        case MOTOR_NUM5: tx_message.Data[0] = ID_MOTOR5_CHOOSE_MODE; break;  //If Motor5 is chosen, Data0 is 0x50
        case MOTOR_NUM6: tx_message.Data[0] = ID_MOTOR6_CHOOSE_MODE; break;  //If Motor6 is chosen, Data0 is 0x60
        case MOTOR_NUM7: tx_message.Data[0] = ID_MOTOR7_CHOOSE_MODE; break;  //If Motor7 is chosen, Data0 is 0x70
        case MOTOR_NUM8: tx_message.Data[0] = ID_MOTOR8_CHOOSE_MODE; break;  //If Motor8 is chosen, Data0 is 0x80
        default: 
        {
            while(1)      //If the Number of the Motor that been chosen is largger than 8, the buzzer will sound for warning
            {
                delay_ms(200);
                #ifdef __DEBUG
                   // printf("ERROR Motor_Reset Motor_ID choise!\r\n");
                #endif
            }
        }
    }
    
    can_tx_success_flag = 0;
    CAN_Transmit(CAN1,&tx_message);
    while(can_tx_success_flag == 0);
}

/*************************************************************************
                            Motor_Init
Description：Reset the driver that connected to the main control board
Input parameters： int Motor_ID     Optional parameters： MOTOR_NUM1   MOTOR_NUM2  
                                       MOTOR_NUM3   MOTOR_NUM4
                                       MOTOR_NUM5   MOTOR_NUM6
                                       MOTOR_NUM7   MOTOR_NUM8
Input Parameters： int Motor_Mode   Optional parameters： PWM_MODE 
                                       SPEED_MODE 
                                       PWM_LOCATION_MODE 
                                       SPEED_LOCATION_MODE
*************************************************************************/
void Motor_Init(int Motor_ID,int Motor_Mode)
{
    CanTxMsg tx_message;
    
    tx_message.RTR = CAN_RTR_DATA;  //Data Frame
    tx_message.IDE = CAN_ID_STD;    //Standard Frame
    tx_message.DLC = 0x04;          //Frame length is 4     
    
    tx_message.Data[0] = 0x59;
    tx_message.Data[3] = 0xae;
    
    switch (Motor_ID)
    {
        case MOTOR_NUM1: tx_message.StdId = ID_MOTOR1_CHOOSE_MODE; break; //If Motor1 is chosen, Frame ID  is 0x10
        case MOTOR_NUM2: tx_message.StdId = ID_MOTOR2_CHOOSE_MODE; break; //If Motor2 is chosen, Frame ID  is 0x20
        case MOTOR_NUM3: tx_message.StdId = ID_MOTOR3_CHOOSE_MODE; break; //If Motor3 is chosen, Frame ID  is 0x30
        case MOTOR_NUM4: tx_message.StdId = ID_MOTOR4_CHOOSE_MODE; break; //If Motor4 is chosen, Frame ID  is 0x40
        case MOTOR_NUM5: tx_message.StdId = ID_MOTOR5_CHOOSE_MODE; break; //If Motor5 is chosen, Frame ID  is 0x50
        case MOTOR_NUM6: tx_message.StdId = ID_MOTOR6_CHOOSE_MODE; break; //If Motor6 is chosen, Frame ID  is 0x60
        case MOTOR_NUM7: tx_message.StdId = ID_MOTOR7_CHOOSE_MODE; break; //If Motor7 is chosen, Frame ID  is 0x70
        case MOTOR_NUM8: tx_message.StdId = ID_MOTOR8_CHOOSE_MODE; break; //If Motor8 is chosen, Frame ID  is 0x80
        default: 
        {
            while(1)      //If the Number of the Motor that been chosen is largger than 8, the buzzer will sound for warning
            {
                delay_ms(200);
                #ifdef __DEBUG
                //    printf("ERROR Motor_Init Motor_ID choise!\r\n");
                #endif
            }
        }
    }
    
    switch (Motor_Mode)
    {
        case PWM_MODE:              tx_message.Data[1] = 0x56; //If the chosen Mode is PWM_MODE
                                    tx_message.Data[2] = 0xab; //Data1 is 0x56，Data2 is 0xab
                                    break;
        case SPEED_MODE:            tx_message.Data[1] = 0x55; //If the chosen Mode is  SPEED_MODE
                                    tx_message.Data[2] = 0xaa; //Data1 is 0x55，Data2 is 0xaa
                                    break;
        case PWM_LOCATION_MODE:     tx_message.Data[1] = 0x58; //If the chosen Mode is  PWM_LOCATION_MODE
                                    tx_message.Data[2] = 0xad; //Data1 is 0x58，Data2 is 0xad
                                    break;
        case SPEED_LOCATION_MODE:   tx_message.Data[1] = 0x57; //If the chosen Mode is  SPEED_LOCATION_MODE
                                    tx_message.Data[2] = 0xac; //Data1 is 0x57，Data2 is 0xac
                                    break;
        default:
        {
            while(1)      //If the Number of the Motor that been chosen is largger than 4, the buzzer will sound for warning
            {
                delay_ms(200);
                #ifdef __DEBUG
                    printf("ERROR Motor_Init Motor_Mode choise!\r\n");
                #endif
            }
        }
    }

    can_tx_success_flag = 0;
    CAN_Transmit(CAN1,&tx_message);
    while(can_tx_success_flag == 0);
}

/*************************************************************************
                           MOTOR_PWM_Set
Description：The driver set=up under PWM Mode 
Input Parameters： int Motor_ID     Optional Parameters： MOTOR_NUM1   MOTOR_NUM2  
                                       MOTOR_NUM3   MOTOR_NUM4
                                       MOTOR_NUM5   MOTOR_NUM6
                                       MOTOR_NUM7   MOTOR_NUM8
Input Parameters： int Give_PWM   Range -5000 -- 0 -- 5000
*************************************************************************/
void Motor_PWM_Set(int Motor_ID,int Give_PWM)
{
    CanTxMsg tx_message;
    
    tx_message.RTR = CAN_RTR_DATA;  //Data Frame
    tx_message.IDE = CAN_ID_STD;    //Standard Frame
    tx_message.DLC = 0x04;          //Frame length is 4    
  
    switch (Motor_ID)
    {
        case MOTOR_NUM1: tx_message.StdId = ID_MOTOR1_PWM_MODE; break; //If Motor1 is chosen, Frame ID  is 0x11 under PWM Mode
        case MOTOR_NUM2: tx_message.StdId = ID_MOTOR2_PWM_MODE; break; //If Motor2 is chosen, Frame ID  is 0x21 under PWM Mode
        case MOTOR_NUM3: tx_message.StdId = ID_MOTOR3_PWM_MODE; break; //If Motor3 is chosen, Frame ID  is 0x31 under PWM Mode
        case MOTOR_NUM4: tx_message.StdId = ID_MOTOR4_PWM_MODE; break; //If Motor4 is chosen, Frame ID  is 0x41 under PWM Mode
        case MOTOR_NUM5: tx_message.StdId = ID_MOTOR5_PWM_MODE; break; //If Motor5 is chosen, Frame ID  is 0x51 under PWM Mode
        case MOTOR_NUM6: tx_message.StdId = ID_MOTOR6_PWM_MODE; break; //If Motor6 is chosen, Frame ID  is 0x61 under PWM Mode
        case MOTOR_NUM7: tx_message.StdId = ID_MOTOR7_PWM_MODE; break; //If Motor7 is chosen, Frame ID  is 0x71 under PWM Mode
        case MOTOR_NUM8: tx_message.StdId = ID_MOTOR8_PWM_MODE; break; //If Motor8 is chosen, Frame ID  is 0x81 under PWM Mode
        default: 
        {
            while(1)      //If the Number of the Motor that been chosen is largger than 8, the buzzer will sound for warning
            {
                delay_ms(200);
                #ifdef __DEBUG
                   // printf("ERROR Motor_PWM_Set Motor_ID choise!\r\n");
                #endif
            }
        }
    }
    
    if(Give_PWM >= 3500)
    {
        Give_PWM = 3500;
    }
    
    if(Give_PWM <= -3500)
    {
        Give_PWM = -3500;
    }
    
    tx_message.Data[0] = (unsigned char)((Give_PWM >> 24)&0xff);
    tx_message.Data[1] = (unsigned char)((Give_PWM >> 16)&0xff);
    tx_message.Data[2] = (unsigned char)((Give_PWM >> 8)&0xff);
    tx_message.Data[3] = (unsigned char)(Give_PWM&0xff);
    
    can_tx_success_flag = 0;
    CAN_Transmit(CAN1,&tx_message);
    //while(can_tx_success_flag == 0);
}

/*************************************************************************
                           MOTOR_Speed_Set
Description：Driver Set-up under Speed Mode 
Input Parameters： int Motor_ID     Optional Parameters： MOTOR_NUM1   MOTOR_NUM2  
                                       MOTOR_NUM3   MOTOR_NUM4
                                       MOTOR_NUM5   MOTOR_NUM6
                                       MOTOR_NUM7   MOTOR_NUM8
Input Parameters： int Give_Speed  -500 -- 0 -- 500
*************************************************************************/
void Motor_Speed_Set(int Motor_ID,int Give_Speed)
{
    CanTxMsg tx_message;
    
    tx_message.RTR = CAN_RTR_DATA;  //Data Frame
    tx_message.IDE = CAN_ID_STD;    //Standard Frame
    tx_message.DLC = 0x04;          //Frame Length is 4   
  
    switch (Motor_ID)
    {
        case MOTOR_NUM1: tx_message.StdId = ID_MOTOR1_SPEED_MODE; break; //If Motor1 is chosen, Frame ID  is 0x12 under Speed Mode
        case MOTOR_NUM2: tx_message.StdId = ID_MOTOR2_SPEED_MODE; break; //If Motor2 is chosen, Frame ID  is 0x22 under Speed Mode
        case MOTOR_NUM3: tx_message.StdId = ID_MOTOR3_SPEED_MODE; break; //If Motor3 is chosen, Frame ID  is 0x32 under Speed Mode
        case MOTOR_NUM4: tx_message.StdId = ID_MOTOR4_SPEED_MODE; break; //If Motor4 is chosen, Frame ID  is 0x42 under Speed Mode
        case MOTOR_NUM5: tx_message.StdId = ID_MOTOR5_SPEED_MODE; break; //If Motor5 is chosen, Frame ID  is 0x52 under Speed Mode
        case MOTOR_NUM6: tx_message.StdId = ID_MOTOR6_SPEED_MODE; break; //If Motor6 is chosen, Frame ID  is 0x62 under Speed Mode
        case MOTOR_NUM7: tx_message.StdId = ID_MOTOR7_SPEED_MODE; break; //If Motor7 is chosen, Frame ID  is 0x72 under Speed Mode
        case MOTOR_NUM8: tx_message.StdId = ID_MOTOR8_SPEED_MODE; break; //If Motor8 is chosen, Frame ID  is 0x82 under Speed Mode
        default: 
        {
            while(1)      //If the Number of the Motor that been chosen is largger than 8, the buzzer will sound for warning
            {
                delay_ms(200);
                #ifdef __DEBUG
                 //   printf("ERROR Motor_Speed_Set Motor_ID choise!\r\n");
                #endif
            }
        }
    }
    
    if(Give_Speed >= 500)
    {
        Give_Speed = 500;
    }
    
    if(Give_Speed <= -500)
    {
        Give_Speed = -500;
    }
    
    tx_message.Data[0] = (unsigned char)((Give_Speed >> 24)&0xff);
    tx_message.Data[1] = (unsigned char)((Give_Speed >> 16)&0xff);
    tx_message.Data[2] = (unsigned char)((Give_Speed >> 8)&0xff);
    tx_message.Data[3] = (unsigned char)(Give_Speed&0xff);
    
    can_tx_success_flag = 0;
    CAN_Transmit(CAN1,&tx_message);
    while(can_tx_success_flag == 0);   
}

/*************************************************************************
                         MOTOR_PWM_Location_Set
Description：在基于PWM的位置环模式下驱动器的设置
Description: Driver Set-up under PWM
Input Parameters： int Motor_ID     Optional Parameters： MOTOR_NUM1   MOTOR_NUM2  
                                       MOTOR_NUM3   MOTOR_NUM4
                                       MOTOR_NUM5   MOTOR_NUM6
                                       MOTOR_NUM7   MOTOR_NUM8
Input parameters： int Give_PWM      0 -- 5000
Input parameters： int Give_PWM_Location  
*************************************************************************/
void Motor_PWM_Location_Set(int Motor_ID,int Give_PWM,int Give_PWM_Location)
{
    CanTxMsg tx_message;
    
    tx_message.RTR = CAN_RTR_DATA;  //Data Frame
    tx_message.IDE = CAN_ID_STD;    //Standard Frame
    tx_message.DLC = 0x08;          //Frame length is 8   
  
    switch (Motor_ID)
    {
        case MOTOR_NUM1: tx_message.StdId = ID_MOTOR1_PWM_LOCATION_MODE; break; //If Motor1 is chosen, Frame ID  is 0x13 under PWM_LOCATION Mode
        case MOTOR_NUM2: tx_message.StdId = ID_MOTOR2_PWM_LOCATION_MODE; break; //If Motor2 is chosen, Frame ID  is 0x23 under PWM_LOCATION Mode
        case MOTOR_NUM3: tx_message.StdId = ID_MOTOR3_PWM_LOCATION_MODE; break; //If Motor3 is chosen, Frame ID  is 0x33 under PWM_LOCATION Mode
        case MOTOR_NUM4: tx_message.StdId = ID_MOTOR4_PWM_LOCATION_MODE; break; //If Motor4 is chosen, Frame ID  is 0x43 under PWM_LOCATION Mode
        case MOTOR_NUM5: tx_message.StdId = ID_MOTOR5_PWM_LOCATION_MODE; break; //If Motor5 is chosen, Frame ID  is 0x53 under PWM_LOCATION Mode
        case MOTOR_NUM6: tx_message.StdId = ID_MOTOR6_PWM_LOCATION_MODE; break; //If Motor6 is chosen, Frame ID  is 0x63 under PWM_LOCATION Mode
        case MOTOR_NUM7: tx_message.StdId = ID_MOTOR7_PWM_LOCATION_MODE; break; //If Motor7 is chosen, Frame ID  is 0x73 under PWM_LOCATION Mode
        case MOTOR_NUM8: tx_message.StdId = ID_MOTOR8_PWM_LOCATION_MODE; break; //If Motor8 is chosen, Frame ID  is 0x83 under PWM_LOCATION Mode    
        default: 
        {
            while(1)      //If the Number of the Motor that been chosen is largger than 8, the buzzer will sound for warning
            {
                delay_ms(200);
                #ifdef __DEBUG
                 //   printf("ERROR Motor_PWM_Location_Set Motor_ID choise!\r\n");
                #endif
            }
        }
    }
    
    Give_PWM = abs(Give_PWM);
    
    if(Give_PWM >= 5000)
    {
        Give_PWM = 5000;
    }
    
    tx_message.Data[0] = (unsigned char)((Give_PWM >> 24)&0xff);
    tx_message.Data[1] = (unsigned char)((Give_PWM >> 16)&0xff);
    tx_message.Data[2] = (unsigned char)((Give_PWM >> 8)&0xff);
    tx_message.Data[3] = (unsigned char)(Give_PWM&0xff);
    tx_message.Data[4] = (unsigned char)((Give_PWM_Location >> 24)&0xff);
    tx_message.Data[5] = (unsigned char)((Give_PWM_Location >> 16)&0xff);
    tx_message.Data[6] = (unsigned char)((Give_PWM_Location >> 8)&0xff);
    tx_message.Data[7] = (unsigned char)(Give_PWM_Location&0xff); 
    
    can_tx_success_flag = 0;
    CAN_Transmit(CAN1,&tx_message);
    //while(can_tx_success_flag == 0);   
}

/*************************************************************************
                        Motor_Speed_Location_Set
描述：在基于Speed的位置环模式下驱动器的设置
Input Parameters： int Motor_ID    Optional Parameters： MOTOR_NUM1   MOTOR_NUM2  
                                       MOTOR_NUM3   MOTOR_NUM4
                                       MOTOR_NUM5   MOTOR_NUM6
                                       MOTOR_NUM7   MOTOR_NUM8
Input Parameters： int Give_Speed   0 - 500
Input Parameters： int Give_Speed_Location 
*************************************************************************/
void Motor_Speed_Location_Set(int Motor_ID,int Give_Speed,int Give_Speed_Location)
{
    CanTxMsg tx_message;
    
    tx_message.RTR = CAN_RTR_DATA;  //Data Frame
    tx_message.IDE = CAN_ID_STD;    //Standard Frame
    tx_message.DLC = 0x08;          //Frame Legnth is 8
  
    switch (Motor_ID)
    {
        case MOTOR_NUM1: tx_message.StdId = ID_MOTOR1_SPEED_LOCATION_MODE; break; //If Motor1 is chosen, Frame ID  is 0x14 under Speed_LOCATION Mode
        case MOTOR_NUM2: tx_message.StdId = ID_MOTOR2_SPEED_LOCATION_MODE; break; //If Motor2 is chosen, Frame ID  is 0x24 under Speed_LOCATION Mode
        case MOTOR_NUM3: tx_message.StdId = ID_MOTOR3_SPEED_LOCATION_MODE; break; //If Motor3 is chosen, Frame ID  is 0x34 under Speed_LOCATION Mode
        case MOTOR_NUM4: tx_message.StdId = ID_MOTOR4_SPEED_LOCATION_MODE; break; //If Motor4 is chosen, Frame ID  is 0x44 under Speed_LOCATION Mode
        case MOTOR_NUM5: tx_message.StdId = ID_MOTOR5_SPEED_LOCATION_MODE; break; //If Motor5 is chosen, Frame ID  is 0x54 under Speed_LOCATION Mode
        case MOTOR_NUM6: tx_message.StdId = ID_MOTOR6_SPEED_LOCATION_MODE; break; //If Motor6 is chosen, Frame ID  is 0x64 under Speed_LOCATION Mode
        case MOTOR_NUM7: tx_message.StdId = ID_MOTOR7_SPEED_LOCATION_MODE; break; //If Motor7 is chosen, Frame ID  is 0x74 under Speed_LOCATION Mode
        case MOTOR_NUM8: tx_message.StdId = ID_MOTOR8_SPEED_LOCATION_MODE; break; //If Motor8 is chosen, Frame ID  is 0x84 under Speed_LOCATION Mode
        default: 
        {
            while(1)      //If the Number of the Motor that been chosen is largger than 8, the buzzer will sound for warning
            {
                delay_ms(200);
                #ifdef __DEBUG
                 //   printf("ERROR Motor_Speed_Location_Set Motor_ID choise!\r\n");
                #endif
            }
        }
    }
    
    Give_Speed = abs(Give_Speed);
    
    if(Give_Speed >= 500)
    {
        Give_Speed = 5000;
    }
    
    tx_message.Data[0] = (unsigned char)((Give_Speed >> 24)&0xff);
    tx_message.Data[1] = (unsigned char)((Give_Speed >> 16)&0xff);
    tx_message.Data[2] = (unsigned char)((Give_Speed >> 8)&0xff);
    tx_message.Data[3] = (unsigned char)(Give_Speed&0xff);
    tx_message.Data[4] = (unsigned char)((Give_Speed_Location >> 24)&0xff);
    tx_message.Data[5] = (unsigned char)((Give_Speed_Location >> 16)&0xff);
    tx_message.Data[6] = (unsigned char)((Give_Speed_Location >> 8)&0xff);
    tx_message.Data[7] = (unsigned char)(Give_Speed_Location&0xff); 
    
    can_tx_success_flag = 0;
    CAN_Transmit(CAN1,&tx_message);
    while(can_tx_success_flag == 0);   
}






