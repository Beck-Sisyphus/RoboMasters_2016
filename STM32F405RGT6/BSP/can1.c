#include "can1.h"

//VP230---CAN_TX---PA12(CANTX) 
//VP230---CAN_RX---PA11(CANRX) 

/*************************************************************************
                          CAN1_Configuration
��������ʼ��CAN1����Ϊ1M������
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


//����жϿ�������1ms����ú���һ��
#define CurrTempNUM 100 //���ֵ�ʱ������
#define CurrLimit 13000 //��������12000 for 6025

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
        // 201��������
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
        // 202��������
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
��������̨�����CAN���ݽ����ж�
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
             //�����̨���0x201������ֵ                 
        }
        if(rx_message.StdId == 0x202)
        { 
             //�����̨���0x202������ֵ           
        }		
        if(rx_message.StdId == 0x203)
        { 
             //�����̨���0x203������ֵ  
        }
        Can1_CNT++;		
    }
}



/*************************************************************************
                            Motor_Reset
���������ҽ������ذ��ϵ���������λ
��������� int Motor_ID     ��ѡ������ MOTOR_NUM1   MOTOR_NUM2  
                                       MOTOR_NUM3   MOTOR_NUM4
                                       MOTOR_NUM5   MOTOR_NUM6
                                       MOTOR_NUM7   MOTOR_NUM8
*************************************************************************/
void Motor_Reset(int Motor_ID)
{
    CanTxMsg tx_message;
    tx_message.IDE = CAN_ID_STD;    //��׼֡
    tx_message.RTR = CAN_RTR_DATA;  //����֡
    tx_message.DLC = 0x01;          //֡����Ϊ1
    tx_message.StdId = 0x00;        //֡IDΪ0x00

    switch (Motor_ID)
    {
        case MOTOR_NUM1: tx_message.Data[0] = ID_MOTOR1_CHOOSE_MODE; break;  //���ѡ���˵��1: ����0Ϊ0x10
        case MOTOR_NUM2: tx_message.Data[0] = ID_MOTOR2_CHOOSE_MODE; break;  //���ѡ���˵��2: ����0Ϊ0x20
        case MOTOR_NUM3: tx_message.Data[0] = ID_MOTOR3_CHOOSE_MODE; break;  //���ѡ���˵��3: ����0Ϊ0x30
        case MOTOR_NUM4: tx_message.Data[0] = ID_MOTOR4_CHOOSE_MODE; break;  //���ѡ���˵��4: ����0Ϊ0x40
        case MOTOR_NUM5: tx_message.Data[0] = ID_MOTOR5_CHOOSE_MODE; break;  //���ѡ���˵��5: ����0Ϊ0x50
        case MOTOR_NUM6: tx_message.Data[0] = ID_MOTOR6_CHOOSE_MODE; break;  //���ѡ���˵��6: ����0Ϊ0x60
        case MOTOR_NUM7: tx_message.Data[0] = ID_MOTOR7_CHOOSE_MODE; break;  //���ѡ���˵��7: ����0Ϊ0x70
        case MOTOR_NUM8: tx_message.Data[0] = ID_MOTOR8_CHOOSE_MODE; break;  //���ѡ���˵��8: ����0Ϊ0x80
        default: 
        {
            while(1)      //���ѡ�ĵ���ķ�Χ������8���ͷ���������
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
���������ҽ������ذ��ϵ���������ʼ��
��������� int Motor_ID     ��ѡ������ MOTOR_NUM1   MOTOR_NUM2  
                                       MOTOR_NUM3   MOTOR_NUM4
                                       MOTOR_NUM5   MOTOR_NUM6
                                       MOTOR_NUM7   MOTOR_NUM8
��������� int Motor_Mode   ��ѡ������ PWM_MODE 
                                       SPEED_MODE 
                                       PWM_LOCATION_MODE 
                                       SPEED_LOCATION_MODE
*************************************************************************/
void Motor_Init(int Motor_ID,int Motor_Mode)
{
    CanTxMsg tx_message;
    
    tx_message.RTR = CAN_RTR_DATA;  //����֡
    tx_message.IDE = CAN_ID_STD;    //��׼֡
    tx_message.DLC = 0x04;          //֡����Ϊ4      
    
    tx_message.Data[0] = 0x59;
    tx_message.Data[3] = 0xae;
    
    switch (Motor_ID)
    {
        case MOTOR_NUM1: tx_message.StdId = ID_MOTOR1_CHOOSE_MODE; break; //���ѡ�����1: ֡IDΪ0x10
        case MOTOR_NUM2: tx_message.StdId = ID_MOTOR2_CHOOSE_MODE; break; //���ѡ�����2: ֡IDΪ0x20
        case MOTOR_NUM3: tx_message.StdId = ID_MOTOR3_CHOOSE_MODE; break; //���ѡ�����3: ֡IDΪ0x30
        case MOTOR_NUM4: tx_message.StdId = ID_MOTOR4_CHOOSE_MODE; break; //���ѡ�����4: ֡IDΪ0x40
        case MOTOR_NUM5: tx_message.StdId = ID_MOTOR5_CHOOSE_MODE; break; //���ѡ�����5: ֡IDΪ0x50
        case MOTOR_NUM6: tx_message.StdId = ID_MOTOR6_CHOOSE_MODE; break; //���ѡ�����6: ֡IDΪ0x60
        case MOTOR_NUM7: tx_message.StdId = ID_MOTOR7_CHOOSE_MODE; break; //���ѡ�����7: ֡IDΪ0x70
        case MOTOR_NUM8: tx_message.StdId = ID_MOTOR8_CHOOSE_MODE; break; //���ѡ�����8: ֡IDΪ0x80
        default: 
        {
            while(1)      //���ѡ�ĵ���ķ�Χ������8���ͷ���������
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
        case PWM_MODE:              tx_message.Data[1] = 0x56; //���ѡ����ģʽ�� PWM_MODE
                                    tx_message.Data[2] = 0xab; //����1Ϊ0x56������2Ϊ0xab
                                    break;
        case SPEED_MODE:            tx_message.Data[1] = 0x55; //���ѡ����ģʽ�� SPEED_MODE
                                    tx_message.Data[2] = 0xaa; //����1Ϊ0x55������2Ϊ0xaa
                                    break;
        case PWM_LOCATION_MODE:     tx_message.Data[1] = 0x58; //���ѡ����ģʽ�� PWM_LOCATION_MODE
                                    tx_message.Data[2] = 0xad; //����1Ϊ0x58������2Ϊ0xad
                                    break;
        case SPEED_LOCATION_MODE:   tx_message.Data[1] = 0x57; //���ѡ����ģʽ�� SPEED_LOCATION_MODE
                                    tx_message.Data[2] = 0xac; //����1Ϊ0x57������2Ϊ0xac
                                    break;
        default:
        {
            while(1)      //���ѡ�ĵ��ģʽ�ķ�Χ������4���ͷ���������
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
������PWMģʽ��������������
��������� int Motor_ID     ��ѡ������ MOTOR_NUM1   MOTOR_NUM2  
                                       MOTOR_NUM3   MOTOR_NUM4
                                       MOTOR_NUM5   MOTOR_NUM6
                                       MOTOR_NUM7   MOTOR_NUM8
��������� int Give_PWM   ��Χ -5000 -- 0 -- 5000
*************************************************************************/
void Motor_PWM_Set(int Motor_ID,int Give_PWM)
{
    CanTxMsg tx_message;
    
    tx_message.RTR = CAN_RTR_DATA;  //����֡
    tx_message.IDE = CAN_ID_STD;    //��׼֡
    tx_message.DLC = 0x04;          //֡����Ϊ4    
  
    switch (Motor_ID)
    {
        case MOTOR_NUM1: tx_message.StdId = ID_MOTOR1_PWM_MODE; break; //���ѡ�����1��PWMģʽ��֡IDΪ0x11
        case MOTOR_NUM2: tx_message.StdId = ID_MOTOR2_PWM_MODE; break; //���ѡ�����2��PWMģʽ��֡IDΪ0x21
        case MOTOR_NUM3: tx_message.StdId = ID_MOTOR3_PWM_MODE; break; //���ѡ�����3��PWMģʽ��֡IDΪ0x31
        case MOTOR_NUM4: tx_message.StdId = ID_MOTOR4_PWM_MODE; break; //���ѡ�����4��PWMģʽ��֡IDΪ0x41
        case MOTOR_NUM5: tx_message.StdId = ID_MOTOR5_PWM_MODE; break; //���ѡ�����5��PWMģʽ��֡IDΪ0x51
        case MOTOR_NUM6: tx_message.StdId = ID_MOTOR6_PWM_MODE; break; //���ѡ�����6��PWMģʽ��֡IDΪ0x61
        case MOTOR_NUM7: tx_message.StdId = ID_MOTOR7_PWM_MODE; break; //���ѡ�����7��PWMģʽ��֡IDΪ0x71
        case MOTOR_NUM8: tx_message.StdId = ID_MOTOR8_PWM_MODE; break; //���ѡ�����8��PWMģʽ��֡IDΪ0x81
        default: 
        {
            while(1)      //���ѡ�ĵ���ķ�Χ������8���ͷ���������
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
������Speedģʽ��������������
��������� int Motor_ID     ��ѡ������ MOTOR_NUM1   MOTOR_NUM2  
                                       MOTOR_NUM3   MOTOR_NUM4
                                       MOTOR_NUM5   MOTOR_NUM6
                                       MOTOR_NUM7   MOTOR_NUM8
��������� int Give_Speed  -500 -- 0 -- 500
*************************************************************************/
void Motor_Speed_Set(int Motor_ID,int Give_Speed)
{
    CanTxMsg tx_message;
    
    tx_message.RTR = CAN_RTR_DATA;  //����֡
    tx_message.IDE = CAN_ID_STD;    //��׼֡
    tx_message.DLC = 0x04;          //֡����Ϊ4    
  
    switch (Motor_ID)
    {
        case MOTOR_NUM1: tx_message.StdId = ID_MOTOR1_SPEED_MODE; break; //���ѡ�����1��Speedģʽ��֡IDΪ0x12
        case MOTOR_NUM2: tx_message.StdId = ID_MOTOR2_SPEED_MODE; break; //���ѡ�����2��Speedģʽ��֡IDΪ0x22
        case MOTOR_NUM3: tx_message.StdId = ID_MOTOR3_SPEED_MODE; break; //���ѡ�����3��Speedģʽ��֡IDΪ0x32
        case MOTOR_NUM4: tx_message.StdId = ID_MOTOR4_SPEED_MODE; break; //���ѡ�����4��Speedģʽ��֡IDΪ0x42
        case MOTOR_NUM5: tx_message.StdId = ID_MOTOR5_SPEED_MODE; break; //���ѡ�����5��Speedģʽ��֡IDΪ0x52
        case MOTOR_NUM6: tx_message.StdId = ID_MOTOR6_SPEED_MODE; break; //���ѡ�����6��Speedģʽ��֡IDΪ0x62
        case MOTOR_NUM7: tx_message.StdId = ID_MOTOR7_SPEED_MODE; break; //���ѡ�����7��Speedģʽ��֡IDΪ0x72
        case MOTOR_NUM8: tx_message.StdId = ID_MOTOR8_SPEED_MODE; break; //���ѡ�����8��Speedģʽ��֡IDΪ0x82
        default: 
        {
            while(1)      //���ѡ�ĵ���ķ�Χ������8���ͷ���������
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
�������ڻ���PWM��λ�û�ģʽ��������������
��������� int Motor_ID     ��ѡ������ MOTOR_NUM1   MOTOR_NUM2  
                                       MOTOR_NUM3   MOTOR_NUM4
                                       MOTOR_NUM5   MOTOR_NUM6
                                       MOTOR_NUM7   MOTOR_NUM8
��������� int Give_PWM      0 -- 5000
��������� int Give_PWM_Location  
*************************************************************************/
void Motor_PWM_Location_Set(int Motor_ID,int Give_PWM,int Give_PWM_Location)
{
    CanTxMsg tx_message;
    
    tx_message.RTR = CAN_RTR_DATA;  //����֡
    tx_message.IDE = CAN_ID_STD;    //��׼֡
    tx_message.DLC = 0x08;          //֡����Ϊ4    
  
    switch (Motor_ID)
    {
        case MOTOR_NUM1: tx_message.StdId = ID_MOTOR1_PWM_LOCATION_MODE; break; //���ѡ�����1��PWM_LOCATIONģʽ��֡IDΪ0x13
        case MOTOR_NUM2: tx_message.StdId = ID_MOTOR2_PWM_LOCATION_MODE; break; //���ѡ�����2��PWM_LOCATIONģʽ��֡IDΪ0x23
        case MOTOR_NUM3: tx_message.StdId = ID_MOTOR3_PWM_LOCATION_MODE; break; //���ѡ�����3��PWM_LOCATIONģʽ��֡IDΪ0x33
        case MOTOR_NUM4: tx_message.StdId = ID_MOTOR4_PWM_LOCATION_MODE; break; //���ѡ�����4��PWM_LOCATIONģʽ��֡IDΪ0x43
        case MOTOR_NUM5: tx_message.StdId = ID_MOTOR5_PWM_LOCATION_MODE; break; //���ѡ�����5��PWM_LOCATIONģʽ��֡IDΪ0x53
        case MOTOR_NUM6: tx_message.StdId = ID_MOTOR6_PWM_LOCATION_MODE; break; //���ѡ�����6��PWM_LOCATIONģʽ��֡IDΪ0x63
        case MOTOR_NUM7: tx_message.StdId = ID_MOTOR7_PWM_LOCATION_MODE; break; //���ѡ�����7��PWM_LOCATIONģʽ��֡IDΪ0x73
        case MOTOR_NUM8: tx_message.StdId = ID_MOTOR8_PWM_LOCATION_MODE; break; //���ѡ�����8��PWM_LOCATIONģʽ��֡IDΪ0x83     
        default: 
        {
            while(1)      //���ѡ�ĵ���ķ�Χ������8���ͷ���������
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
�������ڻ���Speed��λ�û�ģʽ��������������
��������� int Motor_ID     ��ѡ������ MOTOR_NUM1   MOTOR_NUM2  
                                       MOTOR_NUM3   MOTOR_NUM4
                                       MOTOR_NUM5   MOTOR_NUM6
                                       MOTOR_NUM7   MOTOR_NUM8
��������� int Give_Speed   0 - 500
��������� int Give_Speed_Location 
*************************************************************************/
void Motor_Speed_Location_Set(int Motor_ID,int Give_Speed,int Give_Speed_Location)
{
    CanTxMsg tx_message;
    
    tx_message.RTR = CAN_RTR_DATA;  //����֡
    tx_message.IDE = CAN_ID_STD;    //��׼֡
    tx_message.DLC = 0x08;          //֡����Ϊ8 
  
    switch (Motor_ID)
    {
        case MOTOR_NUM1: tx_message.StdId = ID_MOTOR1_SPEED_LOCATION_MODE; break; //��ѡ�����1��Speed_LOCATIONģʽ֡ID=0x14
        case MOTOR_NUM2: tx_message.StdId = ID_MOTOR2_SPEED_LOCATION_MODE; break; //��ѡ�����2��Speed_LOCATIONģʽ֡ID=0x24
        case MOTOR_NUM3: tx_message.StdId = ID_MOTOR3_SPEED_LOCATION_MODE; break; //��ѡ�����3��Speed_LOCATIONģʽ֡ID=0x34
        case MOTOR_NUM4: tx_message.StdId = ID_MOTOR4_SPEED_LOCATION_MODE; break; //��ѡ�����4��Speed_LOCATIONģʽ֡ID=0x44
        case MOTOR_NUM5: tx_message.StdId = ID_MOTOR5_SPEED_LOCATION_MODE; break; //��ѡ�����5��Speed_LOCATIONģʽ֡ID=0x54
        case MOTOR_NUM6: tx_message.StdId = ID_MOTOR6_SPEED_LOCATION_MODE; break; //��ѡ�����6��Speed_LOCATIONģʽ֡ID=0x64
        case MOTOR_NUM7: tx_message.StdId = ID_MOTOR7_SPEED_LOCATION_MODE; break; //��ѡ�����7��Speed_LOCATIONģʽ֡ID=0x74
        case MOTOR_NUM8: tx_message.StdId = ID_MOTOR8_SPEED_LOCATION_MODE; break; //��ѡ�����8��Speed_LOCATIONģʽ֡ID=0x84   
        default: 
        {
            while(1)      //���ѡ�ĵ���ķ�Χ������8���ͷ���������
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






