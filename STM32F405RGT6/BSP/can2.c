#include "can2.h"
#include "led.h"


/*----CAN2_TX-----PB13----*/
/*----CAN2_RX-----PB12----*/

void CAN2_Configuration(void)
{
    CAN_InitTypeDef        can;
    CAN_FilterInitTypeDef  can_filter;
    GPIO_InitTypeDef       gpio;
    NVIC_InitTypeDef       nvic;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_CAN2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_CAN2); 

    gpio.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 ;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    GPIO_Init(GPIOB, &gpio);

    nvic.NVIC_IRQChannel = CAN2_RX0_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    CAN_DeInit(CAN2);
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
    CAN_Init(CAN2, &can);
    
    can_filter.CAN_FilterNumber=14;
    can_filter.CAN_FilterMode=CAN_FilterMode_IdMask;
    can_filter.CAN_FilterScale=CAN_FilterScale_32bit;
    can_filter.CAN_FilterIdHigh=0x0000;
    can_filter.CAN_FilterIdLow=0x0000;
    can_filter.CAN_FilterMaskIdHigh=0x0000;
    can_filter.CAN_FilterMaskIdLow=0x0000;
    can_filter.CAN_FilterFIFOAssignment=0;//the message which pass the filter save in fifo0
    can_filter.CAN_FilterActivation=ENABLE;
    CAN_FilterInit(&can_filter);
    
    CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);
}

void GYRO_RST(void)
{
    CanTxMsg tx_message;
    
    tx_message.StdId = 0x404;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    
    tx_message.Data[0] = 0x00;
    tx_message.Data[1] = 0x01;
    tx_message.Data[2] = 0x02;
    tx_message.Data[3] = 0x03;
    tx_message.Data[4] = 0x04;
    tx_message.Data[5] = 0x05;
    tx_message.Data[6] = 0x06;
    tx_message.Data[7] = 0x07;
    
    CAN_Transmit(CAN2,&tx_message);
}

void Encoder_sent(float encoder_angle)
{
    CanTxMsg tx_message;
    
    tx_message.StdId = 0x601;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    
    encoder_angle = encoder_angle * 100.0; 
    tx_message.Data[0] = (uint8_t)((int32_t)encoder_angle >>24);
    tx_message.Data[1] = (uint8_t)((int32_t)encoder_angle >>16);
    tx_message.Data[2] = (uint8_t)((int32_t)encoder_angle >>8);
    tx_message.Data[3] = (uint8_t)((int32_t)encoder_angle);
    tx_message.Data[4] = 0x00;
    tx_message.Data[5] = 0x00;
    tx_message.Data[6] = 0x00;
    tx_message.Data[7] = 0x00;
    
    CAN_Transmit(CAN2,&tx_message);
}

void Radio_Sent(const uint16_t * radio_channel)
{
    CanTxMsg tx_message;
    
    tx_message.StdId = 0x402;
    tx_message.DLC = 0x08;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.IDE = CAN_Id_Standard;
    
    tx_message.Data[0] = (uint8_t)(*(radio_channel+2)>>8);
    tx_message.Data[1] = (uint8_t)(*(radio_channel+2));
    tx_message.Data[2] = (uint8_t)(*(radio_channel+6)>>8);
    tx_message.Data[3] = (uint8_t)(*(radio_channel+6));
    tx_message.Data[4] = (uint8_t)(*(radio_channel+5)>>8);
    tx_message.Data[5] = (uint8_t)(*(radio_channel+5));
    tx_message.Data[6] = (uint8_t)(*(radio_channel+7)>>8);
    tx_message.Data[7] = (uint8_t)(*(radio_channel+7));
    
    CAN_Transmit(CAN2,&tx_message);
}


int8_t gyro_ok_flag = 0;

// float YAW_Angle;
// float PITCH_Angle;

float this_yaw_angle;
float last_yaw_angle;
// int32_t turn_cnt = 0;
// float dipan_gyro_angle = 0.0;
// int32_t temp_dipan_gyro = 0;

float temp_pitch = 0;
float temp_yaw = 0;
uint8_t shooting_flag = 0;
uint8_t mode_flag=0;
uint8_t ShootFlag=0;

float target_pitch_angle;
float target_yaw_angle;



// yaw and pitch angle rx messages from CAN
uint16_t temp_yaw_angle;
uint16_t temp_pitch_angle;

// yaw and pitch current rx messages from CAN
uint16_t temp_yaw_current;
uint16_t temp_pitch_current;

/*************************************************************************
                          CAN2_RX0_IRQHandler
Description: Interrupt receive for chassis CAN bus data and single axis gyroscope
描述：单轴陀螺仪、底盘主控CAN数据接收中断
*************************************************************************/
void CAN2_RX0_IRQHandler(void)
{
    CanRxMsg rx_message;
    if (CAN_GetITStatus(CAN2,CAN_IT_FMP0)!= RESET) 
    {
       CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
       CAN_Receive(CAN2, CAN_FIFO0, &rx_message);

        /************ YAW ************/
        // Yaw angle range is: [0, around 4770]
        // 0 is right-most yaw position
        // around 4770 is left-most yaw position
        // data 0 and 1 measure angle

        // data 4 and 5 measures what current you are tx to motors 
        // data 4 and 5 not same as current tx value
        // -1000 current value = 27852 (yaw_data4)<<8|(yaw_data5) value
        // -750 current value = 24305 (yaw_data4)<<8|(yaw_data5) value
        // -500 current value = 16630 (yaw_data4)<<8|(yaw_data5) value
        // pitch has same current to (data4<<8)|(data5) conversion
        if(rx_message.StdId == 0x205)
        { 

            // construct message from data[0] and data[1]
            uint16_t yaw_data0 = rx_message.Data[0] << 8;
            uint16_t yaw_data1 = rx_message.Data[1];
            uint16_t yaw_data4 = rx_message.Data[4];
            uint16_t yaw_data5 = rx_message.Data[5];

            temp_yaw_angle = (yaw_data0)|(yaw_data1);
            temp_yaw_current = (yaw_data4)<<8|(yaw_data5);




            // normalize angle range since default angle range is werid
            if(temp_yaw_angle > 6060 && temp_yaw_angle < 8200) {
                temp_yaw_angle = temp_yaw_angle - 6060;
            } else {
                temp_yaw_angle = temp_yaw_angle + 2130;
            }

            // Testing
            // if(temp_yaw_angle > 1000) {
            //     LED1_ON(); // red
            // }  else {
            //     LED1_OFF();
            // }  


        }

        /************ PITCH ************/
        // pitch angle range is [around 3520, around 4500]
        // around 3520 is highest pitch position
        // around 4500 is lowest pitch position  
        // data 0 and 1 measure angle

        // data 4 and 5 measures what current you are tx to motors 
        // data 4 and 5 not same as current tx value
        // -1000 current value = 27852 (pitch_data4)<<8|(pitch_data5) value
        // -750 current value = 24305 (pitch_data4)<<8|(pitch_data5) value
        // -500 current value = 16630 (pitch_data4)<<8|(pitch_data5) value
        // yaw has same current to (data4<<8)|(data5) conversion
        if(rx_message.StdId == 0x206)
        { 
            // construct message from data[0] and data[1]
            uint16_t pitch_data0 = rx_message.Data[0] << 8;
            uint16_t pitch_data1 = rx_message.Data[1];
            uint16_t pitch_data4 = rx_message.Data[4];
            uint16_t pitch_data5 = rx_message.Data[5];

            temp_pitch_angle = (pitch_data0)|(pitch_data1);

            temp_pitch_current = (pitch_data4)<<8|(pitch_data5);



            // Testing
            // if(temp_pitch_angle > 3750) {
            //     LED2_ON(); // green
            // }  else {
            //     LED2_OFF();;
            // }

        }


        
        // Rest of IRQHandler is provided old code I didn't use     
        // Beck: Checked with old control diagram, these are the control data 
        //       that are no longer used
        //Single axis gyroscope data 单轴陀螺仪数据
        if(rx_message.StdId == 0x401)
        { 
            gyro_ok_flag = 1;
            temp_yaw_angle = (int32_t)(rx_message.Data[0]<<24)|(int32_t)(rx_message.Data[1]<<16) 
            | (int32_t)(rx_message.Data[2]<<8) | (int32_t)(rx_message.Data[3]);
            
            last_yaw_angle = this_yaw_angle;
            this_yaw_angle = -((float)temp_yaw_angle*0.01);         
        }
        
        //Remote control and mouse reading from turret channel 遥控器 鼠标  云台通道
        if(rx_message.StdId == 0x402)
        { 
            temp_yaw = (uint16_t)(rx_message.Data[0]<<8)|(uint16_t)(rx_message.Data[1]);
            temp_pitch = (uint16_t)(rx_message.Data[2]<<8)|(uint16_t)(rx_message.Data[3]);
            shooting_flag = (uint8_t)rx_message.Data[4];   
            mode_flag = (uint8_t)rx_message.Data[6];//S2 switch
            
            //for mouse            
            if(shooting_flag == 1)              //cyq: trigger shoot
            {   
                if(ShootFlag == 1)
                {                       
                    Motor_PWM_Set(MOTOR_NUM1,-1000);    
                    ShootFlag=0;
                }                  
            }
            else 
            {         
                if(ShootFlag == 0)
                {   
                    ShootFlag=1;
                }                                
            }
            if (mode_flag == 1)
            {
                target_pitch_angle += (temp_pitch - 1024)/66.0;//remote control
                target_yaw_angle += (temp_yaw - 1024)/600.0 ;//cyq                                
            }
            else
            {
                target_pitch_angle -= (temp_pitch - 1024)/10.0;//cyq: mouse
                target_yaw_angle += (temp_yaw - 1024)/10.0 ;//cyq: target new program
            }
            if(target_pitch_angle > pitch_max)
            {
                target_pitch_angle = pitch_max;
            }
            else if(target_pitch_angle < -pitch_max)
            {
                target_pitch_angle = -pitch_max;
            }
        }  
    }
}

// turn wheels, pitch and yaw
// tx_message1 for wheels
// tx_message2 for pitch and yaw
void Motor_Test_Can_2(void) {


    CanTxMsg tx_message1;
    CanTxMsg tx_message2;  

    tx_message1.StdId = 0x200;
    tx_message1.DLC = 0x08;
    tx_message1.RTR = CAN_RTR_Data;
    tx_message1.IDE = CAN_Id_Standard;

    tx_message2.StdId = 0x1FF;
    tx_message2.DLC = 0x08;
    tx_message2.RTR = CAN_RTR_Data;
    tx_message2.IDE = CAN_Id_Standard;

/*****************************
    tx_message1 Controls wheels
*******************************/

/*
    Data 0 and 1 -> Front right wheel
    Data 2 and 3 -> Front left wheel
    Data 4 and 5 -> Rear left wheel
    Data 6 and 7 -> Rear right wheel
    data is sent in little endian
    positive values -> counter clockwise rotation
    negative values -> clockwise rotation
    I tested:
    +500 to right front wheel. +500 = 0xF401z in little endian so: 
    tx_message1.Data[0] = 0xF4;
    tx_message1.Data[1] = 0x01;
    -500 to right front wheel. -500 = 0x0CFE in little endian so:: 
    tx_message1.Data[0] = 0x0C;
    tx_message1.Data[1] = 0xFE;
*/  
    tx_message1.Data[0] = 0x00;
    tx_message1.Data[1] = 0x00;
    tx_message1.Data[2] = 0x00;
    tx_message1.Data[3] = 0x00;
    tx_message1.Data[4] = 0x00;
    tx_message1.Data[5] = 0x00;
    tx_message1.Data[6] = 0x00;
    tx_message1.Data[7] = 0x00;


/*****************************
    tx_message2 Controls pitch and yaw
*******************************/

/*
    Data 0 and 1 -> yaw (side to side)
    Data 2 and 3 -> pitch (up, down)
   
    data is sent in little endian
    positive values -> yaw right turn        pitch up
    negative values -> yaw left turn        pitch down
    +1000 to yaw. +1000 = 0xE803 in little endian so: 
    tx_message1.Data[0] = 0xE8;
    tx_message1.Data[1] = 0x03;
    -1000 to yaw. -1000 = 0x18FC in little endian so:: 
    tx_message1.Data[0] = 0x18;
    tx_message1.Data[1] = 0xFC;
*/
    tx_message2.Data[0] = 0x00;
    tx_message2.Data[1] = 0x00;
    tx_message2.Data[2] = 0x00;
    tx_message2.Data[3] = 0x00;
    tx_message2.Data[4] = 0x00;
    tx_message2.Data[5] = 0x00;
    tx_message2.Data[6] = 0x00;
    tx_message2.Data[7] = 0x00;

    CAN_Transmit(CAN2,&tx_message1);
    //delay_ms(10);
    CAN_Transmit(CAN2,&tx_message2);
}

