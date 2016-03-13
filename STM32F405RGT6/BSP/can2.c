#include "can2.h"
#include "led.h"
#include "usart3.h"

///Turns on Beck's trying for PID controller
#define PID true


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
uint16_t measured_pitch_angle;

// yaw and pitch current rx messages from CAN
uint16_t temp_yaw_current;
uint16_t temp_pitch_current;

// #if PID
    // Beck read from the datasheet, and guess it is the measured current
    // yaw and pitch measured current rx messages from CAN
    uint16_t measure_yaw_current;
    uint16_t measure_pitch_current;
// #endif

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
            uint16_t yaw_data0 = rx_message.Data[0];
            uint16_t yaw_data1 = rx_message.Data[1];
            uint16_t yaw_data4 = rx_message.Data[4];
            uint16_t yaw_data5 = rx_message.Data[5];

            temp_yaw_angle = (yaw_data0)<<8|(yaw_data1);
            temp_yaw_current = (yaw_data4)<<8|(yaw_data5);


            #if PID
                uint16_t yaw_data2 = rx_message.Data[2];
                uint16_t yaw_data3 = rx_message.Data[3];
                measure_yaw_current = (pitch_data2)<<8|(pitch_data3);
            #endif

            // normalize angle range since default angle range is werid
            if(temp_yaw_angle > 6060 && temp_yaw_angle < 8200) {
                temp_yaw_angle = temp_yaw_angle - 6060;
            } else {
                temp_yaw_angle = temp_yaw_angle + 2130;
            }
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
            uint16_t pitch_data0 = rx_message.Data[0];
            uint16_t pitch_data1 = rx_message.Data[1];
            uint16_t pitch_data4 = rx_message.Data[4];
            uint16_t pitch_data5 = rx_message.Data[5];

            uint16_t pitch_data2 = rx_message.Data[2];
            uint16_t pitch_data3 = rx_message.Data[3];
            measure_pitch_current = (pitch_data2)<<8|(pitch_data3);

            measured_pitch_angle = (pitch_data0)<<8|(pitch_data1);

            temp_pitch_current = (pitch_data4)<<8|(pitch_data5);

            #if PID
                uint16_t pitch_data2 = rx_message.Data[2];
                uint16_t pitch_data3 = rx_message.Data[3];
                measure_pitch_current = (pitch_data2)<<8|(pitch_data3);
                // printf("Pitch angle: %i", measured_pitch_angle);
                // printf("Pitch current measured?: %i", measure_pitch_current);
                // printf("Pitch current: %i", temp_pitch_current);
            #endif
        }
    }
}





/*************************************************************************
                          Code to Set Motor Current Values
Description: Motor_Current_Send(int Motor_ID, int current)
*************************************************************************/

// global variables to store current state in every motor
uint16_t motor_yaw_cur;
uint16_t motor_pitch_cur;
uint16_t motor_front_right_cur;
uint16_t motor_front_left_cur;
uint16_t motor_back_left_cur;
uint16_t motor_back_right_cur;

// different CAN messages for pitch/yaw and wheel motors
CanTxMsg tx_pitchyaw_message;
CanTxMsg tx_wheels_message;

// sets up pitch and yaw address for tx
void PitchYaw_Address_Setup() {
    tx_pitchyaw_message.StdId = 0x1FF;
    tx_pitchyaw_message.DLC = 0x08;
    tx_pitchyaw_message.RTR = CAN_RTR_Data;
    tx_pitchyaw_message.IDE = CAN_Id_Standard;
}

// sets up wheel address for tx
void Wheels_Address_Setup() {
    tx_wheels_message.StdId = 0x200;
    tx_wheels_message.DLC = 0x08;
    tx_wheels_message.RTR = CAN_RTR_Data;
    tx_wheels_message.IDE = CAN_Id_Standard;
}

// prepares whole 0x1FF pitch/yaw CAN message for tx
/*  From Beck's observation, the sending data is the opposite way, big-endian
    first data is top, and that is tested
*/
void Set_PitchYaw_Current() {
    // tx_pitchyaw_message.Data[0] = motor_yaw_cur & 0xFF; // sample out the top 8 bits
    // tx_pitchyaw_message.Data[1] = motor_yaw_cur >> 8;
    // tx_pitchyaw_message.Data[2] = motor_pitch_cur & 0xFF;
    // tx_pitchyaw_message.Data[3] = motor_pitch_cur >> 8;
    tx_pitchyaw_message.Data[0] = motor_yaw_cur >> 8;
    tx_pitchyaw_message.Data[1] = motor_yaw_cur & 0xFF;
    tx_pitchyaw_message.Data[2] = motor_pitch_cur >> 8;
    tx_pitchyaw_message.Data[3] = motor_pitch_cur & 0xFF;
    tx_pitchyaw_message.Data[4] = 0x00;
    tx_pitchyaw_message.Data[5] = 0x00;
    tx_pitchyaw_message.Data[6] = 0x00;
    tx_pitchyaw_message.Data[7] = 0x00;
}

// prepares whole 0x200 wheel CAN message for tx
// Still small-endian
void Set_Wheels_Current() {
    tx_wheels_message.Data[0] = motor_front_right_cur & 0xFF;
    tx_wheels_message.Data[1] = motor_front_right_cur >> 8;
    tx_wheels_message.Data[2] = motor_front_left_cur & 0xFF;
    tx_wheels_message.Data[3] = motor_front_left_cur >> 8;
    tx_wheels_message.Data[4] = motor_back_left_cur & 0xFF;
    tx_wheels_message.Data[5] = motor_back_left_cur >> 8;
    tx_wheels_message.Data[6] = motor_back_right_cur & 0xFF;
    tx_wheels_message.Data[7] = motor_back_right_cur >> 8;
}

// Sends specified current value to motor specified by Motor_ID
// Motor_ID mapping is in can2.h
void Motor_Current_Send(int Motor_ID, int current) {

    //tx_message.StdId = 0x1FF for pitch and yaw
    // Data 0 and 1 -> yaw (side to side)           Motor_ID 1
    // Data 2 and 3 -> pitch (up, down)             Motor_ID 2

    // tx_message.StdId = 0x200 for wheels
    // Data 0 and 1 -> Front right wheel            Motor_ID 3
    // Data 2 and 3 -> Front left wheel             Motor_ID 4
    // Data 4 and 5 -> Rear left wheel              Motor_ID 5
    // Data 6 and 7 -> Rear right wheel             Motor_ID 6

        switch (Motor_ID)
    {
        case MOTOR_YAW:         PitchYaw_Address_Setup();
                                motor_yaw_cur = current;
                                Set_PitchYaw_Current();
                                CAN_Transmit(CAN2,&tx_pitchyaw_message);
                                Wheels_Address_Setup();
                                Set_Wheels_Current();
                                CAN_Transmit(CAN2,&tx_wheels_message); break; //If Motor1 is chosen, Frame ID  is 0x14 under Speed_LOCATION Mode

        case MOTOR_PITCH:       PitchYaw_Address_Setup();
                                motor_pitch_cur = current;
                                Set_PitchYaw_Current();
                                CAN_Transmit(CAN2,&tx_pitchyaw_message);
                                Wheels_Address_Setup();
                                Set_Wheels_Current();
                                CAN_Transmit(CAN2,&tx_wheels_message); break; //If Motor2 is chosen, Frame ID  is 0x24 under Speed_LOCATION Mode

        case MOTOR_FRONT_RIGHT: Wheels_Address_Setup();
                                motor_front_right_cur = current;
                                Set_Wheels_Current();
                                CAN_Transmit(CAN2,&tx_wheels_message);
                                PitchYaw_Address_Setup();
                                Set_PitchYaw_Current();
                                CAN_Transmit(CAN2,&tx_pitchyaw_message); break; //If Motor3 is chosen, Frame ID  is 0x34 under Speed_LOCATION Mode

        case MOTOR_FRONT_LEFT:  Wheels_Address_Setup();
                                motor_front_left_cur = current;
                                Set_Wheels_Current();
                                CAN_Transmit(CAN2,&tx_wheels_message);
                                PitchYaw_Address_Setup();
                                Set_PitchYaw_Current();
                                CAN_Transmit(CAN2,&tx_pitchyaw_message); break; //If Motor4 is chosen, Frame ID  is 0x44 under Speed_LOCATION Mode

        case MOTOR_BACK_LEFT:   Wheels_Address_Setup();
                                motor_back_left_cur = current;
                                Set_Wheels_Current();
                                CAN_Transmit(CAN2,&tx_wheels_message);
                                PitchYaw_Address_Setup();
                                Set_PitchYaw_Current();
                                CAN_Transmit(CAN2,&tx_pitchyaw_message); break; //If Motor5 is chosen, Frame ID  is 0x54 under Speed_LOCATION Mode

        case MOTOR_BACK_RIGHT:  Wheels_Address_Setup();
                                motor_back_right_cur = current;
                                Set_Wheels_Current();
                                CAN_Transmit(CAN2,&tx_wheels_message);
                                PitchYaw_Address_Setup();
                                Set_PitchYaw_Current();
                                CAN_Transmit(CAN2,&tx_pitchyaw_message); break; //If Motor6 is chosen, Frame ID  is 0x64 under Speed_LOCATION Mode

    }
    delay_ms(1);


}



// turn wheels, pitch and yaw
// tx_message1 for wheels
// tx_message2 for pitch and yaw
void Motor_Reset_Can_2(void) {


    CanTxMsg tx_message1;
    CanTxMsg tx_message2;
    motor_yaw_cur = 0;
    motor_pitch_cur = 0;
    motor_front_right_cur = 0;
    motor_front_left_cur = 0;
    motor_back_left_cur = 0;
    motor_back_right_cur = 0;



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
    tx_message.StdId = 0x200 for wheels
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

    tx_message.StdId = 0x1FF for pitch and yaw
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
    CAN_Transmit(CAN2,&tx_message2);
}


// For manually setting currents to motors for testing
void Motor_ManSet_Can_2(void) {


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
    tx_message.StdId = 0x200 for wheels
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

    tx_message.StdId = 0x1FF for pitch and yaw
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
    tx_message2.Data[2] = 0x64;
    tx_message2.Data[3] = 0x00;
    tx_message2.Data[4] = 0x00;
    tx_message2.Data[5] = 0x00;
    tx_message2.Data[6] = 0x00;
    tx_message2.Data[7] = 0x00;

    CAN_Transmit(CAN2,&tx_message2);
}
