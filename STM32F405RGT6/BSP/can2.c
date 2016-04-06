#include "can2.h"
#include "led.h"
#include "usart3.h"

extern RC_Ctl_t RC_Ctl;
extern arduino_data data_usart_3;
uint8_t Remote_On = 0;

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

int32_t turn_cnt = 0;
float dipan_gyro_angle = 0.0;
int32_t temp_dipan_gyro = 0;

float temp_pitch = 0;
float temp_yaw = 0;
uint8_t shooting_flag = 0;
uint8_t mode_flag=0;
uint8_t ShootFlag=0;

float target_pitch_angle;
float target_yaw_angle;

// yaw and pitch angle rx messages from CAN
uint16_t measured_yaw_angle;   // range from 0~8191, 0x1FFF
uint16_t measured_pitch_angle; // range from 0~8191, 0x1FFF

// Beck read from the datasheet, and guess it is the measured current
// yaw and pitch measured current rx messages from CAN
int16_t measured_yaw_current;
int16_t measured_pitch_current;

// yaw and pitch current rx messages from CAN
int16_t target_yaw_current;
int16_t target_pitch_current;

int16_t x101;
int16_t x123;
int16_t x145;
int16_t x167;

int16_t x201;
int16_t x223;
int16_t x245;
int16_t x267;

int16_t x301;
int16_t x323;
int16_t x345;
int16_t x367;

int16_t x401;
int16_t x423;
int16_t x445;
int16_t x467;


/*************************************************************************
                          CAN2_RX0_IRQHandler
Description: Interrupt receive for chassis CAN bus data and single axis gyroscope
描述：单轴陀螺仪、底盘主控CAN数据接收中断
*************************************************************************/

/*******
RX addresses
0x201: Front right wheel
0x202: Front left wheel
0x203: Back left wheel
0x204: Back right wheel
0x205: Yaw
0x206: Pitch

All rx messages mapped the same way:
data 0 and 1 measure angle
data 4 and 5 relates to what current you are tx to motors
data 4 and 5 NOT same as current tx value
*******/
void CAN2_RX0_IRQHandler(void)
{
    CanRxMsg rx_message;
    if (CAN_GetITStatus(CAN2,CAN_IT_FMP0)!= RESET)
    {
       CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
       CAN_Receive(CAN2, CAN_FIFO0, &rx_message);

    /***************
    Wheel RX Address
    0x201: Front right wheel
    0x202: Front left wheel
    0x203: Back left wheel
    0x204: Back right wheel

    data 0 and 1 measure position of wheel (0, around 8200)
    clockwise wheel rotation decreases wheel's position
    wheel position value repeats (decrease from 0 means go back to 8200 again)

    data 4 and 5 relates to what current you are tx to motors
    data 4 and 5 NOT same as current tx value

    data 2, 3, 6, 7 not useful
    ***************/

       if(rx_message.StdId == 0x201)
        {
            uint16_t x1data0 = rx_message.Data[0];
            uint16_t x1data1 = rx_message.Data[1];
            uint16_t x1data2 = rx_message.Data[2];
            uint16_t x1data3 = rx_message.Data[3];
            uint16_t x1data4 = rx_message.Data[4];
            uint16_t x1data5 = rx_message.Data[5];
            uint16_t x1data6 = rx_message.Data[6];
            uint16_t x1data7 = rx_message.Data[7];

            x101 = ( x1data0 << 8 ) | x1data1;
            x123 = ( x1data2 << 8 ) | x1data3;
            x145 = ( x1data4 << 8 ) | x1data5;
            x167 = ( x1data6 << 8 ) | x1data7;
        }

        if(rx_message.StdId == 0x202)
        {

            uint16_t x2data0 = rx_message.Data[0];
            uint16_t x2data1 = rx_message.Data[1];
            uint16_t x2data2 = rx_message.Data[2];
            uint16_t x2data3 = rx_message.Data[3];
            uint16_t x2data4 = rx_message.Data[4];
            uint16_t x2data5 = rx_message.Data[5];
            uint16_t x2data6 = rx_message.Data[6];
            uint16_t x2data7 = rx_message.Data[7];
            x201 = ( x2data0 << 8 ) | x2data1;
            x223 = ( x2data2 << 8 ) | x2data3;
            x245 = ( x2data4 << 8 ) | x2data5;
            x267 = ( x2data6 << 8 ) | x2data7;

        }

        if(rx_message.StdId == 0x203)
        {
            uint16_t x3data0 = rx_message.Data[0];
            uint16_t x3data1 = rx_message.Data[1];
            uint16_t x3data2 = rx_message.Data[2];
            uint16_t x3data3 = rx_message.Data[3];
            uint16_t x3data4 = rx_message.Data[4];
            uint16_t x3data5 = rx_message.Data[5];
            uint16_t x3data6 = rx_message.Data[6];
            uint16_t x3data7 = rx_message.Data[7];
            x301 = ( x3data0 << 8 ) | x3data1;
            x323 = ( x3data2 << 8 ) | x3data3;
            x345 = ( x3data4 << 8 ) | x3data5;
            x367 = ( x3data6 << 8 ) | x3data7;
        }

        if(rx_message.StdId == 0x204)
        {

            uint16_t x4data0 = rx_message.Data[0];;
            uint16_t x4data1 = rx_message.Data[1];;
            uint16_t x4data2 = rx_message.Data[2];;
            uint16_t x4data3 = rx_message.Data[3];;
            uint16_t x4data4 = rx_message.Data[4];;
            uint16_t x4data5 = rx_message.Data[5];;
            uint16_t x4data6 = rx_message.Data[6];;
            uint16_t x4data7 = rx_message.Data[7];;
            x401 = ( x4data0 << 8 ) | x4data1;
            x423 = ( x4data2 << 8 ) | x4data3;
            x445 = ( x4data4 << 8 ) | x4data5;
            x467 = ( x4data6 << 8 ) | x4data7;
        }

/************** End of Wheel Motor RX Code and Address **************/


        /************ YAW ************/
        // Yaw angle range is: [around 40, around 4800]
        // 40 is right-most yaw position
        // around 4800 is left-most yaw position
        // data 0 and 1 measure angle

        // data 4 and 5 relates to what current you are tx to motors
        // data 4 and 5 NOT same as current tx value
        // -1000 current value = 27852 (yaw_data4)<<8|(yaw_data5) value
        // -750 current value = 24305 (yaw_data4)<<8|(yaw_data5) value
        // -500 current value = 16630 (yaw_data4)<<8|(yaw_data5) value
        // pitch has same current to (data4<<8)|(data5) conversion
        if(rx_message.StdId == 0x205)
        {

            // construct message from data[0] and data[1]
            uint16_t yaw_data0 = rx_message.Data[0];
            uint16_t yaw_data1 = rx_message.Data[1];
            uint16_t yaw_data2 = rx_message.Data[2];
            uint16_t yaw_data3 = rx_message.Data[3];
            uint16_t yaw_data4 = rx_message.Data[4];
            uint16_t yaw_data5 = rx_message.Data[5];

            measured_yaw_angle = (yaw_data0)<<8|(yaw_data1);
            measured_yaw_current = (yaw_data2)<<8|(yaw_data3);
            target_yaw_current = (yaw_data4)<<8|(yaw_data5);

            // normalize angle range since default angle range is werid
            if(measured_yaw_angle > 6000 && measured_yaw_angle < 8191) {
                measured_yaw_angle = measured_yaw_angle - 6000;
            } else {
                measured_yaw_angle = measured_yaw_angle + 2190;
            }
        }

        /************ PITCH ************/
        // pitch angle range is [around 3520, around 4500]
        // around 3520 is highest pitch position
        // around 4500 is lowest pitch position
        // data 0 and 1 measure angle

        // data 4 and 5 relates to what current you are tx to motors
        // data 4 and 5 NOT same as current tx value
        // -1000 current value = 27852 (pitch_data4)<<8|(pitch_data5) value
        // -750 current value = 24305 (pitch_data4)<<8|(pitch_data5) value
        // -500 current value = 16630 (pitch_data4)<<8|(pitch_data5) value
        // yaw has same current to (data4<<8)|(data5) conversion
        if(rx_message.StdId == 0x206)
        {
            // construct message from data[0] and data[1]
            uint16_t pitch_data0 = rx_message.Data[0];
            uint16_t pitch_data1 = rx_message.Data[1];
            uint16_t pitch_data2 = rx_message.Data[2];
            uint16_t pitch_data3 = rx_message.Data[3];
            uint16_t pitch_data4 = rx_message.Data[4];
            uint16_t pitch_data5 = rx_message.Data[5];

            measured_pitch_angle = (pitch_data0)<<8|(pitch_data1);
            measured_pitch_current = (pitch_data2)<<8|(pitch_data3);
            target_pitch_current = (pitch_data4)<<8|(pitch_data5);
        }



        // Rest of IRQHandler is provided old code I didn't use
        /*
        //Single axis gyroscope data 单轴陀螺仪数据
        if(rx_message.StdId == 0x401)
        {
            gyro_ok_flag = 1;
            measured_yaw_angle = (int32_t)(rx_message.Data[0]<<24)|(int32_t)(rx_message.Data[1]<<16)
            | (int32_t)(rx_message.Data[2]<<8) | (int32_t)(rx_message.Data[3]);

            last_yaw_angle = this_yaw_angle;
            this_yaw_angle = -((float)measured_yaw_angle*0.01);
        }

        //Remote controller, mouse, and turret channel 遥控器 鼠标  云台通道
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
        */
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

/************ For Red C Motor
// prepares whole 0x200 wheel CAN message for tx
// */
// void Set_Wheels_Current() {
//     tx_wheels_message.Data[0] = motor_front_left_cur >> 8;
//     tx_wheels_message.Data[1] = motor_front_left_cur & 0xFF;
//     tx_wheels_message.Data[2] = motor_back_left_cur >> 8;
//     tx_wheels_message.Data[3] = motor_back_left_cur & 0xFF;
//     tx_wheels_message.Data[4] = motor_front_right_cur >> 8;
//     tx_wheels_message.Data[5] = motor_front_right_cur & 0xFF;
//     tx_wheels_message.Data[6] = motor_back_right_cur >> 8;
//     tx_wheels_message.Data[7] = motor_back_right_cur & 0xFF;
// }


//*********** For Blue Motor
// prepares whole 0x200 wheel CAN message for tx
/*
Change, same as what Beck observed for pitch/yaw
*/
void Set_Wheels_Current() {

    tx_wheels_message.Data[0] = motor_front_right_cur >> 8;
    tx_wheels_message.Data[1] = motor_front_right_cur & 0xFF;
    tx_wheels_message.Data[2] = motor_front_left_cur >> 8;
    tx_wheels_message.Data[3] = motor_front_left_cur & 0xFF;
    tx_wheels_message.Data[4] = motor_back_left_cur >> 8;
    tx_wheels_message.Data[5] = motor_back_left_cur & 0xFF;
    tx_wheels_message.Data[6] = motor_back_right_cur >> 8;
    tx_wheels_message.Data[7] = motor_back_right_cur & 0xFF;
}

// Enables cannon to be driven with remote
void Remote_Control() {
    // To see if remote is off or not
    if (RC_Ctl.rc.ch2 < RC_CH_VALUE_MIN
        || RC_Ctl.rc.ch3 < RC_CH_VALUE_MIN
        ) {
        Remote_On = 0;
    } else {
        Remote_On = 1;
    }


    uint16_t drive = RC_Ctl.rc.ch3 - RC_CH_VALUE_OFFSET;
    uint16_t strafe = RC_Ctl.rc.ch2 - RC_CH_VALUE_OFFSET;
    uint16_t rotate = RC_Ctl.rc.ch0 - RC_CH_VALUE_OFFSET;

    if(Remote_On == 1) {
        motor_front_right_cur = 11*(-1*drive + strafe - rotate);
        motor_back_right_cur = 11*(-1*drive - strafe - rotate);
        motor_front_left_cur = 11*(drive + strafe - rotate);
        motor_back_left_cur = 11*(drive - strafe - rotate);
    }

    Wheels_Address_Setup();
    Set_Wheels_Current();
    CAN_Transmit(CAN2,&tx_wheels_message);
}

// Sends specified current value to motor specified by Motor_ID
// Motor_ID mapping is in can2.h
void Motor_Current_Send(int Motor_ID, int current) {

    //tx_message.StdId = 0x1FF for pitch and yaw
    // Data 0 and 1 -> yaw (side to side)           Motor_ID 1
    // Data 2 and 3 -> pitch (up, down)             Motor_ID 2

    // ************** For Red C Motor **************
    // Data 0 and 1 -> Front left wheel            Motor_ID 3
    // Data 2 and 3 -> back left wheel             Motor_ID 4
    // Data 4 and 5 -> front right wheel           Motor_ID 5
    // Data 6 and 7 -> back right wheel            Motor_ID 6

    // ************** For Blue C Motor **************
    // Data 0 and 1 -> Front right wheel           Motor_ID 3
    // Data 2 and 3 -> Front left wheel            Motor_ID 4
    // Data 4 and 5 -> Rear left wheel             Motor_ID 5
    // Data 6 and 7 -> Rear right wheel            Motor_ID 6

    /****************
    ************** For Red C Motor **************
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

        case MOTOR_FRONT_LEFT: Wheels_Address_Setup();
                                motor_front_left_cur = current;
                                Set_Wheels_Current();
                                CAN_Transmit(CAN2,&tx_wheels_message);
                                PitchYaw_Address_Setup();
                                Set_PitchYaw_Current();
                                CAN_Transmit(CAN2,&tx_pitchyaw_message); break; //If Motor3 is chosen, Frame ID  is 0x34 under Speed_LOCATION Mode

        case MOTOR_BACK_LEFT:  Wheels_Address_Setup();
                                motor_back_left_cur = current;
                                Set_Wheels_Current();
                                CAN_Transmit(CAN2,&tx_wheels_message);
                                PitchYaw_Address_Setup();
                                Set_PitchYaw_Current();
                                CAN_Transmit(CAN2,&tx_pitchyaw_message); break; //If Motor4 is chosen, Frame ID  is 0x44 under Speed_LOCATION Mode

        case MOTOR_FRONT_RIGHT:   Wheels_Address_Setup();
                                motor_front_right_cur = current;
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

    ****************/

    //************** For Blue C Motor **************
        switch (Motor_ID)
    {
        case MOTOR_YAW:         PitchYaw_Address_Setup();
                                motor_yaw_cur = current;
                                Set_PitchYaw_Current();
                                CAN_Transmit(CAN2,&tx_pitchyaw_message); break; //If Motor1 is chosen, Frame ID  is 0x14 under Speed_LOCATION Mode

        case MOTOR_PITCH:       PitchYaw_Address_Setup();
                                motor_pitch_cur = current;
                                Set_PitchYaw_Current();
                                CAN_Transmit(CAN2,&tx_pitchyaw_message); break; //If Motor2 is chosen, Frame ID  is 0x24 under Speed_LOCATION Mode

        // case MOTOR_FRONT_RIGHT: Wheels_Address_Setup();
        //                         motor_front_right_cur = current;
        //                         Set_Wheels_Current();
        //                         CAN_Transmit(CAN2,&tx_wheels_message); break; //If Motor3 is chosen, Frame ID  is 0x34 under Speed_LOCATION Mode
        //
        // case MOTOR_FRONT_LEFT:  Wheels_Address_Setup();
        //                         motor_front_left_cur = current;
        //                         Set_Wheels_Current();
        //                         CAN_Transmit(CAN2,&tx_wheels_message); break; //If Motor4 is chosen, Frame ID  is 0x44 under Speed_LOCATION Mode
        //
        // case MOTOR_BACK_LEFT:   Wheels_Address_Setup();
        //                         motor_back_left_cur = current;
        //                         Set_Wheels_Current();
        //                         CAN_Transmit(CAN2,&tx_wheels_message); break; //If Motor5 is chosen, Frame ID  is 0x54 under Speed_LOCATION Mode
        //
        // case MOTOR_BACK_RIGHT:  Wheels_Address_Setup();
        //                         motor_back_right_cur = current;
        //                         Set_Wheels_Current();
        //                         CAN_Transmit(CAN2,&tx_wheels_message); break; //If Motor6 is chosen, Frame ID  is 0x64 under Speed_LOCATION Mode

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
    ************** For Red C Motor **************
    Data 0 and 1 -> Front left wheel            Motor_ID 3
    Data 2 and 3 -> back left wheel             Motor_ID 4
    Data 4 and 5 -> front right wheel           Motor_ID 5
    Data 6 and 7 -> back right wheel            Motor_ID 6

    ************** For Blue C Motor **************
    Data 0 and 1 -> Front right wheel           Motor_ID 3
    Data 2 and 3 -> Front left wheel            Motor_ID 4
    Data 4 and 5 -> Rear left wheel             Motor_ID 5
    Data 6 and 7 -> Rear right wheel            Motor_ID 6


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

    ************** For Red C Motor **************
    Data 0 and 1 -> Front left wheel            Motor_ID 3
    Data 2 and 3 -> back left wheel             Motor_ID 4
    Data 4 and 5 -> front right wheel           Motor_ID 5
    Data 6 and 7 -> back right wheel            Motor_ID 6

    ************** For Blue C Motor **************
    Data 0 and 1 -> Front right wheel           Motor_ID 3
    Data 2 and 3 -> Front left wheel            Motor_ID 4
    Data 4 and 5 -> Rear left wheel             Motor_ID 5
    Data 6 and 7 -> Rear right wheel            Motor_ID 6

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
    tx_message1.Data[0] = 0xF4;
    tx_message1.Data[1] = 0x01;

    tx_message1.Data[2] = 0xF4;
    tx_message1.Data[3] = 0x01;

    tx_message1.Data[4] = 0x0C;
    tx_message1.Data[5] = 0xFE;

    tx_message1.Data[6] = 0x0C;
    tx_message1.Data[7] = 0xFE;


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

    // CAN_Transmit(CAN2,&tx_message1);
}
