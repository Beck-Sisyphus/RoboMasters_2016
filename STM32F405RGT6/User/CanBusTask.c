#include "main.h"

static uint32_t can_count = 0;

volatile Encoder CM1Encoder = {0,0,0,0,0,0,0};
volatile Encoder CM2Encoder = {0,0,0,0,0,0,0};
volatile Encoder CM3Encoder = {0,0,0,0,0,0,0};
volatile Encoder CM4Encoder = {0,0,0,0,0,0,0};
volatile Encoder GMYawEncoder = {0,0,0,0,0,0,0};
volatile Encoder GMPitchEncoder = {0,0,0,0,0,0,0};

// yaw and pitch angle rx messages from CAN
int16_t measured_yaw_angle;   // range from 0~8191, 0x1FFF
int16_t measured_pitch_angle; // range from 0~8191, 0x1FFF

volatile float measured_201_angle;
volatile int16_t measured_201_speed;

volatile float measured_202_angle;
volatile int16_t measured_202_speed;

volatile float measured_203_angle;
volatile int16_t measured_203_speed;

volatile float measured_204_angle;
volatile int16_t measured_204_speed;

float measured_yaw_angle_401 = 0.0f;

int16_t x145;
int16_t x167;
int16_t x245;
int16_t x267;
int16_t x345;
int16_t x367;
int16_t x445;
int16_t x467;
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
void CanReceiveMsgProcess(CanRxMsg * rx_message)
{
    can_count++;
    /***************
    Wheel RX Address
    0x201: Front right wheel
    0x202: Front left wheel
    0x203: Back left wheel
    0x204: Back right wheel
    0x205: yaw
    0x206: pitch

    For sample robot 2015, with EC60 motor drives:
    data 0 and 1 measure position of wheel, from 0 to 8191
    clockwise wheel rotation decreases wheel's position
    wheel position value repeats (decrease from 0 means go back to 8200 again)

    data 4 and 5 relates to what current you are tx to motors
    data 4 and 5 NOT same as current tx value

    data 2, 3, 6, 7 not useful

    For robot we build in 2016, with 820R motor drives:
    You calibrate the CAN address in the first time
    data 0 and 1 measure position of wheel, from 0 to 8191
    data 2 and 3 measure rotational speed, in unit of RPM
    data 4,5, 6, and 7 are null.
    ***************/
    switch(rx_message->StdId)
    {
        case 0x201:
        {
            // (can_count<=50) ? GetEncoderBias(&CM1Encoder,rx_message):EncoderProcess(&CM1Encoder,rx_message);
            EncoderProcess(&CM1Encoder,rx_message);
            measured_201_angle = CM1Encoder.ecd_angle;
            measured_201_speed = CM1Encoder.velocity_raw;
        }break;
        case 0x202:
        {
            // (can_count<=50) ? GetEncoderBias(&CM2Encoder,rx_message):EncoderProcess(&CM2Encoder,rx_message);
            EncoderProcess(&CM2Encoder,rx_message);
            measured_202_angle = CM2Encoder.ecd_angle;
            measured_202_speed = CM2Encoder.velocity_raw;
        }break;
        case 0x203:
        {
            // (can_count<=50) ? GetEncoderBias(&CM3Encoder,rx_message):EncoderProcess(&CM3Encoder,rx_message);
            EncoderProcess(&CM3Encoder,rx_message);
            measured_203_angle = CM3Encoder.ecd_angle;
            measured_203_speed = CM3Encoder.velocity_raw;
        }break;
        case 0x204:
        {
            // (can_count<=50) ? GetEncoderBias(&CM4Encoder,rx_message):EncoderProcess(&CM4Encoder,rx_message);
            EncoderProcess(&CM4Encoder,rx_message);
            measured_204_angle = CM4Encoder.ecd_angle;
            measured_204_speed = CM4Encoder.velocity_raw;
        }break;
        case 0x205: // yaw
        {
            int16_t yaw_data0 = rx_message->Data[0];
            int16_t yaw_data1 = rx_message->Data[1];

            measured_yaw_angle = (yaw_data0)<<8|(yaw_data1);
            // normalize angle range since default angle range is werid
            if(measured_yaw_angle > 6000 && measured_yaw_angle < 8191) {
                measured_yaw_angle = measured_yaw_angle - 6000;
            } else {
                measured_yaw_angle = measured_yaw_angle + 2190;
            }
        }break;
        case 0x206: // pitch
        {
            int16_t pitch_data0 = rx_message->Data[0];
            int16_t pitch_data1 = rx_message->Data[1];

            measured_pitch_angle = (pitch_data0)<<8|(pitch_data1);
        }break;
        case 0x401:
        {
            measured_yaw_angle_401 = -(float)0.01f * ((int32_t)(rx_message->Data[0]<<24)|(int32_t)(rx_message->Data[1]<<16)\
            | (int32_t)(rx_message->Data[2]<<8) | (int32_t)(rx_message->Data[3]));
        }
        default:{}
    }
}

void EncoderProcess(volatile Encoder *v, CanRxMsg * rx_message)
{
    v->angle_raw_last = v->angle_raw;
    v->angle_raw    = ( rx_message->Data[0] << 8 ) | rx_message->Data[1];
    v->velocity_raw = ( rx_message->Data[2] << 8 ) | rx_message->Data[3];
    v->angle_diff = v->angle_raw - v->angle_raw_last;
    // If the feeback from the encoder changes too much, the rotation is incremented
    if(v->angle_diff < -7500)
    {
      	v->round_count++;
    }
    else if(v->angle_diff > 7500)
    {
      	v->round_count--;
    }
    // Calculate the encoder output in a continous value domain
    v->angle_continous = v->angle_raw + v->round_count * ENCODER_MAX;
    // Calculate the angle, range from negative infinite to positive infinite
    v->ecd_angle = (float)(v->angle_raw - v->angle_bias)* RADIAN_CIRCLE / ENCODER_MAX + v->round_count * RADIAN_CIRCLE;

}

void GetEncoderBias(volatile Encoder *v, CanRxMsg * rx_message)
{
    v->angle_bias = ( rx_message->Data[0] << 8 ) | rx_message->Data[1];
    v->angle_continous = v->angle_bias;
    v->angle_raw_last = v->angle_bias;
}

/*************************************************************************
                          Code to Set Motor Current Values
Description: Motor_Current_Send(int Motor_ID, int current)
*************************************************************************/
// global variables to store current state in every motor
int16_t motor_yaw_cur;
int16_t motor_pitch_cur;
int16_t motor_front_right_cur;
int16_t motor_front_left_cur;
int16_t motor_back_left_cur;
int16_t motor_back_right_cur;

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

// controls pitch and yaw using given currents
void pitchyaw_control(int16_t yaw_current, int16_t pitch_current) {

  PitchYaw_Address_Setup();
  motor_yaw_cur = yaw_current;
  motor_pitch_cur = pitch_current;
  Set_PitchYaw_Current();
  CAN_Transmit(CAN2,&tx_pitchyaw_message);
}

// controls wheels using kinematic equations
void wheel_control(int16_t motor_201_vel, int16_t motor_202_vel, int16_t motor_203_vel, int16_t motor_204_vel)
{
    Wheels_Address_Setup();
    motor_front_right_cur = motor_201_vel;
    motor_front_left_cur  = motor_202_vel;
    motor_back_left_cur   = motor_203_vel;
    motor_back_right_cur  = motor_204_vel;

    Set_Wheels_Current();
    CAN_Transmit(CAN2, &tx_wheels_message);
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

/*

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

    encoder_angle = encoder_angle * 100.0f;
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


*/

        // Rest of IRQHandler is provided old code Tu Vu didn't use
        /*
        float temp_pitch = 0;
        float temp_yaw = 0;
        uint8_t shooting_flag = 0;
        uint8_t mode_flag=0;
        uint8_t ShootFlag=0;

        float target_pitch_angle;
        float target_yaw_angle;

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

// For manually setting currents to motors for testing
//
// void Motor_ManSet_Can_2(void) {
//
//
//     CanTxMsg tx_message1;
//     CanTxMsg tx_message2;
//
//     tx_message1.StdId = 0x200;
//     tx_message1.DLC = 0x08;
//     tx_message1.RTR = CAN_RTR_Data;
//     tx_message1.IDE = CAN_Id_Standard;
//
//     tx_message2.StdId = 0x1FF;
//     tx_message2.DLC = 0x08;
//     tx_message2.RTR = CAN_RTR_Data;
//     tx_message2.IDE = CAN_Id_Standard;
//
// /*****************************
//     tx_message1 Controls wheels
// *******************************/
//
// /*
//     tx_message.StdId = 0x200 for wheels
//
//     ************** For Red C Motor **************
//     Data 0 and 1 -> Front left wheel            Motor_ID 3
//     Data 2 and 3 -> back left wheel             Motor_ID 4
//     Data 4 and 5 -> front right wheel           Motor_ID 5
//     Data 6 and 7 -> back right wheel            Motor_ID 6
//
//     ************** For Blue C Motor **************
//     Data 0 and 1 -> Front right wheel           Motor_ID 3
//     Data 2 and 3 -> Front left wheel            Motor_ID 4
//     Data 4 and 5 -> Rear left wheel             Motor_ID 5
//     Data 6 and 7 -> Rear right wheel            Motor_ID 6
//
//     data is sent in little endian
//     positive values -> counter clockwise rotation
//     negative values -> clockwise rotation
//     I tested:
//     +500 to right front wheel. +500 = 0xF401z in little endian so:
//     tx_message1.Data[0] = 0xF4;
//     tx_message1.Data[1] = 0x01;
//     -500 to right front wheel. -500 = 0x0CFE in little endian so::
//     tx_message1.Data[0] = 0x0C;
//     tx_message1.Data[1] = 0xFE;
// */
//     tx_message1.Data[0] = 0xF4;
//     tx_message1.Data[1] = 0x01;
//
//     tx_message1.Data[2] = 0xF4;
//     tx_message1.Data[3] = 0x01;
//
//     tx_message1.Data[4] = 0x0C;
//     tx_message1.Data[5] = 0xFE;
//
//     tx_message1.Data[6] = 0x0C;
//     tx_message1.Data[7] = 0xFE;
//
//
// /*****************************
//     tx_message2 Controls pitch and yaw
// *******************************/
//
// /*
//
//     tx_message.StdId = 0x1FF for pitch and yaw
//     Data 0 and 1 -> yaw (side to side)
//     Data 2 and 3 -> pitch (up, down)
//
//     data is sent in little endian
//     positive values -> yaw right turn        pitch up
//     negative values -> yaw left turn        pitch down
//     +1000 to yaw. +1000 = 0xE803 in little endian so:
//     tx_message1.Data[0] = 0xE8;
//     tx_message1.Data[1] = 0x03;
//     -1000 to yaw. -1000 = 0x18FC in little endian so::
//     tx_message1.Data[0] = 0x18;
//     tx_message1.Data[1] = 0xFC;
// */
//     tx_message2.Data[0] = 0x00;
//     tx_message2.Data[1] = 0x00;
//     tx_message2.Data[2] = 0x00;
//     tx_message2.Data[3] = 0x00;
//     tx_message2.Data[4] = 0x00;
//     tx_message2.Data[5] = 0x00;
//     tx_message2.Data[6] = 0x00;
//     tx_message2.Data[7] = 0x00;
//
//     // CAN_Transmit(CAN2,&tx_message1);
// }
