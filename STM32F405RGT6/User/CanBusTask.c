#include "main.h"

static uint32_t can_count = 0;

gimbal_mapping_t gimbal_0 = GIMBAL_BLUE_SAMPLE_ROBOT_0;
gimbal_mapping_t gimbal_1 = GIMBAL_RED_SAMPLE_ROBOT_1;
gimbal_mapping_t gimbal_2 = GIMBAL_SOLDIER_2;
gimbal_mapping_t gimbal_3 = GIMBAL_SOLDIER_3;
gimbal_mapping_t gimbal_4 = GIMBAL_SOLDIER_4;
gimbal_mapping_t gimbal_5 = GIMBAL_SOLDIER_5;
gimbal_mapping_t gimbal_6 = GIMBAL_DEFAULT;
gimbal_mapping_t gimbal_7 = GIMBAL_HERO_ROBOT_CANNON_7;
gimbal_mapping_t gimbal_8 = GIMBAL_DEFAULT;

volatile Encoder CM1Encoder = {0,0,0,0,0,0,0,0,0,0};
volatile Encoder CM2Encoder = {0,0,0,0,0,0,0,0,0,0};
volatile Encoder CM3Encoder = {0,0,0,0,0,0,0,0,0,0};
volatile Encoder CM4Encoder = {0,0,0,0,0,0,0,0,0,0};
volatile Encoder GMYawEncoder = {0,0,0,0,0,0,0,0,0,0};
volatile Encoder GMPitchEncoder = {0,0,0,0,0,0,0,0,0,0};

// yaw and pitch angle rx messages from CAN
int16_t measured_yaw_angle;   // range from 0~8191, 0x1FFF
int16_t measured_pitch_angle; // range from 0~8191, 0x1FFF

float measured_yaw_angle_401 = 0.0f;
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
*******/
void CanReceiveMsgProcess(CanRxMsg * rx_message)
{
    can_count++;
    switch(rx_message->StdId)
    {
        case 0x201:
        {
            (can_count<=50) ? GetEncoderBias(&CM1Encoder,rx_message):EncoderProcess(&CM1Encoder,rx_message);
        }break;
        case 0x202:
        {
            (can_count<=50) ? GetEncoderBias(&CM2Encoder,rx_message):EncoderProcess(&CM2Encoder,rx_message);
        }break;
        case 0x203:
        {
            (can_count<=50) ? GetEncoderBias(&CM3Encoder,rx_message):EncoderProcess(&CM3Encoder,rx_message);
        }break;
        case 0x204:
        {
            (can_count<=50) ? GetEncoderBias(&CM4Encoder,rx_message):EncoderProcess(&CM4Encoder,rx_message);
        }break;
        case 0x205: // yaw
        {
            (can_count<=50) ? GetEncoderBias(&GMYawEncoder,rx_message):EncoderProcess(&GMYawEncoder,rx_message);
            // EncoderProcess(&GMYawEncoder,rx_message);
            int16_t yaw_data0 = rx_message->Data[0];
            int16_t yaw_data1 = rx_message->Data[1];

            measured_yaw_angle = (yaw_data0)<<8|(yaw_data1);
        }break;
        case 0x206: // pitch
        {
            (can_count<=50) ? GetEncoderBias(&GMPitchEncoder,rx_message):EncoderProcess(&GMPitchEncoder,rx_message);
            // EncoderProcess(&GMPitchEncoder,rx_message);
            // GMPitchEncoder.ecd_angle = -GMPitchEncoder.ecd_angle;
            int16_t pitch_data0 = rx_message->Data[0];
            int16_t pitch_data1 = rx_message->Data[1];

            measured_pitch_angle = (pitch_data0)<<8|(pitch_data1);
        }break;
        case 0x401:
        {
            measured_yaw_angle_401 = -(float)0.01f * ((int32_t)(rx_message->Data[0]<<24)|(int32_t)(rx_message->Data[1]<<16)\
            | (int32_t)(rx_message->Data[2]<<8) | (int32_t)(rx_message->Data[3]));
        }break;
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
    if(v->angle_diff < -7500) { v->round_count--; }
    else if(v->angle_diff > 7500) { v->round_count++; }
    // Calculate the encoder output in a continous value domain
    v->angle_continous = v->angle_raw + v->round_count * ENCODER_MAX;
    // Calculate the angle, range from negative infinite to positive infinite
    v->ecd_angle = (float) (v->angle_sign) * (v->angle_raw - v->angle_bias)* \
                    RADIAN_CIRCLE / ENCODER_MAX + v->round_count * RADIAN_CIRCLE;
}

void GetEncoderBias(volatile Encoder *v, CanRxMsg * rx_message)
{
    if (rx_message->StdId == 0x205) {
        switch (ROBOT_SERIAL_NUMBER) {
            case BLUE_SAMPLE_ROBOT_0: v->motor = &gimbal_0.yaw;  break;
            case RED_SAMPLE_ROBOT_1:  v->motor = &gimbal_1.yaw;  break;
            case SOLDIER_2:           v->motor = &gimbal_2.yaw;  break;
            case SOLDIER_3:           v->motor = &gimbal_3.yaw;  break;
            case SOLDIER_4:           v->motor = &gimbal_4.yaw;  break;
            case SOLDIER_5:           v->motor = &gimbal_5.yaw;  break;
            case BASE_ROBOT_6:        v->motor = &gimbal_6.yaw;  break;
            case HERO_ROBOT_CANNON_7: v->motor = &gimbal_7.yaw;  break;
            case HERO_ROBOT_TURRET_8: v->motor = &gimbal_8.yaw;  break;
            default:break;
        }
        v->angle_bias = map_motor(YAW_DEFAULT_RADIAN, v->motor);
        if (v->motor->ecd_high - v->motor->ecd_low > 0) { v->angle_sign = 1; }
        else { v->angle_sign = -1; }
    }
    else  if (rx_message->StdId == 0x206) {
        switch (ROBOT_SERIAL_NUMBER) {
            case BLUE_SAMPLE_ROBOT_0: v->motor = &gimbal_0.pitch; break;
            case RED_SAMPLE_ROBOT_1:  v->motor = &gimbal_1.pitch; break;
            case SOLDIER_2:           v->motor = &gimbal_2.pitch; break;
            case SOLDIER_3:           v->motor = &gimbal_3.pitch; break;
            case SOLDIER_4:           v->motor = &gimbal_4.pitch; break;
            case SOLDIER_5:           v->motor = &gimbal_5.pitch; break;
            case BASE_ROBOT_6:        v->motor = &gimbal_6.pitch; break;
            case HERO_ROBOT_CANNON_7: v->motor = &gimbal_7.pitch; break;
            case HERO_ROBOT_TURRET_8: v->motor = &gimbal_8.pitch; break;
            default:break;
        }
        v->angle_bias = map_motor(PITCH_DEFAULT_RADIAN, v->motor);
        if (v->motor->ecd_high - v->motor->ecd_low > 0) { v->angle_sign = 1; }
        else { v->angle_sign = -1; }
    }
    else {
        v->angle_bias = ( rx_message->Data[0] << 8 ) | rx_message->Data[1];
        v->angle_sign = 1;
        v->angle_continous = v->angle_bias;
        v->angle_raw_last = v->angle_bias;
    }
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
void Set_PitchYaw_Current() {
    tx_pitchyaw_message.Data[0] = motor_yaw_cur >> 8;
    tx_pitchyaw_message.Data[1] = motor_yaw_cur & 0xFF;
    tx_pitchyaw_message.Data[2] = motor_pitch_cur >> 8;
    tx_pitchyaw_message.Data[3] = motor_pitch_cur & 0xFF;
    tx_pitchyaw_message.Data[4] = 0x00;
    tx_pitchyaw_message.Data[5] = 0x00;
    tx_pitchyaw_message.Data[6] = 0x00;
    tx_pitchyaw_message.Data[7] = 0x00;
}

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

static int map_motor(int x, const motor_mapping_t * map)
{
  return (x - map->real_low) * (map->ecd_high - map->ecd_low)\
          / ( map->real_high - map->real_low) + map->ecd_low;
}
