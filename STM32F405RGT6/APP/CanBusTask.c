#include "main.h"
#include "CanBusTask.h"

static uint32_t can_count = 0;

volatile Encoder CM1Encoder = {0,0,0,0,0,0,0,0,0};
volatile Encoder CM2Encoder = {0,0,0,0,0,0,0,0,0};
volatile Encoder CM3Encoder = {0,0,0,0,0,0,0,0,0};
volatile Encoder CM4Encoder = {0,0,0,0,0,0,0,0,0};
volatile Encoder GMYawEncoder = {0,0,0,0,0,0,0,0,0};
volatile Encoder GMPitchEncoder = {0,0,0,0,0,0,0,0,0};
float ZGyroModuleAngle = 0.0f;
extern GimbalCaliStruct_t GimbalSavedCaliData;

/*
***********************************************************************************************
*Name          :GetEncoderBias
*Input         :can message
*Return        :void
*Description   :to get the initiatial encoder of the chassis motor 201 202 203 204
*
*
***********************************************************************************************
*/

void GetEncoderBias(volatile Encoder *v, CanRxMsg * msg)
{
    v->ecd_bias = (msg->Data[0]<<8)|msg->Data[1];  //保存初始编码器值作为偏差
    v->ecd_value = v->ecd_bias;
    v->last_raw_value = v->ecd_bias;
    v->temp_count++;
}

/*
***********************************************************************************************
*Name          :EncoderProcess
*Input         :can message
*Return        :void
*Description   :to get the initiatial encoder of the chassis motor 201 202 203 204
*
*
***********************************************************************************************
*/
void EncoderProcess(volatile Encoder *v, CanRxMsg * msg)
{
    int i=0;
    int32_t temp_sum = 0;
    v->last_raw_value = v->raw_value;
    v->raw_value = (msg->Data[0]<<8)|msg->Data[1];
    v->diff = v->raw_value - v->last_raw_value;
    //两次编码器的反馈值差别太大，表示圈数发生了改变
    // If the feeback from the encoder changes too much, the rotation is incremented
    if(v->diff < -7500)
    {
      	v->round_cnt++;
      	v->ecd_raw_rate = v->diff + 8192;
    }
    else if(v->diff>7500)
    {
      	v->round_cnt--;
      	v->ecd_raw_rate = v->diff- 8192;
    }
    else
    {
      	v->ecd_raw_rate = v->diff;
    }
    //计算得到连续的编码器输出值
    // Calculate the encoder output in a continous value domain
    v->ecd_value = v->raw_value + v->round_cnt * 8192;
    //计算得到角度值，范围正负无穷大
    // Calculate the angle, range from negative infinite to positive infinite
    v->ecd_angle = (float)(v->raw_value - v->ecd_bias)*360/8192 + v->round_cnt * 360;
    v->rate_buf[v->buf_count++] = v->ecd_raw_rate;
    if(v->buf_count == RATE_BUF_SIZE)
    {
      	v->buf_count = 0;
    }
    //计算速度平均值
    // Calculate the average speed
    for(i = 0;i < RATE_BUF_SIZE; i++)
    {
      	temp_sum += v->rate_buf[i];
    }
    v->filter_rate = (int32_t)(temp_sum/RATE_BUF_SIZE);
}

/*
************************************************************************************************************************
*Name        : CanReceiveMsgProcess
* Description: This function process the can message representing the encoder data received from the CAN2 bus.
* Arguments  : msg     is a pointer to the can message.
* Returns    : void
* Note(s)    : none
************************************************************************************************************************
*/
void CanReceiveMsgProcess(CanRxMsg * msg)
{
    //GMYawEncoder.ecd_bias = yaw_ecd_bias;
    can_count++;
		switch(msg->StdId)
		{
				case CAN_BUS2_MOTOR1_FEEDBACK_MSG_ID:
				{   (can_count<=50) ? GetEncoderBias(&CM1Encoder ,msg):EncoderProcess(&CM1Encoder ,msg);       //获取到编码器的初始偏差值
				}break;
				case CAN_BUS2_MOTOR2_FEEDBACK_MSG_ID:
				{   (can_count<=50) ? GetEncoderBias(&CM2Encoder ,msg):EncoderProcess(&CM2Encoder ,msg);
				}break;
				case CAN_BUS2_MOTOR3_FEEDBACK_MSG_ID:
				{   (can_count<=50) ? GetEncoderBias(&CM3Encoder ,msg):EncoderProcess(&CM3Encoder ,msg);
				}break;
				case CAN_BUS2_MOTOR4_FEEDBACK_MSG_ID:
				{   (can_count<=50) ? GetEncoderBias(&CM4Encoder ,msg):EncoderProcess(&CM4Encoder ,msg);
				}break;
				case CAN_BUS2_MOTOR5_FEEDBACK_MSG_ID:
				{
            EncoderProcess(&GMYawEncoder ,msg);

            // 比较保存编码器的值和偏差值，如果编码器的值和初始偏差之间差距超过阈值，将偏差值做处理，防止出现云台反方向运动
            // Compare the saved encoder value and bias value, if the difference excceed the threshold, adjust the difference to provent gimbal to move the opposite direction
            // if(can_count>=90 && can_count<=100)
            //准备阶段要求二者之间的差值一定不能大于阈值，否则肯定是出现了临界切换
            // In the prepare state the difference can't excceed the threshold, or something must be wrong
            if(GetWorkState() == PREPARE_STATE)
            {
                if((GMYawEncoder.ecd_bias - GMYawEncoder.ecd_value) <-4000)
                {
                    GMYawEncoder.ecd_bias = GimbalSavedCaliData.GimbalYawOffset + 8192;
                }
                else if((GMYawEncoder.ecd_bias - GMYawEncoder.ecd_value) > 4000)
                {
                    GMYawEncoder.ecd_bias = GimbalSavedCaliData.GimbalYawOffset - 8192;
                }
            }
				}break;
				case CAN_BUS2_MOTOR6_FEEDBACK_MSG_ID:
				{
            EncoderProcess(&GMPitchEncoder ,msg);

            if(can_count<=100)
            {
                if((GMPitchEncoder.ecd_bias - GMPitchEncoder.ecd_value) <-4000)
                {
                    GMPitchEncoder.ecd_bias = GimbalSavedCaliData.GimbalPitchOffset + 8192;
                }
                else if((GMPitchEncoder.ecd_bias - GMPitchEncoder.ecd_value) > 4000)
                {
                    GMPitchEncoder.ecd_bias = GimbalSavedCaliData.GimbalPitchOffset - 8192;
                }
            }
				}break;
				case CAN_BUS1_ZGYRO_FEEDBACK_MSG_ID:
				{    ZGyroModuleAngle = -0.01f*((int32_t)(msg->Data[0]<<24)|(int32_t)(msg->Data[1]<<16) | (int32_t)(msg->Data[2]<<8) | (int32_t)(msg->Data[3]));
				}break;
				default:
				{
				}
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
void Set_Gimbal_Current(int16_t yaw_current, int16_t pitch_current) {

  PitchYaw_Address_Setup();
  motor_yaw_cur = yaw_current;
  motor_pitch_cur = pitch_current;
  Set_PitchYaw_Current();
  CAN_Transmit(CAN2,&tx_pitchyaw_message);
}

// controls wheels using kinematic equations
void wheel_control(int16_t drive, int16_t strafe, int16_t rotate)
{
    int16_t motor_201_pos = (-1*drive + strafe + rotate);
    int16_t motor_204_pos = (-1*drive - strafe + rotate);
    int16_t motor_202_pos = (drive + strafe + rotate);
    int16_t motor_203_pos = (drive - strafe + rotate);

    motor_front_right_cur = CHASSIS_MOTOR_STRENGTH * motor_201_pos;
    motor_back_right_cur = CHASSIS_MOTOR_STRENGTH * motor_204_pos;
    motor_front_left_cur = CHASSIS_MOTOR_STRENGTH * motor_202_pos;
    motor_back_left_cur = CHASSIS_MOTOR_STRENGTH * motor_203_pos;

    Wheels_Address_Setup();
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
