#ifndef _CAN_BUS_TASK_H_
#define _CAN_BUS_TASK_H_

#include <stm32f4xx.h>
#include "main.h"

// #define pitch_max 15.0
// #define yaw_max 720.0
#define RADIAN_CIRCLE 6283 // 2 * pi * 1000 as we defined in the system for angle

typedef struct Encoder
{
    int32_t round_count;
    int32_t velocity_raw;
    int32_t angle_raw;        // Raw reading from encoder without processing
    int32_t angle_raw_last;
    int32_t angle_diff;
    int32_t angle_continous;
    int32_t angle_bias;
    float ecd_angle;					// final angle in radian * 1000
}Encoder;

void CanReceiveMsgProcess(CanRxMsg * msg);
void EncoderProcess(volatile Encoder *v, CanRxMsg * rx_message);
void GetEncoderBias(volatile Encoder *v, CanRxMsg * rx_message);

void PitchYaw_Address_Setup(void);
void Wheels_Address_Setup(void);
void Set_PitchYaw_Current(void);
void Set_Wheels_Current(void);

void Motor_ManSet_Can_2(void);
void pitchyaw_control(int16_t, int16_t);
void wheel_control(int16_t, int16_t, int16_t, int16_t);
void Motor_Reset_Can_2(void);
// void GYRO_RST(void);
// void Encoder_sent(float encoder_angle);

#endif
