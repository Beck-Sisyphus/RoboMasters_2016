#ifndef _CAN_BUS_TASK_H_
#define _CAN_BUS_TASK_H_

#include <stm32f4xx.h>
#include "main.h"

// #define pitch_max 15.0
// #define yaw_max 720.0
#define YAW_DEFAULT_RADIAN 0
#define PITCH_DEFAULT_RADIAN 0 // 90 degree in radians
#define RADIAN_CIRCLE 6283 // 2 * pi * 1000 as we defined in the system for angle
typedef struct motor_mapping_t
{
  int real_low;
  int real_high;
  int ecd_low;
  int ecd_high;
}motor_mapping_t;

typedef struct gimbal_mapping_t
{
  motor_mapping_t pitch;
  motor_mapping_t yaw;
}gimbal_mapping_t;

typedef struct Encoder
{
    int32_t round_count;
    int16_t velocity_raw;
    int16_t angle_raw;        // Raw reading from encoder without processing
    int16_t angle_raw_last;
    int32_t angle_diff;
    int32_t angle_continous;
    int32_t angle_bias;
    int16_t angle_sign;
    struct motor_mapping_t * motor;
    float ecd_angle;					// final angle in radian * 1000
}Encoder;

// measured by protractor, in radian * 1000, in aircraft coordinate
// CAN bus feedback for both RM6025 and RM6623
#define GIMBAL_BLUE_SAMPLE_ROBOT_0 {\
	{\
    -821,\
    419,\
    7530,\
    6244,\
  },\
  {\
    -1798,\
    1798,\
    5157,\
    489,\
  },\
}\

#define GIMBAL_RED_SAMPLE_ROBOT_1 {\
  {\
    -821,\
    419,\
    5238,\
    4039,\
  },\
  {\
    -1798,\
    1798,\
    4640,\
    0,\
  },\
}\

// 83 degree, 135 degree, autonomous mode
#define GIMBAL_SOLDIER_2 {\
  {\
    -122,\
    785,\
    5430,\
    6651,\
  },\
  {\
    -1571,\
    1571,\
    4451,\
    349,\
  },\
}\

// -28 degree，135 degree
// Last update: 2016/8/20, Beck Pang
#define GIMBAL_SOLDIER_3 {\
  {\
    -489,\
    785,\
    6245,\
    7524,\
  },\
  {\
    -1147,\
    1571,\
    8172,\
    4547,\
  },\
}\

// -28 degree，135 degree
// Last update: 2016/8/20, Beck Pang
#define GIMBAL_SOLDIER_4 {\
  {\
    -489,\
    785,\
    1935,\
    2883,\
  },\
  {\
    -1571,\
    1571,\
    5932,\
    1758,\
  },\
}\

// 72 degree，135 degree
#define GIMBAL_SOLDIER_5 {\
  {\
    -314,\
    785,\
    6245,\
    7501,\
  },\
  {\
    -1571,\
    1571,\
    4737,\
    620,\
  },\
}\

// 52 degree，130 degree
#define GIMBAL_HERO_ROBOT_CANNON_7 {\
  {\
    -664,\
    698,\
    7480,\
    9347,\
  },\
  {\
    -1798,\
    1798,\
    5171,\
    470,\
  },\
}\

#define GIMBAL_DEFAULT {\
  {\
    0,\
    0,\
    0,\
    0,\
  },\
  {\
    0,\
    0,\
    0,\
    0,\
  },\
}\

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
static int map_motor(int x, const motor_mapping_t * map);

#endif
