#ifndef __REMOTE_TASK_H__
#define __REMOTE_TASK_H__
#include <stm32f4xx.h>

//remote data process
typedef struct
{
    float pitch_angle_dynamic_ref;
    float yaw_angle_dynamic_ref;
    float pitch_angle_static_ref;
    float yaw_angle_static_ref;
    float pitch_speed_ref;
    float yaw_speed_ref;
}Gimbal_Ref_t;

int min(int, int);
int max(int, int);
void Remote_Control(void);
int round_div(int, float);
#endif
