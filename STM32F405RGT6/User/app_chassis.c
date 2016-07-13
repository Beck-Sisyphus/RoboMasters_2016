#include "main.h"

#define PID_CHASSIS true

extern int16_t measured_201_speed;
extern int16_t measured_202_speed;
extern int16_t measured_203_speed;
extern int16_t measured_204_speed;

extern int16_t measured_201_angle;
extern int16_t measured_202_angle;
extern int16_t measured_203_angle;
extern int16_t measured_204_angle;

const float v_p_201 = 1.0;
const float v_i_201 = 0;
const float v_d_201 = 0;

const float v_p_202 = 1.0;
const float v_i_202 = 0;
const float v_d_202 = 0;

const float v_p_203 = 1.0;
const float v_i_203 = 0;
const float v_d_203 = 0;

const float v_p_204 = 1.0;
const float v_i_204 = 0;
const float v_d_204 = 0;

int16_t PID_Control(float measured, float target, const float p, const float i, const float d)
{
    static float error_v[2] = {0.0,0.0};
    static float output = 0;
    static float inte = 0;

    if(abs(measured) < GAP)
    {
        measured = 0.0;
    }

    error_v[0] = error_v[1];
    error_v[1] = target - measured;
    inte += error_v[1];

    output = error_v[1] * p
            + inte * i
             + (error_v[1] - error_v[0]) * d;

    if(output > ESC_MAX) { output = ESC_MAX; }
    if(output < -ESC_MAX){ output = -ESC_MAX;}

    return -(int16_t)output; // For Blue rover, position reading is in inverse direction
}


int16_t Velocity_Control_201(float target_velocity_raw_201)
{
   int16_t motor_front_right_cur;
   #if PID_CHASSIS
      float target_velocity_201 = map(target_velocity_raw_201, -660, 660, -7780, 7780);
      motor_front_right_cur = PID_Control(measured_201_speed, target_velocity_201, v_p_201, v_i_201, v_d_201);
   #else
      motor_front_right_cur = 6 * target_velocity_201;
   #endif
   return motor_front_right_cur;
}

int16_t Velocity_Control_202(float target_velocity_202)
{
    int16_t motor_front_left_cur;
   #if PID_CHASSIS
      motor_front_left_cur = PID_Control(measured_202_speed, target_velocity_202, v_p_202, v_i_202, v_d_202);
   #else
      motor_front_left_cur = 6 * target_velocity_202;
   #endif
    return motor_front_left_cur;
}

int16_t Velocity_Control_203(float target_velocity_203)
{
    int16_t motor_back_left_cur;
   #if PID_CHASSIS
      motor_back_left_cur = PID_Control(measured_203_speed, target_velocity_203, v_p_203, v_i_203, v_d_203);
   #else
      motor_back_left_cur = 6 * target_velocity_203;
   #endif
    return motor_back_left_cur;
}

int16_t Velocity_Control_204(float target_velocity_204)
{
    int16_t motor_back_right_cur;
    #if PID_CHASSIS
       motor_back_right_cur = PID_Control(measured_204_speed, target_velocity_204, v_p_204, v_i_204, v_d_204);
    #else
       motor_back_right_cur = 6 * target_velocity_204;
    #endif
    return motor_back_right_cur;
}
