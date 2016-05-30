#include "main.h"

Gimbal_Ref_t GimbalRef;
volatile extern RC_Ctl_t RC_Ctl;

volatile int manual_Control_Turret = 0;
volatile extern arduino_data data_usart_3;

static uint8_t Remote_On = 0;
/*************************************************************************
              Code to Enable cannon to be driven with remote
*************************************************************************/
void Remote_Control() {
    int16_t drive;
    int16_t strafe;
    int16_t rotate;
    int16_t pitch;
    int16_t yaw;

	  // To see if remote is off or not
    if (RC_Ctl.rc.ch2 < RC_CH_VALUE_MIN
        || RC_Ctl.rc.ch3 < RC_CH_VALUE_MIN
        ) {
        Remote_On = 0;
    } else {
        Remote_On = 1;
    }

    if(Remote_On == 1) {
        if(RC_Ctl.rc.s1 == RC_SW_UP && RC_Ctl.rc.s2 == RC_SW_UP) {
            drive = RC_Ctl.rc.ch3 - RC_CH_VALUE_OFFSET;
            strafe = RC_Ctl.rc.ch2 - RC_CH_VALUE_OFFSET;
            rotate = RC_Ctl.rc.ch0 - RC_CH_VALUE_OFFSET;
            wheel_control(drive, strafe, rotate);
            GimbalRef.pitch_angle_dynamic_ref = data_usart_3.packet.pitch_req;
            GimbalRef.yaw_angle_dynamic_ref = data_usart_3.packet.yaw_req;
            manual_Control_Turret = 0;
        } else if(RC_Ctl.rc.s1 == RC_SW_DOWN && RC_Ctl.rc.s2 == RC_SW_DOWN) {
            pitch = (RC_Ctl.rc.ch3 - RC_CH_VALUE_OFFSET) / 100;
            yaw = (RC_Ctl.rc.ch2 - RC_CH_VALUE_OFFSET) / 100;
            manual_Control_Turret = 1;
            GimbalRef.pitch_angle_dynamic_ref += pitch;
            GimbalRef.yaw_angle_dynamic_ref += yaw;
        } else {
            // Motor_Reset_Can_2();
            wheel_control(0, 0, 0);
            GimbalRef.pitch_angle_dynamic_ref = data_usart_3.packet.pitch_req;
            GimbalRef.yaw_angle_dynamic_ref = data_usart_3.packet.yaw_req;
            manual_Control_Turret = 0;
        }
    }
}

int min(int a, int b) {
    if(a<b) {
        return a;
    } else {
        return b;
    }
}

int max(int a, int b) {
    if(a>b) {
        return a;
    } else {
        return b;
    }
}



// division rounding
// for smoothening remote control of pitch and yaw
int round_div(int dividend, float divisor)
{
    return (int) ((dividend + (divisor / 2)) / divisor);
}
