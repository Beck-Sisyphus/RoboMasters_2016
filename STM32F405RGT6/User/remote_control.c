#include "main.h"
extern RC_Ctl_t RC_Ctl;
uint8_t Remote_On = 0;

volatile extern int16_t pitch_Position;
volatile extern int16_t yaw_Position;
volatile extern int16_t pitch_Velocity;
volatile extern int16_t yaw_Velocity;

// for velocity controlling pitch and yaw with remote
volatile extern int16_t remote_pitch_change;
volatile extern int16_t remote_yaw_change;


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

    if(RC_Ctl.rc.s1 == RC_SW_UP && RC_Ctl.rc.s2 == RC_SW_UP) {
        drive = RC_Ctl.rc.ch3 - RC_CH_VALUE_OFFSET;
        strafe = RC_Ctl.rc.ch2 - RC_CH_VALUE_OFFSET;
        rotate = RC_Ctl.rc.ch0 - RC_CH_VALUE_OFFSET;

    } else if(RC_Ctl.rc.s1 == RC_SW_DOWN && RC_Ctl.rc.s2 == RC_SW_DOWN) {
        // Velocity control with remote
        pitch = (RC_Ctl.rc.ch3 - RC_CH_VALUE_OFFSET) / 10;  
        yaw = (RC_Ctl.rc.ch2 - RC_CH_VALUE_OFFSET) / 10;   
    }


    if(Remote_On == 1) {
        if(RC_Ctl.rc.s1 == RC_SW_UP && RC_Ctl.rc.s2 == RC_SW_UP) {
            wheel_control(drive, strafe, rotate);
        } else if(RC_Ctl.rc.s1 == RC_SW_DOWN && RC_Ctl.rc.s2 == RC_SW_DOWN) {
            remote_pitch_change = pitch;
            remote_yaw_change = yaw;
            // pitch_Position = pitch;
            // yaw_Position = yaw;
        } else {
            // Motor_Reset_Can_2();
            wheel_control(0, 0, 0);
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
