#include "main.h"
extern RC_Ctl_t RC_Ctl;
uint8_t Remote_On = 0;

extern int16_t pitch_Position;
extern int16_t yaw_Position;
extern int16_t pitch_Velocity;
extern int16_t yaw_Velocity;

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
        // if(pitch >= (REAL_PITCH_HIGH + REAL_PITCH_LOW) / 2) {
        //     pitch = min(pitch + RC_Ctl.rc.ch3 -  RC_CH_VALUE_OFFSET , REAL_PITCH_HIGH);
        // } else {
        //     pitch = max(pitch + RC_Ctl.rc.ch3 -  RC_CH_VALUE_OFFSET , REAL_PITCH_LOW);
        // }
        // if(yaw >= 0) {
        //     yaw = min(yaw + RC_Ctl.rc.ch2 -  RC_CH_VALUE_OFFSET , REAL_YAW_HIGH);
        // } else if(yaw < 0) {
        //     yaw = max(yaw + RC_Ctl.rc.ch2 -  RC_CH_VALUE_OFFSET , REAL_YAW_LOW);
        // }

        pitch += RC_Ctl.rc.ch3 -  RC_CH_VALUE_OFFSET;
        pitch = min(pitch, REAL_PITCH_HIGH);
        pitch = max(pitch, REAL_PITCH_LOW);
        yaw += RC_Ctl.rc.ch2 -  RC_CH_VALUE_OFFSET;
        yaw = min(yaw, REAL_YAW_HIGH);
        yaw = min(yaw, REAL_YAW_LOW);
    }


    if(Remote_On == 1) {
        if(RC_Ctl.rc.s1 == RC_SW_UP && RC_Ctl.rc.s2 == RC_SW_UP) {
            wheel_control(drive, strafe, rotate);
        } else if(RC_Ctl.rc.s1 == RC_SW_DOWN && RC_Ctl.rc.s2 == RC_SW_DOWN) {
            pitch_Position = pitch;
            yaw_Position = yaw;
        } else {
            Motor_Reset_Can_2();
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
