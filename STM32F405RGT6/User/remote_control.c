#include "main.h"
extern RC_Ctl_t RC_Ctl;
uint8_t Remote_On = 0;


int16_t drive;
int16_t strafe;
int16_t rotate;
int16_t pitch;
int16_t yaw;

extern int16_t pitch_Position;
extern int16_t yaw_Position;
extern int16_t pitch_Velocity;
extern int16_t yaw_Velocity;

// for velocity controlling pitch and yaw with remote
extern int16_t remote_pitch_change;
extern int16_t remote_yaw_change;


/*************************************************************************
              Code to Enable cannon to be driven with remote
*************************************************************************/
void Remote_Control() {
    // int16_t drive;
    // int16_t strafe;
    // int16_t rotate;
    // int16_t pitch;
    // int16_t yaw;

    pitch = remote_pitch_change;
    yaw = remote_yaw_change;

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

        // Position control with remote
        // pitch = 79 + floor((RC_Ctl.rc.ch3 - RC_CH_VALUE_OFFSET) / 18.6);
        // yaw = floor((RC_CH_VALUE_OFFSET - RC_Ctl.rc.ch2) / 6.4);


        // Velocity control with remote
        pitch = (RC_Ctl.rc.ch3 - RC_CH_VALUE_OFFSET) / 10;  
        yaw = (RC_Ctl.rc.ch2 - RC_CH_VALUE_OFFSET) / 10;      


        // if(RC_Ctl.rc.ch3 > RC_CH_VALUE_OFFSET) {
        //     pitch = -10 * (RC_Ctl.rc.ch3 - RC_CH_VALUE_OFFSET);
        // } else if(RC_Ctl.rc.ch3 < RC_CH_VALUE_OFFSET) {
        //     pitch = 10 * (RC_Ctl.rc.ch3 - RC_CH_VALUE_OFFSET);
        // }

        // yaw = -2 * (RC_Ctl.rc.ch0 - RC_CH_VALUE_OFFSET);
    }


    if(Remote_On == 1) {
        if(RC_Ctl.rc.s1 == RC_SW_UP && RC_Ctl.rc.s2 == RC_SW_UP) {
            wheel_control(drive, strafe, rotate);
        } else if(RC_Ctl.rc.s1 == RC_SW_DOWN && RC_Ctl.rc.s2 == RC_SW_DOWN) {
            // float yaw_velocity_change = Velocity_Control_206((float)MPU6050_Real_Data.Gyro_Z, 0);
            // pitchyaw_control((int16_t) yaw_velocity_change, 0);
            pitch_Position = remote_pitch_change;
            yaw_Position = remote_yaw_change;
            // yaw_Velocity   = yaw;
            // pitch_Position = pitch;
            // yaw_Position = yaw;
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
