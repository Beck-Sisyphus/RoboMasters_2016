#include "main.h"
extern RC_Ctl_t RC_Ctl;
uint8_t Remote_On = 0;

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

    drive = RC_Ctl.rc.ch3 - RC_CH_VALUE_OFFSET;
    strafe = RC_Ctl.rc.ch2 - RC_CH_VALUE_OFFSET;
    rotate = RC_Ctl.rc.ch0 - RC_CH_VALUE_OFFSET;

    if(RC_Ctl.rc.s1 == RC_SW_UP && RC_Ctl.rc.s2 == RC_SW_UP) {
        drive = RC_Ctl.rc.ch3 - RC_CH_VALUE_OFFSET;
        strafe = RC_Ctl.rc.ch2 - RC_CH_VALUE_OFFSET;
        rotate = RC_Ctl.rc.ch0 - RC_CH_VALUE_OFFSET;

    } else if(RC_Ctl.rc.s1 == RC_SW_DOWN && RC_Ctl.rc.s2 == RC_SW_DOWN) {
        if(RC_Ctl.rc.ch3 > RC_CH_VALUE_OFFSET) {
            pitch = -10 * (RC_Ctl.rc.ch3 - RC_CH_VALUE_OFFSET);
        } else if(RC_Ctl.rc.ch3 < RC_CH_VALUE_OFFSET) {
            pitch = 10 * (RC_Ctl.rc.ch3 - RC_CH_VALUE_OFFSET);
        }

        yaw = -2 * (RC_Ctl.rc.ch0 - RC_CH_VALUE_OFFSET);
    }


    if(Remote_On == 1) {
        if(RC_Ctl.rc.s1 == RC_SW_UP && RC_Ctl.rc.s2 == RC_SW_UP) {
            wheel_control(drive, strafe, rotate);
        } else if(RC_Ctl.rc.s1 == RC_SW_DOWN && RC_Ctl.rc.s2 == RC_SW_DOWN) {
            pitchyaw_control(yaw, pitch);
        } else {
            Motor_Reset_Can_2();
        }
    }
}
