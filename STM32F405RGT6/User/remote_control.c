#include "main.h"
volatile extern RC_Ctl_t RC_Ctl;
uint8_t Remote_On = 0;
volatile int manual_Control_Turret = 0;

volatile extern int16_t pitch_Position;
volatile extern int16_t yaw_Position;
// volatile extern int16_t pitch_Velocity;
// volatile extern int16_t yaw_Velocity;

// for velocity controlling pitch and yaw with remote
volatile extern int16_t remote_pitch_change;
volatile extern int16_t remote_yaw_change;
volatile extern arduino_data data_usart_3;


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
        drive = RC_Ctl.rc.ch3 - RC_CH_VALUE_OFFSET;
        strafe = RC_Ctl.rc.ch2 - RC_CH_VALUE_OFFSET;
        rotate = RC_Ctl.rc.ch0 - RC_CH_VALUE_OFFSET;
        if(RC_Ctl.rc.s1 == RC_SW_UP && RC_Ctl.rc.s2 == RC_SW_UP) {
            wheel_control(drive, strafe, rotate);
            pitch_Position = data_usart_3.packet.pitch_req;
            yaw_Position = data_usart_3.packet.yaw_req;
            manual_Control_Turret = 0;
        } else if(RC_Ctl.rc.s1 == RC_SW_DOWN && RC_Ctl.rc.s2 == RC_SW_DOWN) {
            wheel_control(drive, strafe, 0);
            pitch = (RC_Ctl.rc.ch1 - RC_CH_VALUE_OFFSET) / 100;
            yaw = (RC_Ctl.rc.ch0 - RC_CH_VALUE_OFFSET) / 100;
            manual_Control_Turret = 1;
            pitch_Position += pitch;
            pitch_Position = min(pitch_Position, REAL_CANNON_PITCH_HIGH);
            pitch_Position = max(pitch_Position, REAL_CANNON_PITCH_LOW);
            yaw_Position += yaw;
            yaw_Position = min(yaw_Position, REAL_CANNON_YAW_HIGH);
            yaw_Position = max(yaw_Position, REAL_CANNON_YAW_LOW);
        } else if (RC_Ctl.rc.s2 == RC_SW_MID) {
          /* code */
        } else {
            // Motor_Reset_Can_2();
            wheel_control(0, 0, 0);
            pitch_Position = data_usart_3.packet.pitch_req;
            yaw_Position = data_usart_3.packet.yaw_req;
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
