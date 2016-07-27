#include "main.h"

#define MOUSE_SENSITIVITY (10)

volatile extern RC_Ctl_t RC_Ctl;
uint8_t Remote_On = 0;
// global control information req
volatile int16_t drive;
volatile int16_t strafe;
volatile int16_t rotate;
volatile extern int16_t pitch_Position;
volatile extern int16_t yaw_Position;
volatile int16_t friction_motor_state;
volatile int16_t feeder_motor_state;
volatile int manual_Control_Turret = 0;


// volatile extern int16_t pitch_Velocity;
// volatile extern int16_t yaw_Velocity;

// for velocity controlling pitch and yaw with remote
// volatile extern int16_t remote_pitch_change;
// volatile extern int16_t remote_yaw_change;
volatile extern arduino_data data_usart_3;

#if 0
/*************************************************************************
              Code to Enable cannon to be driven with remote
*************************************************************************/
void Remote_Control() {
    // int16_t drive;
    // int16_t strafe;
    // int16_t rotate;
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
        // drive = RC_Ctl.rc.ch3 - RC_CH_VALUE_OFFSET;
        // strafe = RC_Ctl.rc.ch2 - RC_CH_VALUE_OFFSET;
        // rotate = RC_Ctl.rc.ch0 - RC_CH_VALUE_OFFSET;
        if(RC_Ctl.rc.s1 == RC_SW_UP && RC_Ctl.rc.s2 == RC_SW_UP) {
            // wheel_control(drive, strafe, rotate);
            drive = RC_Ctl.rc.ch3 - RC_CH_VALUE_OFFSET;
            strafe = RC_Ctl.rc.ch2 - RC_CH_VALUE_OFFSET;
            rotate = RC_Ctl.rc.ch0 - RC_CH_VALUE_OFFSET;
            pitch_Position = data_usart_3.packet.pitch_req;
            yaw_Position = data_usart_3.packet.yaw_req;
            manual_Control_Turret = 0;
        } else if(RC_Ctl.rc.s1 == RC_SW_DOWN && RC_Ctl.rc.s2 == RC_SW_DOWN) {
            // wheel_control(drive, strafe, 0);
            drive = RC_Ctl.rc.ch3 - RC_CH_VALUE_OFFSET;
            strafe = RC_Ctl.rc.ch2 - RC_CH_VALUE_OFFSET;
            rotate = 0;
            pitch = (RC_Ctl.rc.ch1 - RC_CH_VALUE_OFFSET) / 100;
            yaw = (RC_Ctl.rc.ch0 - RC_CH_VALUE_OFFSET) / 100;
            manual_Control_Turret = 1;
            pitch_Position += pitch;
            pitch_Position = min(pitch_Position, REAL_CANNON_PITCH_HIGH);
            pitch_Position = max(pitch_Position, REAL_CANNON_PITCH_LOW);
            yaw_Position += yaw;
            yaw_Position = min(yaw_Position, REAL_CANNON_YAW_RIGHT);
            yaw_Position = max(yaw_Position, REAL_CANNON_YAW_LEFT);
        } else if (RC_Ctl.rc.s2 == RC_SW_MID) {
          /* code */
        } else {
            // Motor_Reset_Can_2();
            // wheel_control(0, 0, 0);
            drive = 0;
            strafe = 0;
            rotate = 0;
            pitch_Position = data_usart_3.packet.pitch_req;
            yaw_Position = data_usart_3.packet.yaw_req;
            manual_Control_Turret = 0;
        }
    }
}
#endif

#define ROBOT_STATE_MANUAL (0)
#define ROBOT_STATE_SEMI_AUTO (1)
#define ROBOT_STATE_SPECIAL (2)



uint8_t remote_on = 0;
uint8_t robot_state = ROBOT_STATE_MANUAL;
#if 1
// this is run by tim2
void Remote_Control() {

    if (RC_Ctl.rc.ch2 < RC_CH_VALUE_MIN || RC_Ctl.rc.ch3 < RC_CH_VALUE_MIN) {
        remote_on = 0;
    } else {
        remote_on = 1;
    }

    if (remote_on) {
        // (1) read switch positions and store into state
        // s1 for robot state
        switch (RC_Ctl.rc.s1) {
            case RC_SW_UP:
                robot_state = ROBOT_STATE_MANUAL;
                break;
            case RC_SW_MID:
                robot_state = ROBOT_STATE_SEMI_AUTO;
                break;
            case RC_SW_DOWN:
                robot_state = ROBOT_STATE_SPECIAL;
                break;
        }

        // (2) set global control info requests
        switch (robot_state) {
            case ROBOT_STATE_MANUAL:
                if (ROBOT_SERIAL_NUMBER != HERO_ROBOT_CANNON_7) {
                    // driving
                    drive = RC_Ctl.rc.ch3 - RC_CH_VALUE_OFFSET;
                    strafe = RC_Ctl.rc.ch2 - RC_CH_VALUE_OFFSET;
                    rotate = (RC_Ctl.rc.ch0 - RC_CH_VALUE_OFFSET) + RC_Ctl.mouse.x * MOUSE_SENSITIVITY;

                    // shooting
                    friction_motor_state = (RC_Ctl.rc.s2 == RC_SW_MID || RC_Ctl.rc.s2 == RC_SW_DOWN);
                    feeder_motor_state = (RC_Ctl.rc.s2 == RC_SW_DOWN || RC_Ctl.mouse.press_l);

                    // aiming (untested) todo: limit these two
                    pitch_Position += (RC_Ctl.rc.ch1 - RC_CH_VALUE_OFFSET) / 100;
                    yaw_Position += (RC_Ctl.rc.ch0 - RC_CH_VALUE_OFFSET) / 100;
                }
                break;
            case ROBOT_STATE_SEMI_AUTO:
                if (ROBOT_SERIAL_NUMBER != HERO_ROBOT_CANNON_7) {
                    // driving
                    drive = RC_Ctl.rc.ch3 - RC_CH_VALUE_OFFSET;
                    strafe = RC_Ctl.rc.ch2 - RC_CH_VALUE_OFFSET;
                    rotate = RC_Ctl.rc.ch0 - RC_CH_VALUE_OFFSET;

                    // shooting
                    friction_motor_state = data_usart_3.packet.friction_motor_state;
                    feeder_motor_state = data_usart_3.packet.feeder_motor_state;

                    // aiming
                    pitch_Position = data_usart_3.packet.pitch_req;
                    yaw_Position = data_usart_3.packet.yaw_req;
                }
                break;
            case ROBOT_STATE_SPECIAL:
                if (ROBOT_SERIAL_NUMBER != HERO_ROBOT_CANNON_7) {
                    // driving
                    drive = data_usart_3.packet.drive_req;
                    strafe = data_usart_3.packet.strafe_req;
                    rotate = data_usart_3.packet.rotate_req;

                    // shooting
                    friction_motor_state = data_usart_3.packet.friction_motor_state;
                    feeder_motor_state = data_usart_3.packet.feeder_motor_state;

                    // aiming
                    pitch_Position = data_usart_3.packet.pitch_req;
                    yaw_Position = data_usart_3.packet.yaw_req;
                } else {
                    // shooting is controlled by the arduino

                    // aiming with received data from arduino
                    pitch_Position += (RC_Ctl.rc.ch1 - RC_CH_VALUE_OFFSET) / 100;
                    yaw_Position += (RC_Ctl.rc.ch0 - RC_CH_VALUE_OFFSET) / 100;
                }
                break;
        }
    }
    if (data_usart_3.packet.js_real_chassis_out_power > 10) {
        LED2_ON();
    } else {
        LED2_OFF();
    }
}
#endif

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
